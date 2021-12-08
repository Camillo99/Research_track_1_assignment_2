/*
 * command to launch the map and the robot
 * rosrun stage_ros stageros $(rospack find second_assignment)/world/my_world.world
*/
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "iostream"
#include "math.h"
#include "a2_code/Userparameter.h"
using namespace std;


//publisher for the linear and angular velocity of the robot
ros::Publisher vel_pub;

//lenght of the distance vector given the base_scan topic of the robot
const int dim = 720;

//parameter for the control function
const float k_linear = 0.4;
const float k_angular = 1;

//set initial angular velocity
float vel_ang = 0;
//angular velocity of the previus step
//necessary for the closed-loop control system
float vel_pr_step = vel_ang;



//parameter given by the user
//multiplier of the linear velocity given to the robot
float parameter = 1;

/* find index of the maximum values in the distance vector
 * then "correct" the value of the index in order to obtain better 
 * controllability
 *  
 * 
 */
int find_max_index( float distance_vector[] ){
    
    int index = 0;
    float max = 0;
    for( int i = 0;i<dim;i++){
        if(distance_vector[i] >= max){
            max = distance_vector[i];
            index = i;
        }
    }
    //if the robot, on its sides, is close to an obstacle it tends to 
    //turn a bit farer from the obstacle pointing some degree on the other side
    //90index ~= 22.5 degree
    if(distance_vector[0] < distance_vector[dim-1]){
        index = index + 90;
    }
    else{
        index = index - 90;
    }
    return index;
}


/*  convert the index value in an equivalent value in radiant
 *  following the below rules
 *  index = 0 -> -pi/2
 *  index = 719 -> pi/2
 *  index = 360 -> 0
 *  and so the following linear transformation
 *  alpha = (index*(pi/719))-(pi/2)
 */ 
float convert_index_into_an_angle(int index){
    return (index*(M_PI/719.0))-(M_PI/2.0);
}

/*  proportional controller for linear velocity given distance
 *  of the obstacle in from of the robot
 * 
 *  in front of the robot -> index of distance_vector == 360
 *  velocity is multiplied by the parameter given by the user
 */
float linear_controller( float distance_vector[] ){
    
    int index = 360;
    float distance = distance_vector[index];
    
    //proportional control law
    float velocity = distance*k_linear * parameter;
    
    //print actual value of the multipier and actual surge speed
    cout<<"actual multiplier : "<<parameter<<" actual speed : "<<velocity<<endl; 
    
    return velocity;
}    


/*  proportional controller for angular velocity
 *  this controller has one element of memory
 *  it uses the angular velocity given in the previos step as a parameter
 *  for computing angular velocity for current step
 *  the idea is to allign the surge of the robot with the distance vector 
 *  between the robot and the farest mesured obstable (with some correction)
 */
float angular_controller( float distance_vector[] ){
    
    //get the index of the farest mesured obstable (with correction)
    int max_index = find_max_index(distance_vector);
    
    //convert index into an angle
    float alpha = convert_index_into_an_angle(max_index);
   
    //angular velocity closed-loop control law
    vel_ang = (vel_pr_step + k_angular)* alpha;
    
    //store the value of computed angular velocity in memory for the next iteration
    vel_pr_step = vel_ang;
    
    return vel_ang;
}


/* read values from the LaserScan topic and save in a vector
 * call the 2 control function
 * publish the computed linear and angular velocity on the cmd_vel topic
 */
void robotCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    //creates and fills distance vector for the element taken from the topic
    float distance_vector[dim];
    for (int i = 0;i < dim ;i++){
        distance_vector[i] = msg->ranges[i];
    }
    
    //initialize variables
    float lin_vel = 0;
    float ang_vel = 0;
  
    //call controller for linear velocty
    lin_vel = linear_controller(distance_vector);
    //call controller for angular velocity
    ang_vel = angular_controller(distance_vector);
  
    //publish on cmd_vel topic linear and angular velocity
    geometry_msgs::Twist vel;
    vel.linear.x = lin_vel;
    vel.angular.z = ang_vel;
    vel_pub.publish(vel);    
}

/* server for handling the interaction with the user
 * control the increase or decrease of the multiplier given by the user
 * if input == 1 -> increase
 * if input == -1 -> decrease
 */
bool velocity_parameter(a2_code::Userparameter::Request &req, a2_code::Userparameter::Response &res){
    
    if( req.user_value !=0 ){
        
        if( req.user_value > 0 ){
            //increase
            parameter = parameter + 0.2;
        }
        else{
            //decrease
            if( parameter > 0.2 )
                parameter = parameter - 0.2;
        }   
        //gives as response the actual value of the parameter
        res.multiplier = parameter;
        return true;
    }
    
    return true;
}

/* initialize the ros node
 * and call the subscriber, the pubblisher, and the server for the service
 * 
 */
int main(int argc, char **argv){
    
    //ROS_INFO("main started");
    
    //Initialize the node , setup the NodeHandle for handling the communication with the ROS system
	ros::init( argc , argv , "control_node");
	ros::NodeHandle nh;
    
    // Define the subscriber to LaserScan ropic
	ros::Subscriber laser_sub = nh.subscribe ("/base_scan", 1,robotCallback);
    // define the pubblisher for cmd_vel
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    //define the server for the userparameter service
    ros::ServiceServer service= nh.advertiseService("/userparameter", velocity_parameter);
    ros::spin();
    return 0;
}
