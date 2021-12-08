#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "iostream"
#include "a2_code/Userparameter.h"
#include "std_srvs/Empty.h"

using namespace std;


/* control the interraction with the user
 * gives the user the 3 basis possible command and the key to enter it
 * return a integer corresponding to the selected key
 * -1 -> decrease the speed of the robot
 *  1 -> increase the speed of the robot
 *  2 -> reset the position of the robot to the initial one
 */
int input(){
    
    int user_return = 0;
    char input_letter;
            
    cout<<"speed user controller "<<endl;
    cout<<"'w' for ingrese the speed"<<endl;
    cout<<"'s' for decrease the speed"<<endl;
    cout<<"'r' for reset the robot position"<<endl;
    cin>>input_letter;
            
    if(input_letter == 'w'){
        //increase the multiplier
        user_return = 1;
    }else if(input_letter == 's'){
        //decrease the multiplier
        user_return = -1;
    }else if(input_letter == 'r'){
        //reset
        user_return = 2;
    }
    return user_return;
}    


/* initialize and call the service related to the user interaction with the control node
 * and the world
 * using terminal interface comunicate with the user and continuosly ask for control parameter
 * modification 
 * 
 */
int main (int args, char **argv){
    //init the node
    ros::init(args, argv, "user_node");
    
    //crate the NodeHandler
    ros::NodeHandle nh;
    
    //create a service to interract with the control node
    ros::ServiceClient client = nh.serviceClient<a2_code::Userparameter>("/userparameter");
    a2_code::Userparameter srv1;
    
    //create a service to call the reset position service
    ros::ServiceClient client2 = nh.serviceClient<std_srvs::Empty>("/reset_positions");
    
    
    int message = 0;
    //infinite loop
    while(1){
        
        //call input function and save return value
        message = input();
        if(message == 2){
           //reset robot position to initial position
           std_srvs::Empty srv;
           client2.waitForExistence();
           //call the service
           client2.call(srv);
        }else{
            //encrease or decrease the speed of the robot
            srv1.request.user_value = message;
            //call the service
            client.call(srv1);
        }
    }
    return 0;
}
    
    
    
    


