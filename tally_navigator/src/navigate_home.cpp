#include "ros/ros.h"
#include "ros/console.h"
#include <std_msgs/Bool.h>


int main(int argc, char **argv){
    ros::init(argc, argv, "navigate_home");
    ros::NodeHandle nh;
    ros::Publisher low_battery_publisher = nh.advertise<std_msgs::Bool>("/lowbattery", 10);
    std_msgs::Bool msg;
    
    while(ros::ok()){
        msg.data = 1;
        low_battery_publisher.publish(msg);
        ros::spinOnce();
    }
}













