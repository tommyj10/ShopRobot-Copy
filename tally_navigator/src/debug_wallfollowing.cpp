#include "ros/ros.h"
#include "ros/console.h"
#include <iostream>
#include <string>
#include <cstring>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include "TurtleController.h"
#include "VelocityController.h"
#include "LaserScanner.h"
#include "IMU.h"
#include "WallBouncing.h"
#include "ColourSense.h"

int main(int argc, char **argv){

    // create a ROS node
    ros::init(argc, argv, "WallFollower");
    ros::NodeHandle nh;

	// Initialise a publisher that talks to the turtlebots movement controller
	ros::Publisher velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	VelocityController velcmd(&velocity_publisher);
	LaserScanner laserscan;
	IMU imu;
    ColourSensor coloursensor;
	TurtleController turtlecontroller(&laserscan, &velcmd);
	WallBouncer wallbouncer(&laserscan, &velcmd, &imu, &coloursensor);


	ros::Subscriber laser_subscriber = nh.subscribe("/scan", 1000, &LaserScanner::laserCallback, &laserscan);
	ros::Subscriber imu_subscriber = nh.subscribe("/imu", 100, &IMU::imuCallback, &imu);
	ros::Subscriber colour_subscriber = nh.subscribe("/coloursensor", 3, &ColourSensor::ColourSensorCallback, &coloursensor);
	ros::Rate loop_rate(10);

	ROS_INFO("Waiting for laser... ");
	while (ros::ok()){
		ros::spinOnce();
		if (laserscan.dataIsAvailable()){ break; }
		loop_rate.sleep();
	}
    ROS_INFO("Done.");
	ROS_INFO("Starting navigation node.");

   
    while (ros::ok()){
        ros::spinOnce();
		
        turtlecontroller.run();
			
        

       
	    loop_rate.sleep();
    

    }
}













