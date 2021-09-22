#include "ros/ros.h"
#include "ros/console.h"
#include <iostream>
#include <string>
#include <cstring>
#include <vector>
#include <fstream>
#include <cstdlib>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include "TurtleController.h"
#include "VelocityController.h"
#include "LaserScanner.h"
#include "IMU.h"
#include "WallBouncing.h"
#include "ColourSense.h"

/**
* This program implements a wall following turtlebot maze solver. Four high level classes are used to control the robot
* and receive feedback-signals: 
*
* TurtleController:			Navigation and movement logic. Performs the heavy lifting.
* VelocityController:		Publishes messages to the cmd_vel topic 
* LaserScanner:				Keeps a record of laser scan ranges 
* IMU:						Keeps a record of IMU sensor readings
*
*/

const int CODE_PINK = 1;
const int CODE_YELLOW = 2;
const int CODE_GREEN = 3;
const int CODE_NOTHING = 99;

enum TallyState{
	EXPLORING,
	WALLFOLLOWING
};


// Values to be modified by callbacks only
bool buttonPushed = false;
bool lowBattery = false;
int shelf_name_count = 0;

void buttonPushedCallback(const diagnostic_msgs::DiagnosticArray &msg){
	if (std::strcmp(msg.status.at(4).message.c_str(), "Pushed Nothing")){
		buttonPushed = true;
		ROS_INFO("BUTTON PUSHED");
	}
}

// Called when the /lowbattery topic emits a positive
void lowBatteryCallback(const std_msgs::Bool &msg){
	lowBattery = msg.data;
	ROS_INFO("GOT LOW BATTERY WARNING");
}

// This callback gets called when a QR code with the name of a shelf is detected  by the scanning node
void shelfNameCallback(const std_msgs::String msg){
	shelf_name_count++;
}


// Load the required shelves from disk
std::vector<int> load_shelves(void){
	std::vector<int> loaded_shelves;
	std::string shelf_file = getenv("HOME");
	shelf_file = shelf_file + "/colourfile.txt";
	std::ifstream shelves(shelf_file.c_str());
	if (shelves.good()){
		std::string line;
		while (std::getline(shelves, line)){
			if(!line.compare("Pink")){
				loaded_shelves.push_back(0);
			}
			if(!line.compare("Green")){
				loaded_shelves.push_back(1);
			}
			if(!line.compare("Yellow")){
				loaded_shelves.push_back(2);
			}
		}
	}
	return loaded_shelves;
}

bool check_completion(std::vector<int> shelf_statuses, std::vector<int> required_shelves){
	int completedness = 0;
	for(int i=0; i<required_shelves.size(); i++){
		if (shelf_statuses[required_shelves[i]] == 2){
			completedness += 1;
		}
	}
	if (completedness == required_shelves.size()){
		return 1;
	}
	return 0;
}


int main(int argc, char **argv){

    // create a ROS node
    ros::init(argc, argv, "WallFollower");
    ros::NodeHandle nh;

	// Initialise a publisher that talks to the turtlebots movement controller
	ros::Publisher velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	ros::Publisher tally_state_publisher = nh.advertise<std_msgs::UInt8>("/tally_state", 10);
	ros::Publisher autonomous_navigation_publisher = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

	VelocityController velcmd(&velocity_publisher);
	LaserScanner laserscan;
	IMU imu;
    ColourSensor coloursensor;
	TurtleController turtlecontroller(&laserscan, &velcmd);
	WallBouncer wallbouncer(&laserscan, &velcmd, &imu, &coloursensor);

	ros::Subscriber battery_subscriber = nh.subscribe("/lowbattery", 1, lowBatteryCallback);
	ros::Subscriber laser_subscriber = nh.subscribe("/scan", 1000, &LaserScanner::laserCallback, &laserscan);
	ros::Subscriber imu_subscriber = nh.subscribe("/imu", 100, &IMU::imuCallback, &imu);
	ros::Subscriber colour_subscriber = nh.subscribe("/coloursensor", 3, &ColourSensor::ColourSensorCallback, &coloursensor);
    ros::Subscriber shelf_subscriber = nh.subscribe("/shelfname", 10, shelfNameCallback);
	ros::Rate loop_rate(10);

	ROS_INFO("Waiting for laser... ");
	while (ros::ok()){
		ros::spinOnce();
		if (laserscan.dataIsAvailable()){ break; }
		loop_rate.sleep();
	}
    ROS_INFO("Done.");
	ROS_INFO("Starting navigation node.");

    int res = 0;
	int green_done = 0;
	int yellow_done = 0;
	int pink_done = 0;
	int current_shelf = CODE_NOTHING;
	std::vector<int> shelves_to_do = load_shelves();


    while (ros::ok()){
        ros::spinOnce();

		// If each shelf has been collected to completion, signal the going-home flag
		std::vector<int> shelf_statuses = {pink_done, green_done, yellow_done};
		bool all_shelves_completed = check_completion(shelf_statuses, shelves_to_do);
		
		// The low battery signal automatically sends the turtle home, so signal it
		if (all_shelves_completed){
			lowBattery = 1;
			ROS_INFO("ALL SHELVES ARE FINISHED");
		}

		if (lowBattery){
			ROS_INFO("Received low battery warning, going home");
			velcmd.move({0,0,0},{0,0,0});
			geometry_msgs::PoseStamped goal;
			goal.header.frame_id = "map";
			goal.pose.position.x = 0;
			goal.pose.position.y = 0;
			goal.pose.position.z = 0;
			goal.pose.orientation.w = 1;
			goal.pose.orientation.x = 0;
			goal.pose.orientation.y = 0;
			goal.pose.orientation.z = 0;
			autonomous_navigation_publisher.publish(goal);
			break;
		}
		
		// If we've seen the barcode indicating the shelf name twice, then we've orbited the whole shelf once
		if (shelf_name_count >= 2){
			ROS_INFO("Shelf Orbiting finished, breaking away");
			if (current_shelf == CODE_YELLOW){
				yellow_done = 2;
			}
			if (current_shelf == CODE_PINK){
				pink_done = 2;
			}
			if (current_shelf == CODE_GREEN){
				green_done = 2;
			}
			shelf_name_count = 0;
			res = 0;
		}

		if (res == CODE_PINK){
			if (pink_done){
				ROS_INFO("Pink shelf found but already completed. Going back to explore...");
				res = 0;
			}
			else{
				ROS_INFO("Found a pink shelf, taking inventory on it.");
				current_shelf = CODE_PINK;
				pink_done = 1;
			}
		}
		if (res == CODE_GREEN){
			if (green_done){
				ROS_INFO("Green shelf found but already completed. Going back to explore...");
				res = 0;
			}
			else{
				ROS_INFO("Found a green shelf, taking inventory on it.");
				current_shelf = CODE_GREEN;
				green_done = 1;
			}
			
		}
		if (res == CODE_YELLOW){
			if (yellow_done){
				ROS_INFO("Yellow shelf found but already completed. Going back to explore...");
				res = 0;
			}
			else{
				ROS_INFO("Found a yellow shelf, taking inventory on it.");
				current_shelf = CODE_YELLOW;
				yellow_done = 1;
			}
		}

        if (res > 0){
			res = CODE_NOTHING;
            turtlecontroller.run();
			std_msgs::UInt8 msg;
			msg.data = WALLFOLLOWING;
			tally_state_publisher.publish(msg);
        }

        else{
        	res = wallbouncer.run();
			std_msgs::UInt8 msg;
			msg.data = EXPLORING;
			tally_state_publisher.publish(msg);
		}
	    loop_rate.sleep();
    }

    return 0;
}













