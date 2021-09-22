#include "ros/ros.h"

#include <vector>
#include <cassert>
#include <geometry_msgs/Twist.h>

#include "VelocityController.h"


VelocityController::VelocityController(ros::Publisher * v_pub){
	this->velocity_publisher = v_pub;
}
		
void assertNotNan(std::vector<float> &values){
	for (int i=0; i<values.size(); i++){
		assert(!std::isnan(values.at(i)));
	}
}


void VelocityController::move(std::vector<float> linear, std::vector<float> angular){
	geometry_msgs::Twist msg;
			
	// nan velocities are undefined behaviour		
	assertNotNan(linear);
	assertNotNan(angular);

	msg.linear.x = linear[0];
	msg.linear.y = linear[1];
	msg.linear.z = linear[2];

	msg.angular.x = angular[0];
	msg.angular.y = angular[1];
	msg.angular.z = angular[2];
			
	this->velocity_publisher->publish(msg);
}


