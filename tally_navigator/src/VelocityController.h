#ifndef VELOCITYCONTROLLER_H
#define VELOCITYCONTROLLER_H

#include "ros/ros.h"

#include <vector>
#include <geometry_msgs/Twist.h>

class VelocityController{
	public:
		VelocityController(ros::Publisher * v_pub);
		void move(std::vector<float> linear, std::vector<float> angular);
	
	private:
		ros::Publisher * velocity_publisher;
};

#endif
