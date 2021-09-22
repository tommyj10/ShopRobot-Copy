#include <cmath>				// trigonometric functions
#include <iostream>
#include <sensor_msgs/Imu.h>

#include "IMU.h"

/**
* IMU class reads IMU sensor data. 
*/
IMU::IMU(){
	this->dataAvailable = false;
}


/**
* The imu topic contains orientation data in quaternions. Needs to be converted to euler angles.
*/
float IMU::getCurrentHeading(){
	Euler eulerAngles = quaternionToEuler(this->orientation);
 	return eulerAngles.z;
}



/**
* Function that gets called every time there is new data available in the /imu topic. 
* Read the data and store it in the class variables. 
*/
void IMU::imuCallback(const sensor_msgs::Imu &msg){
	this->dataAvailable = true;
	this->orientation.x = msg.orientation.x;
	this->orientation.y = msg.orientation.y;
	this->orientation.z = msg.orientation.z;
	this->orientation.w = msg.orientation.w;
	return;
}


/**
* Return true if there is data available.
*/
bool IMU::dataIsAvailable(){
	return this->dataAvailable;
}



/**
* Convert a quaternion to euler angles. Algorithm from 
* https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
*/
Euler IMU::quaternionToEuler(const Quaternion &quat){
	Euler orientation;
	
	// Roll
	double sinr_cosp = +2.0 * (quat.w * quat.x + quat.y * quat.z);
	double cosr_cosp = +1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y);
	orientation.x = atan2(sinr_cosp, cosr_cosp);

	// Pitch
	double sinp = +2.0 * (quat.w * quat.y - quat.z * quat.x);
	if (fabs(sinp) >= 1){
		orientation.y = copysign(M_PI / 2, sinp);	
	}
	else{
		orientation.y = asin(sinp);
	}

	// Yaw
	double siny_cosp = +2.0 * (quat.w * quat.z + quat.x * quat.y);
	double cosy_cosp = +1.0 * - 2.0 * (quat.y * quat.y + quat.z * quat.z);
	orientation.z = atan2(siny_cosp, cosy_cosp);

	// Positives get scaled to [0, pi]
	if(orientation.z >= M_PI / 2){
		orientation.z = 2*(orientation.z - M_PI/2);
	}

	// Negatives get scaled to [pi, 2pi]

	// [-pi/2, -pi] +pi/2 -> [0 -> -pi/2] *2 -> [0, -pi] 2pi+ -> [2pi, pi]
	if(orientation.z <= M_PI / -2){
		orientation.z = 2 * M_PI + 2 * (orientation.z + M_PI/2);
	}

	return orientation;
}









