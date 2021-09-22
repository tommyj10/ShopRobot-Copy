#ifndef IMU_H
#define IMU_H

#include <sensor_msgs/Imu.h>
#include <vector>

struct Quaternion{
	float x, y, z, w;
};

struct Euler{
	float x, y, z;
};

class IMU{
	public:
		IMU();
		float getCurrentHeading();
		bool dataIsAvailable();
		void imuCallback(const sensor_msgs::Imu &msg);

	private:
		bool dataAvailable;
		Euler quaternionToEuler(const Quaternion &quat);
		Quaternion orientation;


};

#endif
