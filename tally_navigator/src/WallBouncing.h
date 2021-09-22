#ifndef WALLBOUNCER_H
#define WALLBOUNCER_H
#include <string>
#include "LaserScanner.h"
#include "VelocityController.h"
#include "ColourSense.h"
#include "IMU.h"
#include "ros/ros.h"

enum WallBouncerState{
        GOING_FORWARD,
		ROTATING,
        SCANNING
};

class WallBouncer{
	public:
		WallBouncer(LaserScanner* ls, VelocityController* vc, IMU* imu, ColourSensor* colour);
		int run();
		void goForward();
		void rotateToTargetHeading();
        void scanWallColour();
		float calculateAngleError(float theta1, float theta2);
		float calculateTargetHeading(float theta1, float theta2);
	private:
		LaserScanner* laserscan;
		VelocityController* velocitycontroller;
		IMU* inertial;
		WallBouncerState robotstate;
        ColourSensor* coloursensor;
		float targetHeading;
        float wallScanHeading;
        float bounceRotation;
        bool foundShelf;
		std::string shelfColour;
};



#endif
