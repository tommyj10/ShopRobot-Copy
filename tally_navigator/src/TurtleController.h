#ifndef TURTLECONTROLLER_H
#define TURTLECONTROLLER_H

#include "LaserScanner.h"
#include "VelocityController.h"
#include "IMU.h"
#include <string>

enum TurtleControllerState{
	NO_CHANGE,
	FOLLOWING_WALL,
	OUTSIDE_WALL,
	INSIDE_WALL,
	ACTION_COMPLETE,
	DO_NOTHING		
};

float saturateSignal(float & signal, float saturationLevel);
void nanToZero(float &value);

class TurtleController{
	public:
		TurtleController(LaserScanner * laserscan, VelocityController * velocitycontroller);
		void run();
		void followWall();
		void followOutsideWall();
		void followInsideWall();

	private:
		LaserScanner * laserscan;
		VelocityController * velocitycontroller;
		TurtleControllerState robotState;
		float sweepRadius;

		bool checkWallFollowExitCondition();
		bool checkWallFollowEntryCondition();

		bool checkOutsideWallEntryCondition();
		bool checkOutsideWallExitCondition();

		bool checkInsideWallEntryCondition();
		bool checkInsideWallExitCondition();

		TurtleControllerState decideNextAction();

};


#endif
