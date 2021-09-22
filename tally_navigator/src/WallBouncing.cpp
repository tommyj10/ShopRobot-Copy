#include "ros/ros.h"
#include "ros/console.h"
#include "WallBouncing.h"
#include "LaserScanner.h"
#include "VelocityController.h"
#include "ColourSense.h"

#include <cmath>
#include <iostream>
#include <ctime>

const float WALL_STOP_DISTANCE = 	0.35;
const float FORWARD_VELOCITY = 		0.2;
const float K_PROPORTIONAL_SPIN = 	1.0;
const float K_INTEGRAL_SPIN = 		0.005;
const float MAX_HEADING_DELTA = 	0.05;

const int FOUND_SHELF_CODE =        19;


const int CODE_PINK = 1;
const int CODE_YELLOW = 2;
const int CODE_GREEN = 3;


WallBouncer::WallBouncer(LaserScanner* ls, VelocityController* vc, IMU* imu, ColourSensor* colour){
	this->laserscan = ls;
	this->velocitycontroller = vc;
	this->inertial = imu;
    this->coloursensor = colour;
	this->robotstate = GOING_FORWARD;
    this->foundShelf = false;
}



int WallBouncer::run(){
	if (this->robotstate == GOING_FORWARD){
		this->goForward();
	}

    if (this->robotstate == SCANNING){
        this->scanWallColour();
    }

	if (this->robotstate == ROTATING){
		this->rotateToTargetHeading();
	}

    if (this->foundShelf){
        this->foundShelf = false;

		if (!(this->shelfColour.compare("Pink"))){
			return CODE_PINK;
		}
		if (!(this->shelfColour.compare("Green"))){
			return CODE_GREEN;
		}
		if (!(this->shelfColour.compare("Yellow"))){
			return CODE_YELLOW;
		}
    }
    return 0;
}


void WallBouncer::rotateToTargetHeading(){
	float currentHeading = this->inertial->getCurrentHeading();
	float error = calculateAngleError(this->targetHeading, currentHeading);

	if (fabs(error) < MAX_HEADING_DELTA){
        ROS_INFO("Finished rotation, going forward");
		this->velocitycontroller->move({0,0,0},{0,0,0});
		this->robotstate = GOING_FORWARD;
	}

	float u_angular = error * K_PROPORTIONAL_SPIN;
	this->velocitycontroller->move({0,0,0},{0,0,u_angular});
	return;
}


void WallBouncer::scanWallColour(){
    float currentHeading = this->inertial->getCurrentHeading();
	float error = calculateAngleError(this->wallScanHeading, currentHeading);

	if (fabs(error) < MAX_HEADING_DELTA){
        std::time_t last_recording = this->coloursensor->getTimeRecorded();
        std::string colour = this->coloursensor->getColour();
        std::time_t time_now = std::time(nullptr);

		// If the colour sensor emitted a reading in the last two seconds, exit and start wall following
        if ((time_now - last_recording < 2) && colour.compare("None"))  {
            ROS_INFO("Found a %s shelf to follow", colour.c_str());
			this->bounceRotation = (270/180)*M_PI;
			this->targetHeading = this->calculateTargetHeading(currentHeading, this->bounceRotation);
            this->foundShelf = true;
			this->shelfColour = colour;
        }
        
        ROS_INFO("Finished scanning wall colour");
		this->velocitycontroller->move({0,0,0},{0,0,0});
		this->targetHeading = this->calculateTargetHeading(currentHeading, this->bounceRotation);
        this->robotstate = ROTATING;
	}

	float u_angular = error * K_PROPORTIONAL_SPIN;
	this->velocitycontroller->move({0,0,0},{0,0,u_angular});
	return;
}


float WallBouncer::calculateAngleError(float theta1, float theta2){
	double diff = theta1-theta2;
	if (diff > M_PI){
		diff = diff - 2*M_PI;
	}

	if (diff < -1*M_PI){
		diff = diff + 2*M_PI;
	}

	return diff;

}


float WallBouncer::calculateTargetHeading(float theta1, float theta2){
	float target = theta1 + theta2;

	if (target > M_PI * 2){
		target = target - (M_PI * 2);
	}
	else if (target < 0){
		target = target + (M_PI * 2);
	}
	return target;

}



void WallBouncer::goForward(){
	this->velocitycontroller->move({FORWARD_VELOCITY,0,0},{0, 0, 0});
	LaserStats laserNorth = this->laserscan->getLaserStats(340, 20);
    LaserStats widerNorth = this->laserscan->getLaserStats(315, 45);

	if (laserNorth.min < WALL_STOP_DISTANCE){
		ROS_INFO("Wall ahead, rotating to scan colour");
		this->velocitycontroller->move({0, 0, 0}, {0, 0, 0});
		float currentHeading = this->inertial->getCurrentHeading();

        int heading_of_wall = widerNorth.angle_of_min;

        if (heading_of_wall >= 180){
            float rotation = -1*(360 - heading_of_wall + 90)*M_PI/180;
            this->wallScanHeading = this->calculateTargetHeading(currentHeading, rotation);
            this->bounceRotation = (rand() % 50 + 190)*M_PI/180;
            
            //this->targetHeading = this->calculateTargetHeading(currentHeading, bounce_rotation);
        }
        else {
            float rotation = -1*(90 - heading_of_wall) * M_PI/180;
            this->wallScanHeading = this->calculateTargetHeading(currentHeading, rotation);
            this->bounceRotation = -1*(rand() % 50 + 10)*M_PI/180;
            
            //this->targetHeading = this->calculateTargetHeading(currentHeading, bounce_rotation);
        }
		this->robotstate = SCANNING;
	}
}
