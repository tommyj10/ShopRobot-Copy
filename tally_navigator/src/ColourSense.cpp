#include <iostream>
#include <ctime>

#include <std_msgs/String.h>

#include "ColourSense.h"
#include "ros/ros.h"
#include "ros/console.h"


ColourSensor::ColourSensor(){
	this->dataAvailable = false;
    this->time_recorded = std::time(nullptr);
    this->colour = "None";
}

void ColourSensor::ColourSensorCallback(const std_msgs::String &msg){
    this->dataAvailable = true;
    this->colour = msg.data;
    this->time_recorded = std::time(nullptr);
}


std::time_t ColourSensor::getTimeRecorded(){
    return this->time_recorded;
}

std::string ColourSensor::getColour(){
    return this->colour;
}