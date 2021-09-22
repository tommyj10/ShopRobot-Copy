#ifndef COLOURSENSE_H
#define COLOURSENSE_H


#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ctime>
#include <string>


class ColourSensor{
    public:
        ColourSensor();
        void ColourSensorCallback(const std_msgs::String &msg);
        bool dataIsAvailable();
        std::time_t getTimeRecorded();
        std::string getColour();

    private:
        bool dataAvailable;
        std::string colour;
        std::time_t time_recorded;

};



#endif