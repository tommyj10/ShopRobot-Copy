#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>


#include <vector>
#include <utility>		// pair
#include <numeric>		// accumulate
#include <cmath>
#include <algorithm>

#include "LaserScanner.h"


LaserStats LaserScanner::getLaserStats(int headingFrom, int headingTo){
	std::vector<float> ranges;
	
	LaserStats stats;

	if (headingTo > headingFrom){
		for (int i=headingFrom; i<headingTo; i++){
			ranges.push_back(this->getRawDistance(i));
		}
	}
	else{
		for (int i=headingFrom; i<359; i++){
			ranges.push_back(this->getRawDistance(i));
		}
		for (int i=0; i<headingTo; i++){
			ranges.push_back(this->getRawDistance(i));
		}
	}

	float median = this->getMedian(ranges);
	float mean = this->getMean(ranges);
	std::pair<float, int> minimum = this->getMin(ranges);
	std::pair<float, int> maximum = this->getMax(ranges);

	float min_distance = minimum.first;
	int min_heading = minimum.second + headingFrom;

	float max_distance = maximum.first;
	int max_heading = maximum.second + headingFrom;

	if (max_heading >= 360){max_heading -= 360;}
	if (min_heading >= 360){min_heading -= 360;} 

	stats.ranges = ranges;
	stats.median = median;
	stats.mean = mean;
	
	stats.min = min_distance;
	stats.angle_of_min = min_heading;

	stats.max = max_distance;
	stats.angle_of_max = max_heading;

	return stats;
}




LaserScanner::LaserScanner(){
	this->dataAvailable = false;	
	this->scanningRange = 90;	
}
		
void LaserScanner::laserCallback(const sensor_msgs::LaserScan &msg){
	this->ranges = msg.ranges;
	this->dataAvailable = true;
}


/**
*	Measure the turtlebots orientation relative to the wall on the left. 0 degrees is parallel. positive angle is facing towards
* 	the wall, negative angle is facing away from the wall.
*/
float LaserScanner::measureHeadingFromLeftWall(){
	const int west = 90;
	const int northwest_max_angle = 60;
	int northwest = 30;
	
	
	for (int i=northwest; i<northwest_max_angle; i++){
		float dist_i = this->getRawDistance(i);
		float dist_w = this->getRawDistance(west);
	
		if ((dist_i < dist_w*2)){
			northwest = i;
			break;
		}
	}

	float distanceWest = this->getRawDistance(west);
	float distanceNorthWest = this->getRawDistance(northwest);

	float anglediff = M_PI*(west - northwest)/180;
	float c = this->cosineRuleABTheta(distanceNorthWest, distanceWest, anglediff);

	float beta = asin(distanceNorthWest * sin(anglediff) / c);
	float heading = M_PI/2 - beta;

	// Fix ambiguity from cosine rule
	if (distanceWest*distanceWest + c*c < distanceNorthWest*distanceNorthWest){ 
		heading = -1*heading; 
	}

	return heading;
}

float LaserScanner::getRawDistance(int heading){
	float dist = this->ranges.at(heading);
	if (dist < 0.01){
		dist = 3;
	}
	return dist;
}

float LaserScanner::cosineRuleABTheta(float a, float b, float theta){
	float c_squared = (a*a) + (b*b) - (2*a*b*cos(theta));
	return sqrt(c_squared);
}

float LaserScanner::getMedian(std::vector<float> values){
	std::sort(values.begin(), values.end());
	int n = values.size()/2;
	return values.at(n);
}

float LaserScanner::getMean(const std::vector<float> &values){
	return accumulate(values.begin(), values.end(), 0.0)/values.size();
}

std::pair<float, int> LaserScanner::getMin(const std::vector<float> &values){
	float min = values.at(0);
	int min_heading = 0;

	for(int i=0; i<values.size(); i++){

		if (values.at(i) < min){
			min_heading = i;
			min = values.at(i);
		}

	}
	std::pair<float, int> result;
	result.first = min;
	result.second = min_heading;
	return result;
}

std::pair<float, int> LaserScanner::getMax(const std::vector<float> &values){
	float max = values.at(0);
	int max_heading = 0;
	
	for(int i=0; i<values.size(); i++){

		if (values.at(i) > max){
			max_heading = i;
			max = values.at(i);
		}

	}
	std::pair<float, int> result;
	result.first = max;
	result.second = max_heading;
	return result;
}

bool LaserScanner::dataIsAvailable(){
	return this->dataAvailable;		
}