#ifndef LASERSCANNER_H
#define LASERSCANNER_H


#include "ros/ros.h"

#include <vector>
#include <sensor_msgs/LaserScan.h>

struct LaserStats{
	std::vector<float> ranges;
	float mean;
	float median;
	float min;
	float max;
	float angle_of_min;
	float angle_of_max;
};

class LaserScanner{
	public:
		LaserScanner();
		void laserCallback(const sensor_msgs::LaserScan &msg);
		LaserStats getLaserStats(int headingFrom, int headingTo);
		float getRawDistance(int heading);
		float measureHeadingFromLeftWall();
		bool dataIsAvailable();

	private: 
		float cosineRuleABTheta(float a, float b, float theta);
		float getMedian(std::vector<float> values);
		float getMean(const std::vector<float> &values);
		std::pair<float, int> getMin(const std::vector<float> &values);
		std::pair<float, int> getMax(const std::vector<float> &values);


		bool dataAvailable;
		int scanningRange;
		std::vector<float> ranges;
	
};

#endif
