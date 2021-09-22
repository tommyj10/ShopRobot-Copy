#include <iostream>
#include <fstream>
#include <cstdlib>

#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>



std::vector<std::string> colours;


void callback(const std_msgs::String msg){
    bool found=0;
    
    for (int i=0; i<colours.size(); i++){
        if(!colours[i].compare(msg.data.c_str())){
            found = 1;
        }
    }
    if (!found){
        std::string home = getenv("HOME");
        std::string colourfile_name = home + "/colourfile.txt";
        std::ofstream colourfile(colourfile_name);
        colours.push_back(msg.data);
        for(int i=0; i<colours.size(); i++){
            colourfile << colours[i] << std::endl;
        }
        colourfile.close();
        ROS_INFO("%s", msg.data.c_str());
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "coloursave");
    ros::NodeHandle nh;
    ros::Subscriber coloursubscriber = nh.subscribe<std_msgs::String>("/coloursensor", 10, callback);
    ros::spin();
    return 0;
}