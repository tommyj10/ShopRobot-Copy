
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include "ros/ros.h"
#include "ros/console.h"

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

const double P_MIN = 0.5;
const int S_MIN = 91;
const int S_MAX = 255;
const int V_MIN = 0;
const int V_MAX = 255;

const int PINK_MIN = 155;
const int PINK_MAX = 175;
const int YELLOW_MIN = 19;
const int YELLOW_MAX = 27;
const int GREEN_MIN = 38;
const int GREEN_MAX = 58;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher coloursense_pub;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/raspicam_node/image", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    coloursense_pub = nh_.advertise<std_msgs::String>("/coloursensor", 10);

    ROS_INFO("Colour Sensor Initialised");
    std_msgs::String startup;
    startup.data = "Colour Sensor Initialised";
    coloursense_pub.publish(startup);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
   
    Mat frameHSV, framePink, frameYellow, frameGreen;
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
          
    cvtColor(cv_ptr->image, frameHSV, COLOR_BGR2HSV);
    // Detect the object based on HSV Range Values

    int nPixels = frameHSV.cols*frameHSV.rows;

    inRange(frameHSV, Scalar(PINK_MIN, S_MIN, V_MIN), Scalar(PINK_MAX, S_MAX, V_MAX), framePink);
    inRange(frameHSV, Scalar(YELLOW_MIN, S_MIN, V_MIN), Scalar(YELLOW_MAX, S_MAX, V_MAX), frameYellow);
    inRange(frameHSV, Scalar(GREEN_MIN, S_MIN, V_MIN), Scalar(GREEN_MAX, S_MAX, V_MAX), frameGreen);

    sPink = cv::sum(framePink)[0] / 255 / nPixels;
    sYellow = cv::sum(frameYellow)[0] / 255 / nPixels;
    sGreen = cv::sum(frameGreen)[0] / 255 / nPixels;;

    if (sPink > P_MIN) 
    {
      // ROS_INFO("Pink");
      std_msgs::String colour;
      colour.data = "Pink";
      coloursense_pub.publish(colour);
    }
    else if (sYellow > P_MIN) 
    {
      // ROS_INFO("Yellow");
      std_msgs::String colour;
      colour.data = "Yellow";
      coloursense_pub.publish(colour);
    }
    else if (sGreen > P_MIN)  
    {
      // ROS_INFO("Green");
      std_msgs::String colour;
      colour.data = "Green";
      coloursense_pub.publish(colour);
    }

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }

private:

  double sPink;
  double sYellow;
  double sGreen;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "coloursensing_node");


  ImageConverter ic;
  ros::spin();
  return 0;
}