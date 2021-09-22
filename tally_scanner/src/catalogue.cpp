
#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <zbar.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <turtlebot3_msgs/Sound.h>
#include <cstdlib>

using namespace cv;
using namespace std;
using namespace zbar;

const int HISTORY_SIZE = 15;

struct QRCode{
    string type;
    string data;
    vector <Point> location;
};


// Catalogue class that abstracts writing and reading a catalogue txt file.
// The catalogue contains the possible items that can be scanned by the tallybot. 
class Catalogue{
    std::vector<std::string> catalogue_items;
    ifstream f_in;
    std::string catalogue_file;

    public:
        Catalogue(std::string catalogue_file){
            this->catalogue_file = catalogue_file;
            ifstream f_in(catalogue_file.c_str());
            if (f_in.good()){
                std::string str;
                while (std::getline(f_in, str)){
                    this->catalogue_items.push_back(str);
                }
            }
            f_in.close();    
        }

        bool item_is_stocked(std::string item){
            for (int i=0; i<this->catalogue_items.size(); i++){
                if (!catalogue_items[i].compare(item)){
                    return true;
                }
            }
            return false;
        }

        void add_to_catalogue(std::string item){
            for (int i=0; i<this->catalogue_items.size(); i++){
                if (!catalogue_items[i].compare(item)){
                    return;
                }
            }
            this->catalogue_items.push_back(item);
            ROS_INFO("Added %s to list of catalogue items", item.c_str());
        }
        
        void save_catalogue(){
            ofstream f_out(this->catalogue_file.c_str());
            for (unsigned int i=0; i<this->catalogue_items.size(); i++){
                f_out << catalogue_items[i] << std::endl; 
            }
            f_out.close();
            ROS_INFO("Catalogue file saved to %s", this->catalogue_file.c_str());
        }

};

class ItemScanner{
    public:
        ros::NodeHandle nh_;
  	    ros::Publisher beep_publisher_;
        std::string home = getenv("HOME");

        ItemScanner():
            catalogue(this->home + "/catalogue.txt")
        {
            beep_publisher_ = nh_.advertise<turtlebot3_msgs::Sound>("/sound", 1);
        }
        
        void scanImage(Mat &im, vector<QRCode> &items){
            this->scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
            Mat imGray;
            cvtColor(im, imGray,COLOR_BGR2GRAY);
            Image image(im.cols, im.rows, "Y800", (uchar *)imGray.data, im.cols * im.rows);
            int n = scanner.scan(image);
            
            if (n==0){
                addToRecentHistory("Nothing");
            }
            
            for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol){       
                string data = symbol->get_data();
                QRCode obj;
                obj.data = data;
                for(int i = 0; i< symbol->get_location_size(); i++){
                    obj.location.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));
                }
                items.push_back(obj);
                
                if (!this->isInRecentHistory(data)){
                    if (!this->catalogue.item_is_stocked(data)){
                        this->catalogue.add_to_catalogue(data);
                        ROS_INFO("Added to catalogue: %s", data.c_str());
                        turtlebot3_msgs::Sound beep;
                        beep.value=1;
                        this->beep_publisher_.publish(beep);
                        this->catalogue.save_catalogue();
                    }
                }
                addToRecentHistory(data);
            }
        }

        // Display barcode and QR code location  
        void visualise(Mat &im, vector<QRCode> &qrcodes){
            for(int i = 0; i < qrcodes.size(); i++)
            {
                vector<Point> points = qrcodes[i].location;
                vector<Point> hull;
                
                if(points.size() > 4){ convexHull(points, hull); }
                else{ hull = points; }
                
                int n = hull.size();
                for(int j = 0; j < n; j++){
                    line(im, hull[j], hull[ (j+1) % n], Scalar(255,0,0), 3);
                }   
            }
            imshow("QR-Scanner", im);
            waitKey(3);
    }

    private:
        zbar::ImageScanner scanner;
        Catalogue catalogue;

        void addToRecentHistory(string data){
            history.push_back(data);
            while (history.size() > HISTORY_SIZE){
                history.erase(history.begin());
            }
            return;
        }

        bool isInRecentHistory(string data){
            for (int i=0; i<history.size(); i++){
                if (data.compare(history[i]) == 0){
                    return true;
                }
            }
            return false;
        }
        std::vector<std::string> history;
};


class ImageConverter{
  	ros::NodeHandle nh_;
  	image_transport::ImageTransport it_;
  	image_transport::Subscriber image_sub_;
    ItemScanner itemscanner;

	public:
        ImageConverter() : it_(nh_){
            image_sub_ = it_.subscribe("/raspicam_node/image", 1, &ImageConverter::imageCb, this);
        }

        ~ImageConverter(){
            cv::destroyAllWindows();
        }

        void imageCb(const sensor_msgs::ImageConstPtr& msg){
            cv_bridge::CvImagePtr cv_ptr;
            
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            cv::Mat im = cv_ptr->image;
            std::vector<QRCode> qrcodes;
            itemscanner.scanImage(im, qrcodes);
            itemscanner.visualise(im, qrcodes);    
        }    
};


int main(int argc, char** argv){
    ros::init(argc, argv, "catalogue_entry_node");
    ImageConverter ic;
    ros::spin();
    return 0;
}