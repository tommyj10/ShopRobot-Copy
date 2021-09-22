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
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <csignal>
#include <cstdlib>


using namespace cv;
using namespace std;
using namespace zbar;


const int HISTORY_SIZE = 30;
const int numberOfShelves = 2;
string currentShelf;
std::string sorted_inventory;

struct QRCode{
    string type;
    string data;
    vector <Point> location;
};

struct InventoryItem{
    int count;
    std::string item;
};


void copy_file(std::string src, std::string dest){
    ifstream source_file(src);
    ofstream destination_file(dest);

    if (source_file.good()){
        std::string str;
        while (std::getline(source_file, str)){
            destination_file << str << std::endl;
        }
    }

    destination_file.close();
    source_file.close();
}


class Inventory{
    std::vector<InventoryItem> inventory_items;
    std::vector<InventoryItem> green_shelf;
    std::vector<InventoryItem> yellow_shelf;
    std::vector<InventoryItem> pink_shelf;
    std::vector<InventoryItem> all_shelves;

    std::string inventory_file;

    public:

        // Initialise an inventory file with the name of file specified in constructor arg
        Inventory(std::string inventory_file){
            this->inventory_file = inventory_file;
            string home = getenv("HOME");
            copy_file(this->inventory_file, home+"/old_inventory.txt");
        }

        /**
                          Shelf sorting function
        Sorts all of the items into different shelves.

        Input: no direct input, but must be included in Inventory class
        Output: A txt file containing re-organized shop shelvews.

        **/

        void sortShelves()
        {
            std::string home = getenv("HOME");
            std::string sorted_inventory_file = home + "/suggested.txt";
            ofstream f_out(sorted_inventory_file.c_str());

            int numberOfItems = this->all_shelves.size();
            int numberOfItemsPerShelf = (int) (numberOfItems / numberOfShelves);
            if (numberOfItemsPerShelf == 0){
                numberOfItemsPerShelf = 1;
            }
            int counter = 0;
            for(int i=0; i<numberOfItems; i++){
                if(i % numberOfItemsPerShelf == 0) {

                    if(counter == 2)
                    {
                                f_out << "Green Shelf:" << std::endl;
                    }
                    if(counter == 1)
                    {
                                f_out << "Yellow Shelf:" << std::endl;
                    }
                    if(counter == 0)
                    {
                                f_out << "Pink Shelf:" << std::endl;
                    }

                    counter++;
                }
                f_out << this->all_shelves[i].item.c_str() << " - " << this->all_shelves[i].count << std::endl;
            }

            f_out.close();
        }

        /**
                          Comparing Files function
        Reads an old inventory file from a previous scan and compares to the current inventory scanned by the store.

        Input: no direct input, but must be included in Inventory class and have access to an old inventory file
        Output: A txt file containing the changes between scans.

        **/
        void fileCompare(void)
        {


        /** This first section reads the important information from the previous text file **/
            vector<InventoryItem> oldItems;

            string line;
            string home = getenv("HOME");
            string old_inventory_file = home + "/old_inventory.txt";
            ifstream myfile (old_inventory_file);

            if (myfile.is_open())
            {
            for (int lineno = 0; getline (myfile,line) && lineno < 20; lineno++){
                char newLine [100];
                int newValue;

                if(line == "Green Shelf:")
                {
                    break;
                }

                if (lineno >= 3 ){

                    int i = 3;

                    while( line[i] != 32 || line[i+1] != 45)
                    {
                        newLine[i-3] = line[i];
                        // ROS_INFO("*************************************** %c", newLine[i-3]);
                        i++;
                    }
                    newLine[i-3] = '\0';
                    ROS_INFO("*************************************** %s", newLine);
                    // ROS_INFO("*************************************** %s", newLine);

                    newValue = line[i+3] - 48;

                    InventoryItem newItem;
                    newItem.count = newValue;
                    newItem.item = newLine;
                    oldItems.push_back(newItem);
<<<<<<< HEAD
                    }
=======
                    newLine = NULL;
>>>>>>> 6e71d10a12ef4054de7a4eee78f1cc287d5c0a5b
                }
            }


        /** This section compares the old vector of inventory items to the new inventory values **/
            vector<InventoryItem> comparisonVector;
            int placeHolder;

            int foundItem = 0;

            for(int old = 0; old < oldItems.size() ; old++)
            {

                int valueCompare = oldItems[old].count;
                string name = oldItems[old].item;

                for(int newer = 0; newer < all_shelves.size() ; newer++)
                {
                    InventoryItem newItem;

                    if(name == all_shelves[newer].item)
                    {
                    newItem.item = name;
                    newItem.count = all_shelves[newer].count - valueCompare;
                    foundItem = 1;
                    comparisonVector.push_back(newItem);
                    }



                }

                if (foundItem == 0)
                {
                    InventoryItem newItem;
                    newItem.item = name;
                    newItem.count = -1 * valueCompare;
                    comparisonVector.push_back(newItem);
                }
                foundItem = 0;
            }


            for(int newer = 0; newer < all_shelves.size() ; newer++)
            {
                int valueCompare = all_shelves[newer].count;
                string name = all_shelves[newer].item;

                for(int old = 0; old < oldItems.size() ; old++)
                {
                    if(name == oldItems[newer].item)
                    {
                    foundItem = 1;
                    }
                }

                if (foundItem == 0)
                {
                    InventoryItem newerItem;
                    newerItem.item = name;
                    newerItem.count = valueCompare;
                    comparisonVector.push_back(newerItem);
                }
                foundItem = 0;
            }

            std::string changes_inventory_file = home + "/changes.txt";
            ofstream f_out(changes_inventory_file.c_str());

            f_out << "Changes to Inventory since Last Scan:" << endl;

            for(int i = 0; i<comparisonVector.size(); i++)
            {
                f_out << comparisonVector[i].item << ": ";
                if (comparisonVector[i].count > 0){
                    f_out << "+" << comparisonVector[i].count <<endl;
                }
                else {
                    f_out << comparisonVector[i].count<<endl;
                }
            }
            f_out.close();
        }

        /**
                      Inventory Item Adder
         Adds an item to the running inventory count
         Keeps a global inventory count, but also an individual count for each shelf colour

         Input: Receives the name of the item that has just been scanned
         Output: Increases the tally on both the shelf and the total inventory file for the specific item.
         **/
        void add_to_inventory(std::string item){
            bool already_exists = false;

            for(int i=0; i < this->all_shelves.size(); i++){
                if (!item.compare(this->all_shelves[i].item)){
                    this->all_shelves[i].count++;
                    already_exists = true;
                }
            }

            if (!already_exists){
                InventoryItem new_item;
                new_item.count = 1;
                new_item.item = item;
                this->all_shelves.push_back(new_item);
            }

            if(currentShelf == "green shelf"){
                already_exists = 0;
                for(int i=0; i < this->green_shelf.size(); i++){
                    if (!item.compare(this->green_shelf[i].item)){
                        this->green_shelf[i].count++;
                        already_exists = true;
                    }
                  }
                if (!already_exists){
                    InventoryItem new_item;
                    new_item.count = 1;
                    new_item.item = item;
                    this->green_shelf.push_back(new_item);
                }
            }

            if(currentShelf == "yellow shelf"){
                already_exists = 0;
                for(int i=0; i < this->yellow_shelf.size(); i++){
                    if (!item.compare(this->yellow_shelf[i].item)){
                        this->yellow_shelf[i].count++;
                        already_exists = true;
                    }
                }
                if (!already_exists){
                    InventoryItem new_item;
                    new_item.count = 1;
                    new_item.item = item;
                    this->yellow_shelf.push_back(new_item);
                }
            }

            if(currentShelf == "pink shelf"){
                already_exists = 0;
                for(int i=0; i < this->pink_shelf.size(); i++){
                    if (!item.compare(this->pink_shelf[i].item)){
                        this->pink_shelf[i].count++;
                        already_exists = true;
                    }
                  }
                  if (!already_exists){
                      InventoryItem new_item;
                      new_item.count = 1;
                      new_item.item = item;
                      this->pink_shelf.push_back(new_item);
                  }
            }
        }

        /**
                      Saves the inventory information
         Saves the inventory information of the store in total and by shelf

         Input: No direct input, but must stay in Inventory Class
         Output: Creates an inventory txt file

         **/
        void save_inventory_to_disk(){
            ofstream f_out(this->inventory_file.c_str());
            f_out << "Store Inventory" << std::endl;
            f_out << "    " << std::endl;
            f_out << "Entire Store:" << std::endl;


            for(int i=0; i<this->all_shelves.size(); i++){
		            f_out << "   "<< this->all_shelves[i].item.c_str() << " - " <<
                this->all_shelves[i].count << std::endl;
            }

            f_out << "Green Shelf:" << std::endl;
            for(int i=0; i<this->green_shelf.size(); i++){
		            f_out << "   "<< this->green_shelf[i].item.c_str() << " - " <<
                this->green_shelf[i].count << std::endl;
            }

            f_out << "Yellow Shelf:" << std::endl;
            for(int i=0; i<this->yellow_shelf.size(); i++){
                f_out << "   " << this->yellow_shelf[i].item.c_str() << " - " <<
                this->yellow_shelf[i].count << std::endl;
            }

            f_out << "Pink Shelf:" << std::endl;
            for(int i=0; i<this->pink_shelf.size(); i++){
		            f_out<< "   " << this->pink_shelf[i].item.c_str() << " - " <<
                this->pink_shelf[i].count << std::endl;
            }

            ROS_INFO("Inventory file saved as %s", this->inventory_file.c_str());
            f_out.close();

            ROS_INFO("Creating a suggested reorganisation");
            sortShelves();

            ROS_INFO("Creating comparisson file changes.txt");
            fileCompare();
        }
};

/**
              Catalogue Class

  Catalogue class that abstracts writing and reading a catalogue txt file.
  The catalogue contains the possible items that can be scanned by the tallybot.

 **/

class Catalogue{
    std::vector<std::string> catalogue_items;
    ifstream f_in;
    std::string catalogue_file;

    public:

        // Initialise a catalogue file, if it exists already, read it in to memory
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

        /**
                      Item is stocked

        Cross-check a QR code to check if it is an item we stock

        Input: the name of an item
        Output: A boolean of whether or not the item is stocked
        **/
        bool item_is_stocked(std::string item){
            for (int i=0; i<this->catalogue_items.size(); i++){
                if (!catalogue_items[i].compare(item)){
                    return true;
                }
            }
            return false;
        }

        /**
                      Add to Catalogue

        Add an item to the catalogue

        Input: The name of an item
        Output: No direct output, but adds an item to the catalogue
        **/
        void add_to_catalogue(std::string item){
            for (int i=0; i<this->catalogue_items.size(); i++){
                if (!catalogue_items[i].compare(item)){
                    return;
                }
            }
            this->catalogue_items.push_back(item);
            ROS_INFO("Added %s to list of catalogue items", item.c_str());
        }

        /**
                      Add to Catalogue

        Write the catalogue out to disk

        Input:
        Output: Saves the catalogue of items to the disk
        **/
        void save_catalogue(){
            ofstream f_out(this->catalogue_file.c_str());
            for (unsigned int i=0; i<this->catalogue_items.size(); i++){
                f_out << catalogue_items[i] << std::endl;
            }
            f_out.close();
            ROS_INFO("Catalogue file saved to %s", this->catalogue_file.c_str());
        }
};

/**
                Scanner States

Keep track of the current scanner state
**/
enum scannerstate{
    SCANNING,
    NOT_SCANNING
};

// Weather the robot currently expects inventory to be taken
unsigned int tally_state = 0;
void tallyStateCallback(const std_msgs::UInt8 msg){
    tally_state = msg.data;
}

// Item scanner scans the images and stores QR recognitions
class ItemScanner{
    public:
        ros::NodeHandle nh_;
  	    ros::Publisher beep_publisher_;
        ros::Publisher shelf_name_publisher;
	    ros::Subscriber tally_state_subscriber;
        scannerstate scanning_state;
        std::string home = getenv("HOME");

        // Initialise with a pointer to the catalogue and inventory file
        ItemScanner():
            catalogue(this->home + "/catalogue.txt"),
            inventory(this->home + "/inventory.txt")
        {
            beep_publisher_ = nh_.advertise<turtlebot3_msgs::Sound>("/sound", 1);
            shelf_name_publisher = nh_.advertise<std_msgs::String>("/shelfname", 1);
	        tally_state_subscriber = nh_.subscribe<std_msgs::UInt8>("/tally_state", 10, tallyStateCallback);
            scanning_state = NOT_SCANNING;      // Whether to begin adding items to the inventory
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

                if (!this->isInRecentHistory(data) && tally_state==1){
                    ROS_INFO("%s", data.c_str());

                    if (this->catalogue.item_is_stocked(data) && this->scanning_state==SCANNING){
                        this->inventory.add_to_inventory(data);
                        ROS_INFO("Added to inventory: %s", data.c_str());
                        this->inventory.save_inventory_to_disk();
                        // turtlebot3_msgs::Sound beep;
                        // beep.value=1;
                        // this->beep_publisher_.publish(beep);
                    }
                    else if (!data.compare("green shelf") || !data.compare("yellow shelf") || !data.compare("pink shelf")){
                        ROS_INFO("Shelf named detected: %s", data.c_str());
			                  currentShelf = data.c_str();
                        std_msgs::String msg;
                        msg.data = data;
                        shelf_name_publisher.publish(msg);
                        if (this->scanning_state == SCANNING){
                            this->scanning_state = NOT_SCANNING;
                            ROS_INFO("Not scanning anymore");
                        }
                        else{
                            this->scanning_state = SCANNING;
                            ROS_INFO("Start scanning");
                        }
                    }
                    else{
                        ROS_INFO("%s scanned, but ignored.", data.c_str());
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
        Inventory inventory;

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


/**
          Image Converter

Class for converting images to QR codes and colors
**/
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
	ros::init(argc, argv, "inventory_node");
    ImageConverter ic;
    ros::spin();
    // copy_file("/home/joe/inventory.txt", "/home/joe/old_inventory.txt");
    return 0;
}
