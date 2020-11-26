#ifndef OBJECT_DETECT_HPP
#define OBJECT_DETECT_HPP

//*****add library******//
// ROS Headers
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

// System Headers
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream

using namespace std;
namespace object_detect
{
    class ObjectDetect 
    {
    private:
        //***************** NODE HANDLES ***************//
        ros::NodeHandle nh_;

        //***************** NODE PUBLISHER ***************//
        ros::Publisher pub_whisker_collision;
        ros::Publisher pub_obstable; 

        //***************** REFERENCE FRAME ***************//

        //***************** OTHER VARIABLES ***************//
        geometry_msgs::Quaternion fake_quaterion;
        int scani = 0;  
        vector<vector<float>> collision; 
        vector<vector<float>> x_val;
        vector<vector<float>> y_val;
        vector<vector<float>> z_val;

    public:
        ObjectDetect(ros::NodeHandle* nodehandle);
        void initial_publishers_subscribers();
        vector<vector<float> > read_csv(string filename);
        void read_files();
        void analyze_data(); 
        const char *referenceFrame = "/base_link";
    };   
}

#endif