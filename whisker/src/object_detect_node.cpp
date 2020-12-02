#include "whisker/object_detect.hpp"

using namespace object_detect;
int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "object_detect");
    ros::NodeHandle n;
    ObjectDetect object_detect_class(&n);
}

