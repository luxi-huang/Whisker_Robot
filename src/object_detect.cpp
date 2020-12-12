#include "whisker/object_detect.hpp"

using namespace std;

namespace object_detect
{
    ObjectDetect::ObjectDetect(ros::NodeHandle* nodehandle):nh_(*nodehandle)
	{
		/* THIS IS CLASS CONSTRCTOR */
		initial_publishers_subscribers();
        read_files();

        ros::Rate loop_rate(40.0);

        while (ros::ok())
        {
            analyze_data ();
            ros::spinOnce();
            loop_rate.sleep();
        }
	}

    void ObjectDetect::initial_publishers_subscribers()
    {
        /* initial publishers and subscribers*/
        pub_whisker_collision = nh_.advertise<visualization_msgs::Marker> ("dp_marker", 1); 
        pub_obstable = nh_.advertise<visualization_msgs::Marker> ("obstacle_marker", 1);
        cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("cloud", 50);
    }

    vector<vector<float>> ObjectDetect::read_csv(string filename) 
    {
        /* Read csv files*/
        ifstream classFile(filename);

        if(!classFile.is_open()) throw runtime_error("Could not open file");
        
        vector<vector<float>> classData;
        string line;
        string field;
        int count = 0;
        

        while(getline(classFile,line)) 
        {
            istringstream s(line);
            classData.push_back({});
            
            while (getline(s,field, ',')) 
            {
                float val;
                val = stoi(field);
                classData.back().push_back(val);
            }
        }
        
        return classData;
    }

    void ObjectDetect::read_files() 
    {
        /* Read files from the path */
        collision = read_csv("/home/luxi/github_projects/Whisker_Robot/src/whisker/data/output/test/kinematics/c/RC0.csv");
        x_val = read_csv("/home/luxi/github_projects/Whisker_Robot/src/whisker/data/output/test/kinematics/x/RC0.csv");
        y_val = read_csv("/home/luxi/github_projects/Whisker_Robot/src/whisker/data/output/test/kinematics/y/RC0.csv");
        z_val = read_csv("/home/luxi/github_projects/Whisker_Robot/src/whisker/data/output/test/kinematics/z/RC0.csv");
    }

    void ObjectDetect::analyze_data() 
    {
        /* Analyze data from csv files */

        // inital variables 
        int row = collision.size();
        int count = 0;
        ros::Rate sleep_rate(1.0);
        vector<vector<float>> points; // the points of whisker detect obstacles.
        
        // std::cout<<"row: "<< row <<"\n";
        
        for (int i=0; i < row; i++) 
        {
            // points.push_back({});
            for (int j = 0; j < 20; j++) 
            {
                if (collision[i][j] == 1) 
                {   
                    float x_point = (x_val[i][j] + x_val[i][j+1]) / 2 / 100;
                    float y_point = (y_val[i][j] + y_val[i][j+1]) / 2 / 100;
                    float z_point = (z_val[i][j] + z_val[i][j+1]) / 2 / 100;

                    points.push_back({x_point,y_point,z_point});
                    count ++;
                    ////////////////////////////////
                         //publish pointclouds 
                        // if (count > 100) {
                        //     num_points = count;
                        //     sensor_msgs::PointCloud cloud;
                        //     cloud.header.stamp = ros::Time::now();
                        //     cloud.header.frame_id = referenceFrame;
                        //     cloud.points.resize(num_points);

                        //     //we'll also add an intensity channel to the cloud
                        //     cloud.channels.resize(1);
                        //     cloud.channels[0].name = "intensities";
                        //     cloud.channels[0].values.resize(num_points);

                        //     for(unsigned int i = 0; i < num_points; i++) {
                        //         cloud.points[i].x = points[i][0];
                        //         cloud.points[i].y = points[i][1];
                        //         cloud.points[i].z = abs(points[i][2]);
                        //         cloud.channels[0].values[0] = 100 + i;
                        //     } 
                        //     cloud_pub.publish(cloud); 
                        //     sleep_rate.sleep();

                        //     points.clear(); 
                        //     count = 0; 
                        // }

                    //////////////////////////////
                    
                    visualization_msgs::Marker dp_marker;
                    dp_marker.header.frame_id = referenceFrame;
                    dp_marker.header.stamp = ros::Time::now() - ros::Duration(0.05);
                    dp_marker.ns = "whisker_point";
                    dp_marker.id = scani;
                    dp_marker.type = visualization_msgs::Marker::SPHERE;
                    dp_marker.action = visualization_msgs::Marker::ADD;
                    dp_marker.pose.position.x = x_point;
                    dp_marker.pose.position.y = y_point;
                    dp_marker.pose.position.z = z_point;
                    dp_marker.pose.orientation.x = 0.0;
                    dp_marker.pose.orientation.y = 0.0;
                    dp_marker.pose.orientation.z = 0.0;
                    dp_marker.pose.orientation.w = 1.0;
                    dp_marker.scale.x = 0.25;
                    dp_marker.scale.y = 0.25;
                    dp_marker.scale.z = 0.25;
                    dp_marker.color.a = 1.0;
                    dp_marker.color.r = 0.0f;
                    dp_marker.color.g = 0.0f;
                    dp_marker.color.b = 1.0f;
                    dp_marker.lifetime = ros::Duration(1.0);
                    pub_whisker_collision.publish(dp_marker);

                    visualization_msgs::Marker obstacle_marker;
                    obstacle_marker.header.frame_id = referenceFrame;
                    obstacle_marker.header.stamp = ros::Time::now() - ros::Duration(0.05);
                    obstacle_marker.ns = "obstable_point";
                    obstacle_marker.id = scani;
                    obstacle_marker.type = visualization_msgs::Marker::CYLINDER;
                    obstacle_marker.action = visualization_msgs::Marker::ADD;
                    obstacle_marker.pose.position.x = 20.0;
                    obstacle_marker.pose.position.y = 25.0;
                    obstacle_marker.pose.position.z = 0.0;
                    obstacle_marker.pose.orientation.x = 0.0;
                    obstacle_marker.pose.orientation.y = 0.0;
                    obstacle_marker.pose.orientation.z = 0.0;
                    obstacle_marker.pose.orientation.w = 1.0;
                    obstacle_marker.scale.x = 1.0;
                    obstacle_marker.scale.y = 1.0;
                    obstacle_marker.scale.z = 80;
                    obstacle_marker.color.a = 1.0;
                    obstacle_marker.color.r = 1.0f;
                    obstacle_marker.color.g = 0.0f;
                    obstacle_marker.color.b = 0.0f;
                    obstacle_marker.lifetime = ros::Duration(1.0);
                    pub_obstable.publish(obstacle_marker);
                    scani ++;   

                    // sleep_rate.sleep();    
                } 
            }   
        }

        //publish pointclouds 
        num_points = count;
        sensor_msgs::PointCloud cloud;
        cloud.header.stamp = ros::Time::now();
        cloud.header.frame_id = referenceFrame;
        cloud.points.resize(num_points);

        //we'll also add an intensity channel to the cloud
        cloud.channels.resize(1);
        cloud.channels[0].name = "intensities";
        cloud.channels[0].values.resize(num_points);

        for(unsigned int i = 0; i < num_points; i++) {
            cloud.points[i].x = points[i][0];
            cloud.points[i].y = points[i][1];
            cloud.points[i].z = abs(points[i][2]);
            cloud.channels[0].values[0] = 100 + i;
        } 
        cloud_pub.publish(cloud);   
    }
}    