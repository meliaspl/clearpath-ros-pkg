// Includes

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "Eigen/Dense"

#include "Vector3.h"
#include "ClearpathDemoTools.h"

#include "geometry_msgs/Twist.h"
#include "turtlebot_node/SetTurtlebotMode.h"

// Slowest permissible is 10Hz
#define MAX_RUNTIME_SECONDS 0.1f

// Typedef PCL Clouds
typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PCLPointCloud;

// Publishers
ros::Publisher pubvel;

// ROS Param Variables
double lin_speed = 0.3;
double ang_speed = 0.75;
double still_ang_speed = 2.0;
double cam_height = 0.55;
double stop_distance = 0.8;
int stop_point_count = 50;

// Floor Normal
bool normal_initialized = false;
double base_normal[4] = {0.0};
double last_normal[4] = {0.0};

// PointClouds
PointCloud cloudCopy;
PointCloud cloudCut;
PointCloud upright;
PointCloud slice;

// Timers
double start_command = 0.0;
double turn_time = 0.0;

// Robot States
double my_ang_speed = 0.0;
bool stop_mode = false;

// Main Callback
void callback(const PCLPointCloud::ConstPtr& cloud)
{
    // Clear all point clouds
    cloudCopy.points.clear();
    cloudCut.points.clear();
    upright.points.clear();
    slice.points.clear();

    // Start Timer
    ros::Time begin = ros::Time::now();


    // Convert point cloud to the Clearpath type (could optimize by removing this... unfortunately ClearpathDemoTools has to work without ROS and PCL)
    for (int i = 0; i < cloud->points.size(); i++)
    {
        Point p;  p.x = cloud->points[i].x; p.y = cloud->points[i].y; p.z = cloud->points[i].z; p.rgb = cloud->points[i].rgb;
        cloudCopy.points.push_back(p);
    }

    // Initialize the floor plane one time.
    if (normal_initialized == false)
    {
        // Guess at Plane Points
        ClearpathDemoTools::VerticalCut(&cloudCopy, &cloudCut, base_normal[3]-0.3, base_normal[3]+0.3, 13);

        // Find the plane using RANSAC and Least Squares
        double norm[4];
        norm[0] = last_normal[0]; norm[1] = last_normal[1]; norm[2] = last_normal[2]; norm[3] = last_normal[3];
        if ( ClearpathDemoTools::PlaneRANSAC(&cloudCut, &norm[0], true) ) {
            ClearpathDemoTools::PlaneLS(&cloudCut, &norm[0]);
            last_normal[0] = norm[0]; last_normal[1] = norm[1]; last_normal[2] = norm[2]; last_normal[3] = norm[3];
            normal_initialized = true;
        } 
    }

    // Transform points such that the xz plane is where the calibrated floor was
    ClearpathDemoTools::TransformByNormal(&cloudCopy, &upright, &last_normal[0]);

    // Trim the cloud and only keep the region above the floor
    ClearpathDemoTools::VerticalCut(&upright, &slice, -1.0, -0.02, 1);
    
    // Find closest point
    unsigned closeCount = 0;
    for ( int i = 0; i < slice.points.size(); i++ )
    {
        double d = sqrt(slice.points[i].x*slice.points[i].x + slice.points[i].z*slice.points[i].z);
        if ( d < stop_distance )
            closeCount++;
    }

    // Choose Velocity
    geometry_msgs::Twist tw;
    tw.linear.x = 0.0;
    tw.angular.z = 0.0;
    
    if (stop_mode == true) // Was previously stopped and turning, continue unless breaks timeout
    {
        if (ros::Time::now().toSec() - start_command > turn_time) // Check if turn has timed out
        {
            stop_mode = false;
            my_ang_speed = 0.0;
        }
    } 
    else // Do motion as normal
    {
        // Generate random numbers
        srand ( time(NULL) );
        int rndm2 = rand() % 2; // 0 or 1
        int rndm3 = rand() % 3 - 1; // -1, 0 or 1

        if (closeCount > stop_point_count) // A cluster of points is within unsafe distance, stop!
        {
            stop_mode = true;

            // Choose a direction
            if (rndm2 == 1) my_ang_speed = still_ang_speed;
            else my_ang_speed = -still_ang_speed;

            // Start timer
            start_command = ros::Time::now().toSec();

            // Choose a random timeout
            turn_time = double(rand() % 3 + 1)*0.5;
        } 
        else // Safe to travel
        {
            // Has been travelling for 1 second safely, switch motion type
            if (ros::Time::now().toSec() - start_command > 1.0)
            {
                my_ang_speed = rndm3*ang_speed;   
                start_command = ros::Time::now().toSec();
            }
            
            // Set linear speed
            tw.linear.x = lin_speed;
        }
    }

    // Set angular speed
    tw.angular.z = my_ang_speed;
    pubvel.publish(tw);

    ros::Duration duration = ros::Time::now() - begin;
    if (duration.toSec() > MAX_RUNTIME_SECONDS) {
      ROS_WARN("Iteration of vision loop took %f seconds (max is %f)", duration.toSec(), MAX_RUNTIME_SECONDS);
    }
}

int main(int argc, char** argv)
{
    ros::init (argc, argv, "turtle_default");    

    // Get ROS Params
    ros::NodeHandle nh = ros::NodeHandle(ros::this_node::getName());
    std::string in, out, out_vel;
    nh.param<std::string>("in", in, "rndmwalk_in");
    nh.param<std::string>("out_vel", out_vel, "rndmwalk_out_vel");
    nh.param<double>("lin_speed", lin_speed, 0.3);
    nh.param<double>("ang_speed", ang_speed, 0.75);
    nh.param<double>("still_ang_speed", still_ang_speed, 2.0);
    nh.param<double>("cam_height", cam_height, 0.3);
    nh.param<double>("stop_distance", stop_distance, 0.8);
    nh.param<int>("stop_point_count", stop_point_count, 50);

    // Init Timer
    start_command = ros::Time::now().toSec() - 5.0;

    // Subscribe to Kinect Cloud
    ros::NodeHandle nx;
    ros::Subscriber sub = nx.subscribe<PCLPointCloud>(in, 1, callback);

    // Set to FULL Turtlebot Mode
    ros::ServiceClient operation_mode_client;
    operation_mode_client = nx.serviceClient<turtlebot_node::SetTurtlebotMode>("/turtlebot_node/set_operation_mode");
    turtlebot_node::SetTurtlebotMode srvMsg;
    srvMsg.request.mode = 3;
    operation_mode_client.call(srvMsg);

    // Open up publisher channel
    pubvel = nx.advertise<geometry_msgs::Twist> (out_vel, 1);

    // Initialize Floor Plane Normal
    base_normal[0] = 0.0;
    base_normal[1] = 1.0;
    base_normal[2] = 0.0;
    base_normal[3] = cam_height;
    last_normal[0] = base_normal[0];
    last_normal[1] = base_normal[1];
    last_normal[2] = base_normal[2];
    last_normal[3] = base_normal[3];

    // Spin
    ros::spin();

    return (0);
}



