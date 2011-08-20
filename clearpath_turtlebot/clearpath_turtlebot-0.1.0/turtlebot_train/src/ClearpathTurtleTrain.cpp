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

// Slowest permissible is 5Hz
#define MAX_RUNTIME_SECONDS 0.2f

// Typedef PCL Clouds
typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PCLPointCloud;

// Turtle Structure
struct Turtle
{
    double x;
    double z;
    int count;
};

// Publishers
ros::Publisher pub;
ros::Publisher pubvel;

// ROS Param Variables
int debug_draw = 0;
int num_ransac_iter = 500;
double angular_vel_correction = 1.0;
double lin_speed = 0.3;
double lin_speed_var = 0.1;
double ang_speed = 0.2;
double cam_height = 0.55;
double bot_window_size = 0.5;
double bot_radius = 0.165;
double ransac_thresh = 0.02;
double ang_speed_gain = 1.0;
double target_dist = 1.2;
double min_stop_dist = 0.9;

// Floor Normal
bool normal_initialized = false;
double base_normal[4] = {0.0};
double last_normal[4] = {0.0};

// Target Window
double targetX = 0.0;
double targetZ = 1.5;

// PointClouds
PointCloud cloudCopy;
PointCloud cloudCut;
PointCloud upright;
PointCloud area;
PointCloud slice;
PointCloud inliers;
PointCloud final;

// Trim the cloud by only using points inside a certain a given circle (x, z, radius)
void GetSearchArea(PointCloud* cloud, PointCloud* cloudout, double x, double z, double radius)
{
    cloudout->points.clear();
    for (unsigned int i = 0; i < cloud->points.size(); i++)
    {   
        double mag = (cloud->points[i].x-x)*(cloud->points[i].x-x) + (cloud->points[i].z-z)*(cloud->points[i].z-z);
        if ( mag < radius*radius)
        {
            cloudout->points.push_back(cloud->points[i]);
        }
    }
}

// Randomly take 2 points in the cloud, project them on the xz plane and then return the two possible circle centers given a radius
bool GetCircleFromRnd2(PointCloud* cloud, double* xzxz, double radius)
{
    int i1, i2, i3, num;
    i1 = i2 = i3 = 0;
    num = cloud->points.size();
    
    if (num > 10) // less than 10 points is too few
    {
        Vector3 a1, a2;

        unsigned int counter = 0;
        while (i2 == i1 || (a1-a2).Length() > radius*2.0 || (a1-a2).Length() < 0.15) { // choose points that aren't too far away, not too close and not the exact same point
            counter++;
            if (counter > 5) // if we've done this 5 times without success, just quit, it probably doesn't exist or is hard to find
                return false;
            i1 = rand() % num; 
            i2 = rand() % num; 
            a1.x = cloud->points[i1].x; a1.y = 0.0; a1.z = cloud->points[i1].z;
            a2.x = cloud->points[i2].x; a2.y = 0.0; a2.z = cloud->points[i2].z;
        }

        // Calculate the two circle centers given the two selected points
        Vector3 ax, ay, az;
        ax = a2 - a1; ax.Normalize();
        ay.x = 0.0; ay.y = 1.0; ay.z = 0.0;
        az = ax.Cross(ay); az.Normalize();
        double d1 = ((a1-a2).Length()/2.0);
        double d2 = sqrt(radius*radius - d1*d1);
        Vector3 p1, p2;
        p1 = a1 + ax*d1 + az*d2;
        p2 = a1 + ax*d1 - az*d2;

        // Fill return array
        xzxz[0] = p1.x;
        xzxz[1] = p1.z;
        xzxz[2] = p2.x;
        xzxz[3] = p2.z;

    } else {
        return false;
    }

    return true;
}

double distBetweenTurtles(Turtle t1, Turtle t2)
{
    return sqrt((t1.x-t2.x)*(t1.x-t2.x) + (t1.z-t2.z)*(t1.z-t2.z));
}

// Finds all objects with a high enough quality to be considered a turtle
bool FindTurtles(PointCloud* cloud, std::vector<Turtle>* turtles, double radius, double thresh)
{
    turtles->clear();

    // Make it more random
    srand ( time(NULL) );

    int iter_count = 0;   
    double bestxz[2];
    while (iter_count < num_ransac_iter) // for x iterations:
    {   
        iter_count++;  
        
        double xzxz[4];
        
        // Get two circle centers from a random 2 points
        if (!GetCircleFromRnd2(cloud, &xzxz[0], radius))
            continue;
        
        // Try each circle
        for (unsigned int c = 0; c < 2; c++)
        {
            int count = 0;      

            Vector3 circ;
            circ.x = xzxz[c*2+0];
            circ.y = 0.0f;
            circ.z = xzxz[c*2+1];

            // Check to see how many points in the cloud lie on the surface of this cylinder (within a threshold)
            for (unsigned int i = 0; i < cloud->points.size(); i++)
            {   
                Vector3 a1;
                a1.x = cloud->points[i].x; a1.y = 0.0f; a1.z = cloud->points[i].z;
                double mag = sqrt((a1.x-circ.x)*(a1.x-circ.x) + (a1.z-circ.z)*(a1.z-circ.z)) - radius;
                if ( fabs(mag) < thresh) // Check if distance is less than threshold
                    count++;
            }
            
            // If the current consensus is better than a count of X
            if ( count > 75 )
            {
                // Make a turtle
                Turtle t; t.x = circ.x; t.z = circ.z; t.count = count;
                bool replica = false;

                // Check to see if we already have this turtle
                for (int i = 0; i < turtles->size(); i++)
                {
                    if (distBetweenTurtles((*turtles)[i], t) < radius/2.0)
                    {
                        replica = true;
                        if (t.count > (*turtles)[i].count)
                            (*turtles)[i] = t;
                    }
                }

                // Brand new turtle
                if (replica == false)
                    turtles->push_back(t);
            }
        }
    }

    // Check that the best consensus has atleast x votes, otherwise it is probably garbage
    if (turtles->size() < 1)
        return false;

    return true;
}

// Main Callback
void callback(const PCLPointCloud::ConstPtr& cloud)
{
    bool found_target = false;

    // Clear all point clouds
    cloudCopy.points.clear();
    cloudCut.points.clear();
    upright.points.clear();
    area.points.clear();
    slice.points.clear();
    inliers.points.clear();
    final.points.clear();

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

    // Trim the cloud and only keep a small vertical portion (where the Create is)
    ClearpathDemoTools::VerticalCut(&upright, &slice, -0.08, -0.02, 1);

    // Trim the cloud again such that we only keep poings inside the search window
    GetSearchArea(&slice, &area, targetX, targetZ, bot_radius+bot_window_size);
    
    // Find all possible Turtles
    std::vector<Turtle> turtles;
    if ( FindTurtles(&slice, &turtles, bot_radius, ransac_thresh) )
    {
        // Search through turtles for the best turtle inside the window
        Turtle t; t.x = targetX; t.z = targetZ;
        int besti = 0;
        int bestcount = 0;
        for (int ti = 0; ti < turtles.size(); ti++)
        {
            if (distBetweenTurtles(t, turtles[ti]) < bot_window_size)
            {
                if (turtles[ti].count > bestcount)
                {
                    bestcount = turtles[ti].count;
                    besti = ti;
                    found_target = true;
                }
            }
        }

        // Set the new target
        if (found_target == true)
        {
            targetX = turtles[besti].x;
            targetZ = turtles[besti].z;
        }
    }

    // Print out our new guess
    printf("Chasing the robot @ %fm in X and %fm in Z \n", targetX, targetZ);
    
    // We found a target
    if (found_target == true) 
    { 
        // We are following! calculate the radial path that intersects us with the targetXZ
        double distToTarget = sqrt(targetX*targetX + targetZ*targetZ);
        if (distToTarget > min_stop_dist) {
            Twist tw2 = ClearpathDemoTools::DetermineVelocity(targetX, targetZ, lin_speed);
            geometry_msgs::Twist tw;
            
            double err = distToTarget-target_dist; // positive means go faster
            tw.linear.x = tw2.linear + lin_speed_var*err/(target_dist-min_stop_dist);

            tw.angular.z = ang_speed_gain*tw2.angular; // although tw2.angular should put us right on the target, this usually needs a push..
            
            pubvel.publish(tw);
        }
        // We are within stop distance, turn to face the robot
        else { 
            geometry_msgs::Twist tw;
            tw.linear.x = 0.0;
            double tempspd = ang_speed;
            if (fabs(atan2(targetX,targetZ)) < 10.0/180.0*3.141592)
                tempspd = 0.0;
            tw.angular.z = -ang_speed_gain*copysign(tempspd, targetX);
            pubvel.publish(tw);
        }
    } else {
        // Publish zero velocity
        geometry_msgs::Twist tw;
        tw.linear.x = 0.0;
        tw.angular.z = 0.0;
        pubvel.publish(tw);
    }

    if (debug_draw == 1)
    {
        PCLPointCloud f;
        for (int ti = 0; ti < turtles.size(); ti++)
        {
            PointCloud area2, red, final2;
            GetSearchArea(&upright, &area2, turtles[ti].x, turtles[ti].z, bot_radius+bot_window_size);

            // Get area around the guess for visualization
            ClearpathDemoTools::VerticalCut(&area2, &red, -0.02, 0.02, 1);
            ClearpathDemoTools::VerticalCut(&area2, &final2, -0.08, -0.02, 1);

            // Convert back to PCL Type for rendering
            for (int i = 0; i < final2.points.size(); i++)
            {
                PointType p;  p.x = final2.points[i].x; p.y = final2.points[i].y; p.z = final2.points[i].z; p.rgb = final2.points[i].rgb;
                double err = sqrt((p.x-turtles[ti].x)*(p.x-turtles[ti].x) + (p.z-turtles[ti].z)*(p.z-turtles[ti].z)) - bot_radius;
                if ( fabs(err) < ransac_thresh) // Check if distance is less than threshold
                    f.points.push_back(p);
            }

            // Choose Color for Floor Points
            char c[4];
            c[0] = 0;             c[1] = 0;             c[2] = 0; 
            if ( sqrt((targetX-turtles[ti].x)*(targetX-turtles[ti].x) + (targetZ-turtles[ti].z)*(targetZ-turtles[ti].z)) < 0.0001)
                c[1] = 255;
            else
                c[2] = 255;
            float* cf = (float*)(&c[0]);

            // Add Floor Points to show circle
            for (int i = 0; i < red.points.size(); i++)
            {
                PointType p;  p.x = red.points[i].x; p.y = red.points[i].y; p.z = red.points[i].z; p.rgb = *cf;
                f.points.push_back(p);
            }
        }

        // Publish the visualization cloud
        pub.publish(f);
    }

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
    nh.param<std::string>("in", in, "idfloor_in");
    nh.param<std::string>("out", out, "idfloor_out");
    nh.param<std::string>("out_vel", out_vel, "idfloor_out_vel");

    nh.param<double>("lin_speed", lin_speed, 0.3);
    nh.param<double>("lin_speed_var", lin_speed_var, 0.1);
    nh.param<double>("ang_speed", ang_speed, 0.3);
    nh.param<double>("ang_speed_gain", ang_speed_gain, 1.0);
    nh.param<double>("target_dist", target_dist, 1.2);
    nh.param<double>("min_stop_dist", min_stop_dist, 0.9);

    nh.param<double>("cam_height", cam_height, 0.55);
    nh.param<double>("bot_radius", bot_radius, 0.165);

    nh.param<double>("ransac_thresh", ransac_thresh, 0.02);
    nh.param<int>("num_ransac_iter", num_ransac_iter, 500);

    double window_init_targetX, window_init_targetZ;
    nh.param<double>("window_size", bot_window_size, 0.5);
    nh.param<double>("window_init_targetX", window_init_targetX, 0.0);
    nh.param<double>("window_init_targetZ", window_init_targetZ, 1.5);
    targetX = window_init_targetX;
    targetZ = window_init_targetZ;

    nh.param<int>("debug_draw", debug_draw, 0);

    // Subscribe to Kinect Cloud
    ros::NodeHandle nx;
    ros::Subscriber sub = nx.subscribe<PCLPointCloud>(in, 1, callback);

    // Set to FULL Turtlebot Mode
    ros::ServiceClient operation_mode_client;
    operation_mode_client = nx.serviceClient<turtlebot_node::SetTurtlebotMode>("/turtlebot_node/set_operation_mode");
    turtlebot_node::SetTurtlebotMode srvMsg;
    srvMsg.request.mode = 3;
    operation_mode_client.call(srvMsg);

    // Open up publisher channels
    pub = nx.advertise<PCLPointCloud> (out, 1);
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



