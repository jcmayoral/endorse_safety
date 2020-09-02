#include <ros/ros.h>
//#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/foreach.hpp>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <math.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//global variable to store position of the cluster centroid once we receive it
//TO DO: create node with class/object
bool point_in_danger;

void vector_prediction_callback(const sensor_msgs::PointCloud2Ptr cloud_msg)
{
    /*
    if (msg->poses.size() == 0)
        return;
	predicted_position.x = msg->poses[0].position.x;
	predicted_position.y = msg->poses[0].position.y;
	predicted_position.z = msg->poses[0].position.z;
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // Convert received cloud_msg to PCL data type cloud
    pcl::fromROSMsg(*cloud_msg, *cloud);
    //pcl_conversions::toPCL(*cloud_msg, *cloud);

    bool local_danger_flag = false;

    for(int n = 0; n < cloud->points.size(); n++){
        if (cloud->points[n].x > 0.5 && cloud->points[n].x < 1.5 ){
            ROS_WARN_STREAM("POINT IN DANGER ZONE: "<< cloud->points[n].x);
            local_danger_flag = true;
            break;
        }
    }
    point_in_danger = local_danger_flag;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointwise_safe_detections");
    ros::NodeHandle nh;

    ros::Subscriber vector_prediction = nh.subscribe("/velodyne_points/filtered", 1, vector_prediction_callback);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("safe_marker", 1000);
    //predicted_position.x = -10;
    ros::Rate loop_rate(10);
    while (nh.ok()) 
    {
        std_msgs::Int8 zone1_detected;
        zone1_detected.data=0;
        //Create visualization marker (cylinder) and place it in robot workspace
        visualization_msgs::Marker marker;
        marker.header.frame_id = "velodyne";
        marker.header.stamp = ros::Time();
        marker.ns = "zone1";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0.25;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z =  0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.5;
        marker.scale.y = 1.5;
        marker.scale.z = 1.5;

        //check if cluster centroid enters zone 1, zone 2 or zone 3

        if (point_in_danger)
        {
            marker.color.a = 0.7;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            zone1_detected.data=2;
        }
        else
        {
            marker.color.a = 0.7;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        marker_pub.publish(marker);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

