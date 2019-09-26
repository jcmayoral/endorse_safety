#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <math.h>
#include <visualization_msgs/Marker.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//global variable to store position of the cluster centroid once we receive it
//TO DO: create node with class/object

geometry_msgs::Vector3 predicted_position;
void vector_prediction_callback(const geometry_msgs::Vector3 msg)
{
	predicted_position.x = msg.x;
	predicted_position.y = msg.y;
	predicted_position.z = msg.z;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cluster_zone_detection");
    ros::NodeHandle nh;

    ros::Subscriber vector_prediction = nh.subscribe("vector_prediction", 1, vector_prediction_callback);
    ros::Publisher zone1_pub=nh.advertise<std_msgs::Int8>("zone1_detection", 1000);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
    predicted_position.x = -10;
    ros::Rate loop_rate(10);
    while (nh.ok()) 
    {
        std_msgs::Int8 zone1_detected;
        zone1_detected.data=0;
        //Create visualization marker (cylinder) and place it in robot workspace
        visualization_msgs::Marker marker;
        marker.header.frame_id = "rslidar";
        marker.header.stamp = ros::Time();
        marker.ns = "zone1";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 1.;
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
        double R_distance = sqrt(pow(predicted_position.x-marker.pose.position.x,2)+pow(predicted_position.y-marker.pose.position.y,2));
        if(R_distance < 2 && R_distance > 1.5)
        {
            marker.color.a = 0.7;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            zone1_detected.data=1;
        }
        else if (R_distance <= 1.5)
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
        zone1_pub.publish(zone1_detected);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

