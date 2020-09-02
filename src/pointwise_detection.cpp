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

//minx,miny,minz, max, maxy, maxz
//[0.5,1.5] [-1.5, 1.5] [-0.5, 0.5]
float region_limits[6] = {0.5, -1.5, 0.0, 2.5, 1.5, 1.5};

visualization_msgs::Marker createDangerZoneMarker(){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.ns = "dangerzone";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    float rx = std::abs(region_limits[3] -region_limits[0])/2;
    float ry = std::abs(region_limits[4] -region_limits[1])/2;
    float rz = std::abs(region_limits[5] -region_limits[2])/2;
    
    marker.pose.position.x = region_limits[0] + rx; //1.0;
    marker.pose.position.y = region_limits[1] + ry;//0.0;
    marker.pose.position.z = region_limits[2] + rz;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z =  0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = rx*2;//1.0;
    marker.scale.y = ry*2;//3.0;
    marker.scale.z = rz*2;//1.0;
    marker.color.a = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    return marker;
}


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
        if (cloud->points[n].x > region_limits[0] && cloud->points[n].x < region_limits[3] &&
            cloud->points[n].y > region_limits[1] && cloud->points[n].y < region_limits[4] &&
            cloud->points[n].z > region_limits[2] && cloud->points[n].z < region_limits[5]){
            //ROS_WARN_STREAM("POINT IN DANGER ZONE: "<< cloud->points[n].x);
            local_danger_flag = true;
            break;
        }
    }
    ROS_ERROR_STREAM("finish PC "<< local_danger_flag);
    point_in_danger = local_danger_flag;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointwise_safe_detections");
    ros::NodeHandle nh;

    ros::Subscriber vector_prediction = nh.subscribe("/velodyne_points/filtered", 1, vector_prediction_callback);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("safe_marker", 1000);
    //predicted_position.x = -10;

    ros::Publisher danger_marker_pub = nh.advertise<visualization_msgs::Marker>("danger_zone", 1000);
    visualization_msgs::Marker danger_maker = createDangerZoneMarker();

    ros::Rate loop_rate(10);
    while (nh.ok()) 
    {
        std_msgs::Int8 zone1_detected;
        zone1_detected.data=0;
        //Create visualization marker (cylinder) and place it in robot workspace
        visualization_msgs::Marker marker;
        marker.header.frame_id = "velodyne";
        marker.header.stamp = ros::Time::now();
        danger_maker.header.stamp = ros::Time::now();

        marker.ns = "zone1";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0.0;
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
        danger_marker_pub.publish(danger_maker);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

