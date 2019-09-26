#include <ros/ros.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
//ROS specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>

ros::Publisher pub_cluster,pub_cluster2,pub_cluster_centroid;
bool init_flag;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  
  pcl::PCDWriter writer;
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
  
  // Convert received cloud_msg to PCL data type cloud
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering -> cloud_filtered
  pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
  vg.setInputCloud (cloudPtr);
  vg.setLeafSize (0.2f, 0.2f, 0.2f);
  vg.setFilterLimits(0.0f,2.0f);
  vg.filter (cloud_filtered);
  
  // change representation of cloud_filtered to XYZ cloud_filtered_xyz
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz( new pcl::PointCloud<pcl::PointXYZ> );
  
  //storage for original data
  static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz_origin( new pcl::PointCloud<pcl::PointXYZ> );
  if (init_flag)
  {
      pcl::fromPCLPointCloud2(cloud_filtered, *cloud_filtered_xyz_origin); 
      init_flag = false;
  }
  else  pcl::fromPCLPointCloud2(cloud_filtered, *cloud_filtered_xyz ); 
  
  // Octree resolution - side length of octree voxels
  float resolution = 32.0f;

//   // Instantiate octree-based point cloud change detection class
//   pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);
//   
//   //octree.deleteCurrentBuffer();
//   // Add points from cloud voxel grid representation to octree
//   octree.setInputCloud (cloud_filtered_xyz_origin);
//   octree.addPointsFromInputCloud ();
//   
//   // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
//   octree.switchBuffers ();
//   octree.setInputCloud (cloud_filtered_xyz);
//   octree.addPointsFromInputCloud ();
//   std::vector<int> diffPointIdxVector;
//   
//   // Convert to ROS data type which is published and later processed
  sensor_msgs::PointCloud2 cluster_output;
  sensor_msgs::PointCloud2 cluster_output2;
//   // Get vector of point indices from octree voxels which did not exist in previous buffer
//   octree.getPointIndicesFromNewVoxels (diffPointIdxVector);
//   std::cout << diffPointIdxVector.size() << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_diff (new pcl::PointCloud<pcl::PointXYZ>);
  double x=0.0,y=.0, z=.0;
  for(pcl::PointCloud<pcl::PointXYZ>::iterator cl_it_1 = cloud_filtered_xyz->begin(); cl_it_1 != cloud_filtered_xyz->end(); cl_it_1++){
      bool similar_point_found = false;  
      if (((cl_it_1->x>-2)&&(cl_it_1->x<1.5))&&((cl_it_1->y>0.5)&&(cl_it_1->y<2)))
      {
      for(pcl::PointCloud<pcl::PointXYZ>::iterator cl_it_2 = cloud_filtered_xyz_origin->begin(); cl_it_2 != cloud_filtered_xyz_origin->end(); cl_it_2++)
        {
            if (((cl_it_2->x>-2)&&(cl_it_2->x<1.5))&&((cl_it_2->y>0.5)&&(cl_it_2->y<2)))
            {
                double dist = sqrt(pow(cl_it_1->x-cl_it_2->x,2)+pow(cl_it_1->y-cl_it_2->y,2)+pow(cl_it_1->z-cl_it_2->z,2));
                if (dist<0.2) similar_point_found = true;
            }
        }
        if(!similar_point_found) cloud_diff->points.push_back (*cl_it_1);
      }
    }
  
  //for (std::vector<int>::const_iterator pit = diffPointIdxVector.begin (); pit != diffPointIdxVector.end (); ++pit)
        //calculate cluster centroid
  //  {
  //      
  //      cloud_diff->points.push_back (cloud_filtered_xyz->points[*pit]); //*
  //      
  //  }
  cloud_diff->width = cloud_diff->points.size ();
  cloud_diff->height = 1;
  cloud_diff->is_dense = true;
  //pcl::toROSMsg(*cloud_filtered_xyz_origin,cluster_output);
  pcl::toROSMsg(*cloud_diff,cluster_output2);
  //cluster_output.header.frame_id = "velodyne";
  cluster_output2.header.frame_id = "rslidar";
  //pub_cluster.publish (cluster_output);
  pub_cluster2.publish (cluster_output2);
  
  //extract KDtree which is useful for range and nearest neighbor searches
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_diff);
  
  //Create storage for point clusters cluster_indices, and extract it from voxel point cloud using KDtree representation
  // http://pointclouds.org/documentation/tutorials/cluster_extraction.php
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.2); // 2cm
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (2500);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_diff);
  ec.extract (cluster_indices);

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
      //search all clusters to find their centroid
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    double x=0.0,y=.0, z=.0;
    int iter=0;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        //calculate cluster centroid
    {
      iter++;//number of points
      cloud_cluster->points.push_back (cloud_diff->points[*pit]); //*
      x+=cloud_diff->points[*pit].x;
      y+=cloud_diff->points[*pit].y;
      z+=cloud_diff->points[*pit].z;
        
    }
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    //detection zone x=<0,3>, y=<-0.25,2>
    //if ((x/iter<2.5)&&(x/iter>-2.0)&&(y/iter>-1)&&(y/iter<0))
    //{
        //send cluster vector centroid as ros msg
        //and send the cluster to be displayed in rviz
        geometry_msgs::Vector3 predicted_position;
        pcl::toROSMsg(*cloud_cluster,cluster_output);
        cluster_output.header.frame_id = "rslidar";
        pub_cluster.publish (cluster_output);
        predicted_position.x = x/iter;
        predicted_position.y = y/iter;
        predicted_position.z = z/iter;
        pub_cluster_centroid.publish (predicted_position);
    //}
  }
  
}

int 
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "endorse_cluster_search");
  ros::NodeHandle nh;
  
  init_flag = true;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("rslidar_points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub_cluster = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  pub_cluster2 = nh.advertise<sensor_msgs::PointCloud2> ("output2", 1);
  pub_cluster_centroid = nh.advertise<geometry_msgs::Vector3> ("vector_prediction", 1);
  // Spin
  ros::spin ();
}

