#include <string>
#include <vector>

#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "Eigen/Eigen"
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>


static ros::Publisher cloud_pub;
static ros::Publisher outliers_pub;
static ros::Publisher floor_pub;

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &scan) {

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    sensor_msgs::PointCloud2::Ptr ros_cloud(new sensor_msgs::PointCloud2 ());


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outliers (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cleaned (new pcl::PointCloud<pcl::PointXYZ>);


    // set time stamp and frame id
    ros::Time tstamp = ros::Time::now();

    // Convert to pcl
    ROS_DEBUG("Convert incoming cloud to pcl cloud");
    pcl::fromROSMsg(*scan, *cloud);

    Eigen::Vector3f axis = Eigen::Vector3f(0.0,1.0,0.0);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setAxis(axis);
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setEpsAngle(30.0 * 3.14/180.);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    // Get inliers
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.filter (*output);

    // Get Outliers
    extract.setNegative (true);
    extract.filter (*outliers);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
    }

    // std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
    //                                     << coefficients->values[1] << " "
    //                                     << coefficients->values[2] << " " 
    //                                     << coefficients->values[3] << std::endl;

    for (const auto& point : outliers -> points){
        if (point.y < coefficients->values[3]){
            cleaned->points.push_back(point);
        }
    }

    pcl::toROSMsg(*output, *ros_cloud);
    ros_cloud->header.frame_id = scan->header.frame_id;
    cloud_pub.publish(ros_cloud);

    pcl::toROSMsg(*cleaned, *ros_cloud);
    ros_cloud->header.frame_id = scan->header.frame_id;
    outliers_pub.publish(ros_cloud);

    // for (const auto& idx : inliers->indices){
    //     floor->points.push_back(cloud->points[idx]);
    // }
    // pcl::toROSMsg(*floor, *ros_cloud);
    // ros_cloud->header.frame_id = scan->header.frame_id;
    // floor_pub.publish(ros_cloud);

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "surface_extractor");
  ros::NodeHandle nh;

//   ros::Publisher marker_pub =
//       nh.advertise<visualization_msgs::Marker>("surface_objects", 100);
//   ros::Publisher cropped_input_pub = nh.advertise<sensor_msgs::PointCloud2>(
//       "demo_cropped_input_cloud", 1, true);

//   std::string target_frame("base_link");
//   if (argc > 1) {
//     target_frame = argv[1];
//   }

//   SurfaceViz viz(marker_pub);
//   Demo demo(viz, target_frame, cropped_input_pub);

    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/inliers", 1);
    outliers_pub = nh.advertise<sensor_msgs::PointCloud2>("/outliers", 1);
    floor_pub = nh.advertise<sensor_msgs::PointCloud2>("/floor", 1);

    ros::Subscriber pc_sub = nh.subscribe<sensor_msgs::PointCloud2>(
        "cloud_in", 1, cloud_callback);
    ros::spin();
}