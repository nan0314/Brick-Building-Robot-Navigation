#include <iostream>
#include <fstream>
#include <memory>
#include <cmath>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vector>
#include <thread>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>



//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------
#define MAX_CLUSTERS 10
typedef pcl::PointXYZ PointT;
std::string filename;

//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------

class ClusterExtractor
{

private:
    ros::NodeHandle n_;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub[MAX_CLUSTERS];
    ros::Publisher point_pub[MAX_CLUSTERS];
    tf::TransformBroadcaster br;


public:
    ClusterExtractor()
    {
        ROS_DEBUG("Creating subscribers and publishers");
        cloud_sub = n_.subscribe("/outlier/cutoff/output", 10, &ClusterExtractor::cloudcb, this);
        br = tf::TransformBroadcaster();
        for(int i = 0; i < MAX_CLUSTERS; i++)
        {
            cloud_pub[i] = n_.advertise<sensor_msgs::PointCloud2>("/cluster_" + std::to_string(i + 1) + "_cloud", 1);
            point_pub[i] = n_.advertise<geometry_msgs::PointStamped>("/cluster_" + std::to_string(i + 1) + "_point", 1);
        }
    }

    // this function gets called every time new pcl data comes in
    void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan)
    {
        ROS_DEBUG("Filtered cloud receieved");
        sensor_msgs::PointCloud2::Ptr ros_cloud(new sensor_msgs::PointCloud2 ());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

        // set time stamp and frame id
        ros::Time tstamp = ros::Time::now();

        // Convert to pcl
        ROS_DEBUG("Convert incoming cloud to pcl cloud");
        pcl::fromROSMsg(*scan, *cloud);
       
        ////////////////////////////////////////
        // STARTING CLUSTER EXTRACTION    //
        ////////////////////////////////////////
        ROS_DEBUG("Begin cluster extraction");

        // create a vector for storing the indices of the clusters
        std::vector<pcl::PointIndices> cluster_indices;

        // setup extraction:
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.01); // cm
        ec.setMinClusterSize (75);
        ec.setMaxClusterSize (2000);
        ec.setInputCloud (cloud);
        // perform cluster extraction
        ec.extract (cluster_indices);

        // run through the indices, create new clouds, and then publish them
        int j=0;
        int number_clusters=0;
        geometry_msgs::PointStamped pt;
        Eigen::Vector4f centroid;

        // std::cout << cluster_indices.size() << std::endl;
        // tf::Transform transform;
        // transform.setOrigin( tf::Vector3(1.5, -0.14,-0.38) );
        // tf::Quaternion q;
        // q.setRPY(0, 0, 0);
        // transform.setRotation(q);
        // br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), scan->header.frame_id, "cluster_1_frame"));

        

        for(const auto & index : cluster_indices)
        {
            number_clusters = (int) cluster_indices.size();
            ROS_DEBUG("Number of clusters found: %d",number_clusters);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto & point : index.indices)
            {
                cloud_cluster->points.push_back(cloud->points[point]);
            }
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            // Compute centroid
            pcl::compute3DCentroid(*cloud_cluster, centroid);
            pt.point.x = centroid(0);
            pt.point.y = centroid(1);
            pt.point.z = centroid(2);

            if (pt.point.y < 0.21 | pt.point.y > 0.24){
                continue;
            }

            // Compute standard deviation
            double x_spread = 0;
            double y_spread = 0;
            double z_spread = 0;

            for (const auto point : cloud_cluster->points){
                x_spread += pow(point.x - pt.point.x,2)/cloud_cluster->width;
                y_spread += pow(point.y - pt.point.y,2)/cloud_cluster->width;
                z_spread += pow(point.z - pt.point.z,2)/cloud_cluster->width;
            }

            x_spread = sqrt(x_spread);
            y_spread = sqrt(y_spread);
            z_spread = sqrt(z_spread);

            // if (z_spread > 0.025){
            //     continue;
            // } else if (x_spread + y_spread > 0.2){
            //     continue;
            // }

            // std::cout << pt.point.x << " " << pt.point.y << " " << pt.point.z << std::endl;

            // convert to rosmsg and publish:
            ROS_DEBUG("Publishing extracted cloud");
            pcl::toROSMsg(*cloud_cluster, *ros_cloud);
            ros_cloud->header.frame_id = scan->header.frame_id;
            if(j < 1)
            {
                // cloud_pub[j].publish(ros_cloud);

                // Initialize Moment of Inertia Estimator
                pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
                feature_extractor.setInputCloud (cloud_cluster);
                feature_extractor.compute ();

                // Declare variables for features
                std::vector <float> moment_of_inertia;
                std::vector <float> eccentricity;
                pcl::PointXYZ min_point_AABB;
                pcl::PointXYZ max_point_AABB;
                pcl::PointXYZ min_point_OBB;
                pcl::PointXYZ max_point_OBB;
                pcl::PointXYZ position_OBB;
                Eigen::Matrix3f rotational_matrix_OBB;
                float major_value, middle_value, minor_value;
                Eigen::Vector3f major_vector, middle_vector, minor_vector;
                Eigen::Vector3f mass_center;

                // Extract features
                feature_extractor.getMomentOfInertia (moment_of_inertia);
                feature_extractor.getEccentricity (eccentricity);
                feature_extractor.getAABB (min_point_AABB, max_point_AABB);
                feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
                feature_extractor.getEigenValues (major_value, middle_value, minor_value);
                feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
                feature_extractor.getMassCenter (mass_center);

                // Find OBB
                pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
                pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
                pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
                pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
                // viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
                // viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
                // viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
                

           
                // compute centroid and publish
                // pcl::compute3DCentroid(*cloud_cluster, centroid);
                // pt.point.x = centroid(0);
                // pt.point.y = centroid(1);
                // pt.point.z = centroid(2);
                pt.header.stamp = scan->header.stamp;
                pt.header.frame_id = scan->header.frame_id;

                double zdist = fabs(center.z - x_axis.z);
                double xdist = fabs(center.x - x_axis.x);
                double angle = atan2(zdist,xdist);

                // let's send transforms as well:
                tf::Transform transform;
                transform.setOrigin( tf::Vector3(centroid(0), centroid(1), centroid(2)) );
                tf::Quaternion q;
                q.setRPY(0, angle, 0);
                transform.setRotation(q);
                br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), scan->header.frame_id, "cluster_" + std::to_string(j + 1) + "_frame"));
            }
            j++;
        }
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cluster_extractor");
    ClusterExtractor extractor;
  
    ros::spin();
  
    return 0;
}
