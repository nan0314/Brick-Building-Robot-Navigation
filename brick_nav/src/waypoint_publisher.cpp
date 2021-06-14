#include <string>
#include <vector>

#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cmath>
#include <time.h>
// #include <tf>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Empty.h>

static ros::Publisher pub_wp;
static bool publish = false;
static geometry_msgs::PoseWithCovarianceStamped waypoint;


void home_callback(const std_msgs::Empty msg) {

    waypoint.header.stamp = ros::Time::now();

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, 0);

    waypoint.pose.pose.orientation.x = quaternion[0];
    waypoint.pose.pose.orientation.y = quaternion[1];
    waypoint.pose.pose.orientation.z = quaternion[2];
    waypoint.pose.pose.orientation.w = quaternion[3];

    waypoint.pose.pose.position.x = 0;
    waypoint.pose.pose.position.y = 0;

    publish = true;

}

void target_callback(const std_msgs::Empty msg){

    // Get brick location from tf tree
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;

    while (ros::ok()){
        
        try{
            transformStamped = tfBuffer.lookupTransform("map", "target",ros::Time(0));
            break;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }

    // Set up waypoint
    waypoint.header.stamp = ros::Time::now();

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, M_PI_2);

    waypoint.pose.pose.orientation.x = quaternion[0];
    waypoint.pose.pose.orientation.y = quaternion[1];
    waypoint.pose.pose.orientation.z = quaternion[2];
    waypoint.pose.pose.orientation.w = quaternion[3];

    waypoint.pose.pose.position.x = transformStamped.transform.translation.x;
    waypoint.pose.pose.position.y = transformStamped.transform.translation.y;

    publish = true;

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "waypoint_publisher");
    ros::NodeHandle nh;

    // Set up publishers and subscribers

    pub_wp = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/my_jackal_waypoints", 1);
    ros::Publisher pub_init_wp = nh.advertise<std_msgs::Empty>("/path_ready", 1);

    ros::Subscriber home_sub = nh.subscribe<std_msgs::Empty>(
        "/home_ready", 1, home_callback);
    ros::Subscriber target_sub = nh.subscribe<std_msgs::Empty>(
        "/target_ready", 1, target_callback);

    waypoint.header.frame_id = "/map";

    // Establish loop rate
    ros::Rate r(10);

    while (ros::ok()){
        
        if (publish){

            // Publish waypoint
            while (ros::ok()){

                int connections = pub_wp.getNumSubscribers();
                if (connections > 0){
                    pub_wp.publish(waypoint);
                    break;
                }
                std::cout << "Waiting for /my_jackal_waypoints topic" << std::endl;
            }

            std::cout << "Published waypoint" << std::endl;

            // Publish Waypoint follower start command
            std_msgs::Empty start_command;
    
            while (ros::ok()){
                int connections = pub_init_wp.getNumSubscribers();
                if (connections > 0){
                    pub_init_wp.publish(start_command);
                    std::cout << "Sent waypoint execution command" << std::endl;
                    break;
                }

                std::cout << "Waiting for /path_ready topic" << std::endl;
            }

            publish = false;
        }

        r.sleep();
        ros::spinOnce();
    }

    return 0;
}