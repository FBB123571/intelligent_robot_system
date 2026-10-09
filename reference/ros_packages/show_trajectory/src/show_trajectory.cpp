#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher path_pub;
nav_msgs::Path path;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::PoseStamped poseStamped;
    // poseStamped.pose = msg->pose;
    poseStamped.pose.position.x = msg->pose.pose.position.x;
    poseStamped.pose.position.y = msg->pose.pose.position.y;
    poseStamped.pose.position.z = msg->pose.pose.position.z;
    poseStamped.pose.orientation = msg->pose.pose.orientation;
    poseStamped.header.stamp = msg->header.stamp;
    poseStamped.header.frame_id = "odom";

    path.poses.push_back(poseStamped);
    path.header.stamp = msg->header.stamp;
    path.header.frame_id = "odom";

    path_pub.publish(path);
}


int main (int argc, char **argv)
{
    ros::init (argc, argv, "show_trajectory");

    ros::NodeHandle nh;
    path_pub = nh.advertise<nav_msgs::Path>("trajectory", 1, true);

    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCallback);
    ros::spin();

    return 0;
}
