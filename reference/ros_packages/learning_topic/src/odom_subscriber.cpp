#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

// 接收到订阅的消息后，会进入消息回调函数
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // 获取Turtlebot2的位置(x, y)
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    // 四元数转换为欧拉角（获取Turtlebot2的朝向角theta = yaw）
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // 将接收到的消息打印出来
    ROS_INFO("Turtlebot2's odom: [x:%.2f, y:%.2f, theta:%.2f]", x, y, yaw);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "odom_subscriber");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 创建一个Subscriber，订阅名为/odom的topic，注册回调函数odomCallback
    ros::Subscriber pose_sub = nh.subscribe("/odom", 10, odomCallback);

    // 循环等待回调函数
    ros::spin();

    return 0;
}
