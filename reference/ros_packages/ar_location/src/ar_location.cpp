#include "arLocation.h"

#define PUBLISHER_BUFFER_SIZE 10
#define SUBSCRIBER_BUFFER_SIZE 1

int main(int argc, char** argv)
{
    double target_distance;  // 目标距离
    double T_Kp, R_Kp;       // 比例控制参数
    double T_Kd, R_Kd;       // 微分控制参数

    // ROS节点初始化
    ros::init(argc, argv, "ar_location");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 获取参数（与ar_location.launch文件中对应）
    nh.getParam("/target_distance", target_distance);
    nh.getParam("/t_kp", T_Kp);
    nh.getParam("/t_kd", T_Kd);
    nh.getParam("/r_kp", R_Kp);
    nh.getParam("/r_kd", R_Kd);

    // 创建一个Publisher，主题名为 /mobile_base/commands/velocity
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", PUBLISHER_BUFFER_SIZE);

    arLocation *al = new arLocation(vel_pub, target_distance, T_Kp, T_Kd, R_Kp, R_Kd);

    ros::Subscriber odom_sub = nh.subscribe(
        "/odom", SUBSCRIBER_BUFFER_SIZE, &arLocation::odomCallback, al);
    ros::Subscriber pose_sub = nh.subscribe(
        "/ar_pose_marker", SUBSCRIBER_BUFFER_SIZE, &arLocation::markersCallback, al);

    ros::spin();

    return 0;
}
