#ifndef ARLOCATION_H
#define ARLOCATION_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <tf/tf.h>
#include <math.h>
#include "myPoint.h"


#define PI 3.141592

#define MAX_LINEAR_SPEED   0.35  // 最大移动速度
#define MAX_ANGULAR_SPEED  0.45  // 最大旋转速度

#define TOLERANCE_DISTANCE 0.010  // 允许的距离误差
#define TOLERANCE_ANGLE    0.015  // 允许的角度误差


class arLocation
{   
    ros::Publisher vel_pub;  // Turtlebot2的速度发布者

    bool begin_measured;     // 出发前是否已经实现一次二维码定位
    bool move;               // Turtlebot2是否开始移动
    bool isArrived;          // Turtlebot2是否已经到达终点

    double target_distance;  // 目标距离
    double target_angle;     // 目标角度
    myPoint *start_point;    // 出发点
    myPoint *last_point;     // 上一点

    double T_Kp, R_Kp;       // 比例控制参数
    double T_Kd, R_Kd;       // 微分控制参数

    // Turtlebot2在起点时，二维码中心相对于相机坐标系的位置
    double x_ar0, y_ar0, z_ar0;


public:
    // 构造函数
    arLocation(ros::Publisher vel_pub_, double target_distance_,
                    double T_Kp_, double T_Kd_, double R_Kp_, double R_Kd_);

    // 析构函数
    ~arLocation();

    // 发布速度信息
    void publishVelMsg(double linear_speed, double angular_speed);

    // 订阅 /odom 话题的回调函数
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg); 

    // 订阅 /ar_pose_marker 话题的回调函数
    void markersCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);

    // 判断是否到达目标位置或角度
    bool isCloseEnough(myPoint* current_piont);

    // PD控制器计算速度
    double calculatePD(double current, double last, double target, double dt, double Kp, double Kd);
};

#endif
