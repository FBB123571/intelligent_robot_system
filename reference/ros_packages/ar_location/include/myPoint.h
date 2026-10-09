#ifndef MYPOINT_H
#define MYPOINT_H

#include <ros/ros.h>
#include <math.h> 

class myPoint
{
public:
    // 变量
    double x;        // Turtlebot的x轴坐标
    double y;        // Turtlebot的y轴坐标
    double angle;    // Turtlebot的旋转角度
    ros::Time time;  // 里程计测量的时间戳

    // 构造函数
    myPoint(double xPos, double yPos, double alpha, ros::Time t);

    // 析构函数
    ~myPoint();

    // 获取两点之间的角度
    double getAngle(myPoint* target);

    // 获取两点之间的距离
    double getDistance(myPoint* target);

    void copyFrom(myPoint* target);
};

#endif

