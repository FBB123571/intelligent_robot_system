#include "myPoint.h"

// 构造函数
myPoint::myPoint(double xPos, double yPos, double alpha, ros::Time t)
{
    x = xPos;
    y = yPos;
    angle = alpha;
    time = t;
}


// 析构函数
myPoint::~myPoint()
{
}


// 获取两点之间的角度
double myPoint::getAngle(myPoint* target)
{
    return atan2((target->y - y), (target->x - x));
}


// 获取两点之间的距离
double myPoint::getDistance(myPoint* target)
{
    return sqrt((pow(target->y - y, 2.0)) + (pow(target->x - x, 2.0)));
}


void myPoint::copyFrom(myPoint* target) 
{
    x = target->x;
    y = target->y;
    angle = target->angle;
    time = target->time;
}
