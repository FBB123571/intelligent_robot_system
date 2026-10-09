#include "arLocation.h"

// 构造函数
arLocation::arLocation(ros::Publisher vel_pub_, double target_distance_,
                        double T_Kp_, double T_Kd_, double R_Kp_, double R_Kd_)
{
    // Turtlebot2的速度发布者
    vel_pub = vel_pub_;

    // 目标距离和目标角度
    target_distance = target_distance_;
    target_angle = 0.0;

    // 比例控制参数
    T_Kp = T_Kp_;
    R_Kp = R_Kp_;
    // 微分控制参数
    T_Kd = T_Kd_;
    R_Kd = R_Kd_;

    // 出发点
    start_point = new myPoint(0.0, 0.0, 0.0, ros::Time::now());
    // 上一点
    last_point  = new myPoint(0.0, 0.0, 0.0, ros::Time::now());

    begin_measured = false;  // 出发前是否已经实现一次二维码定位
    move           = false;  // Turtlebot2是否开始移动
    isArrived      = false;  // Turtlebot2是否已经到达终点
}


// 析构函数
arLocation::~arLocation()
{
}


// 发布速度信息
void arLocation::publishVelMsg(double linear_speed, double angular_speed)
{
    geometry_msgs::Twist vel_msg;

    vel_msg.linear.x = linear_speed;
    vel_msg.angular.z = angular_speed;

    vel_pub.publish(vel_msg);
}


// 订阅 /odom 话题的回调函数
void arLocation::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    if (begin_measured && !isArrived)
    {
        // 四元数转换为欧拉角（获取Turtlebot2的朝向角theta = yaw）
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        myPoint* current_piont = new myPoint(
            msg->pose.pose.position.x, msg->pose.pose.position.y, 
            yaw, msg->header.stamp);
        
        double dt = current_piont->time.toSec() - last_point->time.toSec();
        if (dt == 0) return;

        if (!move)
        {
            start_point->copyFrom(current_piont);
            last_point->copyFrom(current_piont);

            ROS_INFO("Going Straight");
            move = true;
        }

        if (isCloseEnough(current_piont))
        { 
            publishVelMsg(0.0, 0.0);
            ROS_INFO("Arrived");
            isArrived = true;
        }


        // 初始化速度变量
        double linear_speed = 0.0;
        double angular_speed = 0.0;

        // 计算前进速度
        linear_speed = calculatePD(
            start_point->getDistance(current_piont), start_point->getDistance(last_point), 
            target_distance, dt, T_Kp, T_Kd);

        // 计算得到的欧拉角范围为-180~180度
        double current_diff = current_piont->angle - start_point->angle;
        if (current_diff < -PI) current_diff += 2*PI;
        else if (current_diff > PI) current_diff -= 2*PI;

        double last_diff = last_point->angle - start_point->angle;
        if (last_diff < -PI) last_diff += 2*PI;
        else if (last_diff > PI) last_diff -= 2*PI;

        angular_speed = calculatePD(current_diff, last_diff, target_angle, dt, R_Kp, R_Kd);


        // 发布速度信息
        publishVelMsg(fmin(MAX_LINEAR_SPEED, linear_speed), fmin(MAX_ANGULAR_SPEED, angular_speed));
        
        // 更新
        last_point->copyFrom(current_piont);
    }
}


// 判断是否到达目标位置或角度
bool arLocation::isCloseEnough(myPoint* current_piont)
{
    double distance = start_point->getDistance(current_piont);
    if (fabs(target_distance - distance) > TOLERANCE_DISTANCE)
    {
        return false;
    }
    if (fabs(target_angle - (current_piont->angle - start_point->angle)) > TOLERANCE_ANGLE &&
        fabs(target_angle - (current_piont->angle - start_point->angle) + 2*PI) > TOLERANCE_ANGLE &&
        fabs(target_angle - (current_piont->angle - start_point->angle) - 2*PI) > TOLERANCE_ANGLE)
    {
        return false;
    }
    return true;
}


// 订阅 /ar_pose_marker 话题的回调函数
void arLocation::markersCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    if (msg->markers.size() > 0)
    {
        if (!begin_measured)
        {
            x_ar0 = msg->markers[0].pose.pose.position.x;
            y_ar0 = msg->markers[0].pose.pose.position.y;
            z_ar0 = msg->markers[0].pose.pose.position.z;
            begin_measured = true;
        }    
        if (isArrived)
        {
            double x_ar1 = msg->markers[0].pose.pose.position.x;
            double y_ar1 = msg->markers[0].pose.pose.position.y;
            double z_ar1 = msg->markers[0].pose.pose.position.z;
            // 计算Turtlebot2的移动距离
            double distance = 0.0;
            ROS_INFO("Measurement: %.2f", distance);
            exit(0);
        }
    }
}


// PD控制器计算速度
double arLocation::calculatePD(double current, double last, double target, double dt, double Kp, double Kd)
{
    double speed = 0.0;
    double error = target - current;
    double preError = target - last;
    double derivative = (error - preError) / dt;
    speed = Kp * error + Kd * derivative;
    return speed;
}
