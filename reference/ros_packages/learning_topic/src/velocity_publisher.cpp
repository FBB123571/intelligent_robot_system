#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc,char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "velocity_publisher");
    
    // 创建节点句柄 
    ros::NodeHandle nh;
    
    // 创建一个Publisher，发布名为/mobile_base/commands/velocity的topic
    // 消息类型为geometry_msgs::Twist，队列长度10 
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>(
        "/mobile_base/commands/velocity", 10);
    
    // 设置循环的频率 
    ros::Rate loop_rate(10); 
    
    while (ros::ok())
    {
        // 初始化geometry_msgs::Twist类型的消息 
        geometry_msgs::Twist vel_msg; 
        vel_msg.linear.x  = 0.5; 
        vel_msg.linear.y  = 0.0; 
        vel_msg.angular.z = 0.2;
        
        // 发布消息
        vel_pub.publish(vel_msg);
        ROS_INFO("Publish Turtlebot2 velocity command[%0.2f m/s, %0.2f rad/s]", 
            vel_msg.linear.x, vel_msg.angular.z);
        
        // 按照循环频率延时 
        loop_rate.sleep();
    }
        
    return 0;
}

