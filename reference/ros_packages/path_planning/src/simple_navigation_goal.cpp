#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#define PI 3.1415926

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>  MoveBaseClient;


int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_navigation_goal");

    // 目标点
    double x_d, y_d, theta_d;

    // 创建action客户端
    MoveBaseClient ac("move_base", true);  // true：自动调用ros::spin()

    // 等待action服务器
    while(!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server");
    }
    ROS_INFO("Connected to move base server");

    // 设置路径规划目的地
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";  // 目的地相对的坐标系
    goal.target_pose.header.stamp = ros::Time::now();

    // 设置目标点
    x_d     = 3.0;
    y_d     = -0.6;
    theta_d = -20.0;
    goal.target_pose.pose.position.x = x_d;
    goal.target_pose.pose.position.y = y_d;
    goal.target_pose.pose.orientation.z = sin(theta_d / 2 * PI / 180);
    goal.target_pose.pose.orientation.w = cos(theta_d / 2 * PI / 180);

    // 发送目标点
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);   

    // 等待自主导航结果
    ac.waitForResult();  
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("You have reached the goal!");
    else
        ROS_INFO("The base failed for some reason");

    // ...

    return 0;
}
