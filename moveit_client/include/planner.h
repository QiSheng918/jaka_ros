//
// Created by aemc on 2020/10/14.
//

#ifndef SRC_PLANNER_H
#define SRC_PLANNER_H
//ros
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

//message
#include <moveit_client/plan_and_move.h>
#include <universal_msgs/Command.h>
#include <universal_msgs/RobotMsg.h>
#include <sensor_msgs/JointState.h>

#define pi 3.1415926

static const std::string PLANNING_GROUP = "arm";///这里指的是针对机器人对应group name进行分析
static const std::string base_link = "base_link";///这里指的是可视化中的基坐标系

class planner {
public:
    planner(ros::NodeHandle &nh);
    ros::AsyncSpinner spinner;
    //moveit
    moveit::planning_interface::MoveGroupInterface move_group;///moveit的主要类
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;///用于在分析场景中添加障碍物进行避障
    const robot_state::JointModelGroup* joint_model_group;
    moveit_visual_tools::MoveItVisualTools visual_tools;///用于在rviz中体现机器人，可视化轨迹等

    ros::ServiceServer service;///接收机器人目标位姿信号，更新虚拟机器人位姿，规划机器人轨迹，指示机器人进行运动，最后反馈成功信号
    bool target_pose_service_callback(moveit_client::plan_and_move::Request &req,moveit_client::plan_and_move::Response &res);

    ros::Subscriber robot_pos_sub;///实时获取机器人的末端工具位姿
    void robot_pos_callback(const universal_msgs::RobotMsg &robot_pos);

//    ros::Subscriber target_pose_sub;
//    void target_pose_callback(const universal_msgs::Command &robot_msg);

    std::vector<float> target_pose;///所接收到的机器人目标位姿信号，6维，xyz，rpy
    int target_vel;///所接收到的机器人目标速度
    std::vector<double> current_pose;///当前的机器人位姿，xyz，四元数
    std::vector<double> current_joints;///当前机器人关节角度
    ros::Publisher joints_state_publisher;///发布当前机器人的姿态给虚拟机器人

    ///move_it
    sensor_msgs::JointState jointstate_msg;///用于发布给虚拟机器人的信息
    geometry_msgs::Pose current_pose_msg;//储存虚拟机器人的当前状态，也可以认为是作为规划的起点位姿状态
    geometry_msgs::Pose target_pose_msg;//储存机器人目标位姿
    //角度转换
    Eigen::Quaterniond RPY_2_Q(double roll,double pitch,double yall);

    ///障碍物
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    moveit_msgs::CollisionObject collision_object;
};


#endif //SRC_PLANNER_H
