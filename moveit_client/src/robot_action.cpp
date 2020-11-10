//
// Created by aemc on 2020/10/14.
///此代码用于驱动机器人运动
//
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Float32MultiArray.h>
#include <universal_msgs/Command.h>
#define pi 3.1415926
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;
universal_msgs::Command msg;
ros::Publisher robot_move;
std::basic_string<char> Link1;
std::basic_string<char> Link2;
std::basic_string<char> Link3;
std::basic_string<char> Link4;
std::basic_string<char> Link5;
std::basic_string<char> Link6;
void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as)
{
    //ros::Rate rate(1);
//    double tp1;
//    double tp2;
//    double tp3;
//    double tp4;
//    double tp5;
//    double tp6;
    ros::NodeHandle t;
    Link1 = goal->trajectory.joint_names[0];
    Link2 = goal->trajectory.joint_names[1];
    Link3 = goal->trajectory.joint_names[2];
    Link4 = goal->trajectory.joint_names[3];
    Link5 = goal->trajectory.joint_names[4];
    Link6 = goal->trajectory.joint_names[5];
    std::cout<<Link1<<" "<<Link2<<" "<<Link3<<" "<<Link4<<" "<<Link5<<" "<<Link6<<" "<<std::endl;
    std::cout<<"get "<<goal->trajectory.points.size()<<" waypoiunts"<<std::endl;
    t.setParam("robot_in_position",false);
//    for(int i  =0;5*i<goal->trajectory.points.size();i++)
//    {
//        msg.pose.clear();
//        msg.pose.push_back(goal->trajectory.points[5*i].positions[0]*180/pi);
//        msg.pose.push_back(-goal->trajectory.points[5*i].positions[1]*180/pi);
//        msg.pose.push_back(-goal->trajectory.points[5*i].positions[2]*180/pi);
//        msg.pose.push_back(-goal->trajectory.points[5*i].positions[3]*180/pi);
//        msg.pose.push_back(goal->trajectory.points[5*i].positions[4]*180/pi);
//        msg.pose.push_back(-goal->trajectory.points[5*i].positions[5]*180/pi);
//        robot_move.publish(msg);
//        //usleep(8000);
//    }
    msg.pose.clear();
    int num = goal->trajectory.points.size()-1;
    msg.pose.push_back(goal->trajectory.points[num].positions[0]*180/pi);
    msg.pose.push_back(-goal->trajectory.points[num].positions[1]*180/pi);
    msg.pose.push_back(-goal->trajectory.points[num].positions[2]*180/pi);
    msg.pose.push_back(-goal->trajectory.points[num].positions[3]*180/pi);
    msg.pose.push_back(goal->trajectory.points[num].positions[4]*180/pi);
    msg.pose.push_back(-goal->trajectory.points[num].positions[5]*180/pi);
    robot_move.publish(msg);
    bool in_position = false;
    //universal_msgs::Command yse_msg;
    //yse_msg.type = 11;
    //robot_move.publish(yse_msg);
    t.getParam("robot_in_position",in_position);
    while(!in_position){
        //robot_move.publish(yse_msg);
        //usleep(2e5);+
        sleep(1);
        t.getParam("robot_in_position",in_position);
        std::cout<<"the robot is still running"<<std::endl;
    }
    t.setParam("robot_in_position",false);
    ROS_INFO("Recieve action successful!");
    as->setSucceeded();
}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"robot_action");
    ros::NodeHandle nh;

    msg.type =2;
    msg.speed = 35;
    robot_move= nh.advertise<universal_msgs::Command>("command",100);;
    Server server(nh, "jaka_hand/follow_joint_trajectory", boost::bind(&execute, _1, &server), false);
    server.start();
//    ros::AsyncSpinner spinner(1);
//    spinner.start();
    while(ros::ok())
    {
        ros::spinOnce();
    }
}