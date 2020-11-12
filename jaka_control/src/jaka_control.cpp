#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <universal_msgs/Command.h>
#include "jaka_control/cubicSpline.h"

#define JNUMBER 6
#define PI 3.1415926
const double rad2deg=180/3.1415926;

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> ActionServer;
ros::Publisher command_pub;
ros::Publisher joint_state_pub;

void executeTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, ActionServer* as)
{
    ROS_INFO("get the moveit planning result");
    trajectory_msgs::JointTrajectory msg=goal->trajectory;
	if((int)msg.points.size() != 0)
	{
		cubicSpline spline;

		int points_count= msg.points.size();                        //获取规划后点的个数
		double *Time=new double[points_count];              //划分数组
		double *data_arr=new double[points_count];

		std::vector<std::vector<double> > all_points;

		for (int j=0; j < JNUMBER; j++)
		{
			for (int i=0; i < points_count; i++)
			{
				data_arr[i]=msg.points[i].positions[j];
				Time[i]=((double)msg.points[i].time_from_start.toSec());
			} 
			spline.loadData(Time,data_arr,points_count,0,0, cubicSpline::BoundType_First_Derivative);
			double x_out = 0;
			double y_out = 0;
			std::vector<double> single_axis_points;
			double step = 0.008;  //default 0.004
			for(double i=0; i<=(((double)msg.points[points_count-1].time_from_start.toSec())); i=i+step)
			{
				x_out = x_out + step;
				spline.getYbyX(x_out, y_out); //x是时间，y是点
				single_axis_points.push_back(y_out); //get points in single axis
			}

			all_points.push_back(single_axis_points); //pull all point together. rows represent axis
		}


		ros::Rate loop_rate(125);
		sensor_msgs::JointState joint_state_msg;
		joint_state_msg.name.resize(6);
		joint_state_msg.position.resize(6);
		for(int i=0;i<6;i++){
			joint_state_msg.name[i]="joint_"+std::to_string(i+1);
			joint_state_msg.position[i]=0;
		}
		universal_msgs::Command command_msg;
		command_msg.type=11;
		command_msg.servo_enable_flag=1;
		command_pub.publish(command_msg);
		// ros::Duration(0.5).sleep(); 
        loop_rate.sleep();
		command_msg.type=12;
		command_msg.joint.resize(6);
		for(int i=0; i<6; i++) command_msg.joint[i]=0;
		std::cout<<all_points[0].size()<<std::endl;
		for(int j=0;j<all_points[0].size();j++){
			for(int i=0; i<6; i++){
				command_msg.joint[i]=all_points[i][j]*rad2deg;
				joint_state_msg.position[i]=all_points[i][j];
			}
			joint_state_pub.publish(joint_state_msg);
			command_pub.publish(command_msg);
			loop_rate.sleep();
		}
		loop_rate.sleep();
		// ros::Duration(0.5).sleep(); 
		command_msg.type=11;
		command_msg.servo_enable_flag=0;
		command_pub.publish(command_msg);
	}

    as->setSucceeded();
}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "jaka_control_node");
	ros::NodeHandle nh;

	command_pub=nh.advertise<universal_msgs::Command>("command",1);
	joint_state_pub=nh.advertise<sensor_msgs::JointState>("my_joint_states",1);
	//Start the ActionServer for JointTrajectoryActions and GripperCommandActions from MoveIT
	ActionServer action_server(nh, "arm_controller/follow_joint_trajectory", boost::bind(&executeTrajectory, _1, &action_server), false);
  	ROS_INFO("TrajectoryActionServer: Starting");
  	action_server.start();
    ros::spin();
	return(0);
}






