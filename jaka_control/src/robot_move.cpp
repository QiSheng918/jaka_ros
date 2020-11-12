#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <boost/thread.hpp>
#include <sstream>
#include <vector>
#include <cstdio>
#include "jaka_control/cubicSpline.h"
#include <universal_msgs/Command.h>
#define JNUMBER 6
#define PI 3.1415926
#define AXIS_COUNT 6

using namespace std;

vector<vector<double> > all_points;
vector<float> pos(JNUMBER,0.0);


void chatterCB(const trajectory_msgs::JointTrajectory& msg)
{
	if((int)msg.points.size() != 0)
	{
		cubicSpline spline;
		double time_nsecs=0.0;
		int points_count=0;
		double joint_positions[6] {0.0};
		double right_bound=0;
		ROS_INFO("get the joint_path_command");
		std::basic_string<char> jointname=msg.joint_names[0] ;
		points_count= msg.points.size();                        //获取规划后点的个数
		double *Time=new double[points_count];              //划分数组
		double *data_arr=new double[points_count];
		for (int j=0; j < AXIS_COUNT; j++)
		{
			for (int i{}; i < points_count; i++)
			{
				data_arr[i]=msg.points[i].positions[j];
				Time[i]=((double)msg.points[i].time_from_start.toSec());
			} 
			spline.loadData(Time,data_arr,points_count,0,0, cubicSpline::BoundType_First_Derivative);
			double x_out = 0;
			double y_out = 0;
			vector<double> single_axis_points;
			double step = 0.008;  //default 0.004
			for(double i=0; i<=(((double)msg.points[points_count-1].time_from_start.toSec())); i=i+step)
			{
				x_out = x_out + step;
				spline.getYbyX(x_out, y_out); //x是时间，y是点
				single_axis_points.push_back(y_out); //get points in single axis
			}
			all_points.push_back(single_axis_points); //pull all point together. rows represent axis
		}

	}
}


double radian_to_angle(double rad)
{
	double result =(rad * 180)/PI;
	return result;
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "joint_states_pub");
	ros::NodeHandle n;
	// 订阅关节路径命令
	ros::Subscriber sub=n.subscribe("joint_path_command",10,chatterCB);
	ros::Publisher pub=n.advertise<universal_msgs::Command>("command",1);
	universal_msgs::Command command_msg;
	// 定义机器人
	int count=0;
	ros::Rate loop_rate(125);
	
	command_msg.type=11;
	command_msg.servo_enable_flag=1;
	pub.publish(command_msg);
	

	int j = 0;
	while (ros::ok())
	{
		std_msgs::String msg;
		//print 6 axis point in same time stamp, should change later. should be sent to the sj interface
		if (all_points.size() == 0)
		{
			//printf("hello\n");
			ros::spinOnce();
			loop_rate.sleep();
		}
		else if (j<all_points[0].size())
		{
			for(int i=0; i<all_points.size(); i++)
			{
				pos[i] = radian_to_angle(all_points[i][j]); //change to angle
				printf("%dpos%d:%f\t", j, i, pos[i]);

			}
			j++;
			//
			//
			//cout<<pos[0]<<endl;
			// robot.jointMove(pos,10);
			command_msg.type=12;
			command_msg.joint=pos;
			pub.publish(command_msg);
			ros::spinOnce();
			loop_rate.sleep();
		}
		else
		{
			all_points.clear();
			j = 0;
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	printf("sleeping!\n");
	sleep(10);
	printf("enable shut down!\n");
	command_msg.type=11;
	command_msg.servo_enable_flag=0;
	pub.publish(command_msg);
	// reply=robot.ServoJEnable(false);
	// std::cout<<"servoj enable "<<reply<<std::endl;
	//  robot.stop();
	return 0;
}