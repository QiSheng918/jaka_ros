//C++
#include <iostream>
#include <vector>
#include <boost/thread.hpp>

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>

//gRPC related
#include "robot_client_tcp.h"

//MSG
#include "universal_msgs/Command.h"
#include "universal_msgs/RobotMsg.h"

std::string address= "192.168.51.188";
std::string command_Topic_Name = "command";
std::string robotMsg_Topic_Name = "jaka_pose";
const double deg2rad=3.1415926/180;

class jaka_ros_node
{
public:
	jaka_ros_node()
	{
		robot_ip=nh.param("robot_ip",address);
		robotclient.SetIP(robot_ip);

		robotclient.PowerOn();
		robotclient.Enable();
		cmd_sub=nh.subscribe(command_Topic_Name, 1, &jaka_ros_node::CommandCallback,this);
	}
	
private:
	ros::NodeHandle nh;
	ros::Subscriber cmd_sub;
	std::string robot_ip;
	RobotClient robotclient;
	void CommandCallback(const universal_msgs::Command::ConstPtr &msg);
};


void jaka_ros_node::CommandCallback(const universal_msgs::Command::ConstPtr &msg)
{
	if (msg->type == 0)
	{
		std::cout<<"empty Command!"<<std::endl;
		return ;
	}else if (msg->type == 1)
	{
		std::cout<<"Move Line Pose!"<<std::endl;
		return ;
	}else if (msg->type == 2)
	{
		std::cout<<"Move Joint Pose!"<<std::endl;
		robotclient.MoveE(msg->pose, msg->speed);
		return ;
	}else if (msg->type == 3)
	{
		std::cout<<"DIO Command!"<<std::endl;
		// robotclient.DIO(msg->io_type, msg->io_id, msg->dio_value);
		return ;
	}else if (msg->type == 4)
	{
		std::cout<<"Stop Command!"<<std::endl;
		// robotclient.MtnAbort();
		return ;
	}else if (msg->type == 5)
	{
		std::cout<<"Move Tool Command!"<<std::endl;
		robotclient.MoveE(msg->pose, msg->speed);
		return ;
	}else if (msg->type == 6)
	{
		std::cout<<"Move Joint-Multiple Command!"<<std::endl;
		return ;
	}else if (msg->type == 7)
	{
		std::cout<<"Move Line-Multiple Command!"<<std::endl;
		return ;
	}else if (msg->type == 8)
	{
		std::cout<<"JOG Mode!"<<std::endl;
		robotclient.Jog(msg->jogmode, msg->jogcoord, msg->jogaxis,msg->speed,msg->increase);
		return ;
	}else if (msg->type == 9)
	{
		std::cout<<"Move Joint Joint-Angle Command!"<<std::endl;
		std::vector<float> jnt(6,0.0);
		sleep(0.01);
		robotclient.MoveJ(msg->joint, msg->speed);
		return ;
	}else if (msg->type == 10)
	{
		std::cout<<"AIO Command!"<<std::endl;
		robotclient.SetAout(msg->io_type, msg->io_id, msg->aio_value);
		return ;
	}else if(msg->type == 11){
		std::cout<<"ServoJ Enable Command!"<<std::endl;
		robotclient.ServoJEnable();
		return ;
	}else if(msg->type == 12){
		std::cout<<"ServoJ Disable Command!"<<std::endl;
		robotclient.ServoJDisable();
		return ;
	}else if(msg->type == 13){
		std::cout<<"Servo_J Command!"<<std::endl;
		robotclient.ServoJ(msg->joint);
		return ;
	}else{
		std::cout<<"Unknown Command!"<<std::endl;
		return ;
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "jaka_ros_node");
	jaka_ros_node node;
	ros::spin();
	return 0;
}