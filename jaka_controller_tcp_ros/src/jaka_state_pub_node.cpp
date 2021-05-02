#include <string>
#include <vector>
#include <memory>
#include <unistd.h>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <json/json.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "universal_msgs/RobotMsg.h"

#define MAXLINE 8192
const std::string address="192.168.51.188";
const double deg2rad=3.1415926/180;

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "jaka_state_node");
	ros::NodeHandle nh;
	std::string robot_ip;

	robot_ip=nh.param("robot_ip",address);

    ros::Publisher jaka_joint_state_pub=nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher jaka_pose_pub=nh.advertise<universal_msgs::RobotMsg>("jaka_pose", 1);

    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.name.resize(6);
    joint_state_msg.position.resize(6);
    for(int i=0;i<6;i++){
        joint_state_msg.name[i]="joint"+std::to_string(i+1);
        joint_state_msg.position[0]=0;
    }

	//建立socket通讯
    int socketrqt;
    struct sockaddr_in servaddr_rqt;
	std::cout << "state_pub_node connecting to IP address : " << robot_ip << std::endl;
	const char *address_ptr = robot_ip.c_str();
    // 创建socketrqt
	if ((socketrqt = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		std::cout << "create socket error:" << strerror(errno) << "(errno:" << errno << ")" << std::endl;
		exit(0);
	}
	memset(&servaddr_rqt, 0, sizeof(servaddr_rqt));
    // 指定IP地址版本为IPV4
	servaddr_rqt.sin_family = AF_INET;
    // 设置端口10001
	servaddr_rqt.sin_port = htons(10000);
    // IP地址转换函数
	if (inet_pton(AF_INET, address_ptr, &servaddr_rqt.sin_addr) <= 0) {
		std::cout << "inet_pton error for " << address_ptr;
		exit(-1);
	}

	if (connect(socketrqt, (struct sockaddr *) &servaddr_rqt, sizeof(servaddr_rqt)) < 0) {
		std::cout << "connect error:" << strerror(errno) << "(errno:" << errno << ")" << std::endl;
		exit(-1);
	}
	std::cout << "state_pub_node socket connects successfully!" << std::endl;

	
    Json::Reader reader;
    Json::Value root;
    while(ros::ok()){
        char buf[MAXLINE];
        int rec_len = recv(socketrqt, buf, MAXLINE, 0);
	    buf[rec_len] = '\0';
        // std::cout<<rec_len<<std::endl;
        // std::cout<<buf<<std::endl;
		
		//解析收到的字节流获取机器人状态信息
		std::vector<double> q_actual(6,0.0);
		std::vector<double> tool_vector(6,0.0);
		try{
			if (reader.parse(buf, root)) {
			int joint_size = root["joint_actual_position"].size();
			int cart_size = root["actual_position"].size();
			if(joint_size==6 && cart_size==6){
				for (Json::Value::ArrayIndex i = 0; i < joint_size; ++i) {
						q_actual[(int) i] = root["joint_actual_position"][i].asDouble();
				}
				for (Json::Value::ArrayIndex i = 0; i < cart_size; ++i) {
						tool_vector[(int) i] = root["actual_position"][i].asDouble();
				}
			
				joint_state_msg.header.stamp=ros::Time::now();
				for(int i=0;i<6;i++) joint_state_msg.position[i]=q_actual[i]*deg2rad;
				jaka_joint_state_pub.publish(joint_state_msg);

				universal_msgs::RobotMsg msg;
				msg.header.stamp = ros::Time::now();
				msg.data.q_actual=q_actual;
				msg.data.tool_vector=tool_vector;
				jaka_pose_pub.publish(msg);

				std::cout << "Joint: [ " << q_actual[0] << ", " << q_actual[1] << ", " << q_actual[2]
						<< ", " << q_actual[3] << ", " << q_actual[4] << ", " << q_actual[5] << " ]" << std::endl;
				std::cout << "Cart: [ " << tool_vector[0] << ", " << tool_vector[1] << ", " << tool_vector[2]
						<< ", " << tool_vector[3] << ", " << tool_vector[4] << ", " << tool_vector[5] << " ]" << std::endl;
				} 
			}   
		}
		catch(...){}
    }

	//关闭socket连接
    close(socketrqt);
	std::cout<<"Robot disconnected!"<<std::endl;
    return 0;
}
