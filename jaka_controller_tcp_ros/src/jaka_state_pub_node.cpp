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

#define MAXLINE 8192
const std::string address="192.168.51.188";
const double deg2rad=3.1415926/180;


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "jaka_state_node");
	ros::NodeHandle nh;
    ros::Publisher joint_state_pub=nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    sensor_msgs::JointState joint_state_msg;

    joint_state_msg.name.resize(6);
    joint_state_msg.position.resize(6);

    for(int i=0;i<6;i++){
        joint_state_msg.name[i]="joint_"+std::to_string(i+1);
        joint_state_msg.position[0]=0;
    }

    Json::Reader reader;
    Json::Value root;
    int socketrqt;
    struct sockaddr_in servaddr_rqt;

    const char *cmd_ptr;

	std::cout << "construction Function of Class RobotClient!" << std::endl;
	std::cout << "Connecting to IP address : " << address << std::endl;
	const char *address_ptr = address.c_str();
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

	std::cout << "Socket connects successfully!" << std::endl;
    int flag=0;
    ros::Rate loop_rate(125);
    while(ros::ok()){
        char buf[MAXLINE];
        int rec_len = recv(socketrqt, buf, MAXLINE, 0);
	    buf[rec_len] = '\0';
	    // std::cout << "Reveived: " << buf << std::endl;
	    float Arr[6], Arr2[6];
        int joint_size ;
        int cart_size;
		joint_size = 0;
		cart_size = 0;
        try{
            // if (reader.parse(buf, root)) {
            reader.parse(buf, root);
            ROS_INFO("parse success");
			joint_size = root["joint_actual_position"].size();
			cart_size = root["actual_position"].size();
            ROS_INFO("parse success1");
            // 解析成功，读取到的数据正确：退出循环
			if (joint_size == 6 && cart_size == 6) {
				for (Json::Value::ArrayIndex i = 0; i < joint_size; ++i) {
					Arr[(int) i] = root["joint_actual_position"][i].asFloat();
				}
				for (Json::Value::ArrayIndex i = 0; i < cart_size; ++i) {
					Arr2[(int) i] = root["actual_position"][i].asFloat();
				}
				// break;
			}
            joint_state_msg.header.stamp=ros::Time::now();
	        for(int i=0;i<6;i++) joint_state_msg.position[i]=Arr[i]*deg2rad;
	        joint_state_pub.publish(joint_state_msg);
        }
        catch(...){
            ROS_ERROR("ERROR");
        }
		    // }
        ROS_INFO("hello world");
        // loop_rate.sleep();
        usleep(8000);
        // flag++;
        // if(flag==2) break;
    }
    close(socketrqt);
	std::cout<<"Class RobotClient Unconstructed!"<<std::endl;
    return 0;
}
