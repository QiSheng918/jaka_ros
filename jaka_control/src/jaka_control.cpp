#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>


typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> ActionServer;
ros::Publisher trajectory_pub;
void executeTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, ActionServer* as)
{
//   //double rad2deg = 180.0 / 3.141;
// //   robotState rs;
    // std::cout<<"hello world"<<std::endl;
//     std::cout<<goal->trajectory.points.size()<<std::endl;
//     std::cout<<goal->trajectory.points[0].positions[0]<<"<"<<goal->trajectory.points[0]<<std::endl;
// //   float lastDuration = 0.0;

// //   int nrOfPoints =;		// Number of points to add
    ROS_INFO("start moved");
    trajectory_msgs::JointTrajectory msg=goal->trajectory;
    trajectory_pub.publish(msg);
    as->setSucceeded();
}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "rob_mover");
	ros::NodeHandle nh;
    trajectory_pub=nh.advertise<trajectory_msgs::JointTrajectory>("trajectory_command",1000);
	//Start the ActionServer for JointTrajectoryActions and GripperCommandActions from MoveIT
	ActionServer action_server(nh, "arm_controller/follow_joint_trajectory", boost::bind(&executeTrajectory, _1, &action_server), false);
  	ROS_INFO("TrajectoryActionServer: Starting");
  	action_server.start();


    ros::spin();
	// ros::waitForShutdown();
	return(0);
}






