//
// Created by aemc on 2020/10/14.
//

#include "../include/planner.h"
namespace rvt = rviz_visual_tools;
Eigen::Quaterniond planner::RPY_2_Q(double roll, double pitch, double yall)
{
    Eigen::Vector3d eulerAngle(yall,pitch,roll);
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitZ()));

    Eigen::Quaterniond quaternion;
    quaternion=yawAngle*pitchAngle*rollAngle;
    return quaternion;
}
bool planner::target_pose_service_callback(moveit_client::plan_and_move::Request &req, moveit_client::plan_and_move::Response &res)
{
    spinner.start();
    this->target_pose.clear();
    for(int i=0;i<req.joints.size();i++)
        target_pose.push_back(req.joints[i]);
    this->target_vel = req.vel;
    Eigen::Quaterniond quat = RPY_2_Q(target_pose[3]*pi/180,target_pose[4]*pi/180,target_pose[5]*pi/180);
    Eigen::Matrix3d matrix_temp;
    matrix_temp<<
               1,0,0,
            0,cos(pi), -sin(pi),
            0,       sin(pi), cos(pi);
    quat = Eigen::Quaterniond(quat.toRotationMatrix() * matrix_temp);
    ///读取当前的机器人状态，并更新move_group中的机器人位姿
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    current_state->setJointGroupPositions(joint_model_group,this->current_joints);
    move_group.setStartState(*current_state);
    //current_state.setFromIK(joint_model_group, current_pose_msg);
    ///接下来是针对机器人进行位姿规划
    target_pose_msg.orientation.x = quat.x();target_pose_msg.orientation.y = quat.y();target_pose_msg.orientation.z = quat.z();target_pose_msg.orientation.w = quat.w();
    target_pose_msg.position.x = target_pose[0]/1000;target_pose_msg.position.y = target_pose[1]/1000;target_pose_msg.position.z = target_pose[2]/1000;
    move_group.setPoseTarget(target_pose_msg);
/////位姿约束
//    moveit_msgs::OrientationConstraint ocm;
//    Eigen::Quaterniond qu_temp(0.707,-0.707,0,0);
//    qu_temp = qu_temp*Eigen::Quaterniond(0,0,0,-1);
//    ocm.link_name = "link_6";
//    ocm.header.frame_id = "base_link";
//    ocm.orientation.w = current_pose[6];ocm.orientation.x = current_pose[3];ocm.orientation.y = current_pose[4];ocm.orientation.z = current_pose[5];
//    ocm.absolute_x_axis_tolerance =100;
//    ocm.absolute_y_axis_tolerance = 100;
//    ocm.absolute_z_axis_tolerance = 100;
//    ocm.weight = 1.0;
//    moveit_msgs::Constraints test_constraints;
//    test_constraints.orientation_constraints.push_back(ocm);
//    move_group.setPathConstraints(test_constraints);
//    move_group.setPlanningTime(10.0);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group.move();
//
//    // Visualizing plans
//    // ^^^^^^^^^^^^^^^^^
//    // We can also visualize the plan as a line with markers in RViz.
//    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
//    visual_tools.publishAxisLabeled(target_pose_msg, "pose1");
//    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
//    text_pose.translation().z() = 1.75;
//    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//    //visual_tools.trigger();
//    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    ///以及执行
    res.is_finish =1;
    spinner.stop();
    return true;


}


planner::planner(ros::NodeHandle &nh):
spinner(1),
move_group(PLANNING_GROUP),
visual_tools(base_link)
{

    //this->target_pose_sub = nh.subscribe("target_pose_command",1,&planner::target_pose_callback,this);
    this->service = nh.advertiseService("target",&planner::target_pose_service_callback,this);
    this->robot_pos_sub = nh.subscribe("/jaka_pose",1,&planner::robot_pos_callback, this);
    this->joints_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states",1);
    jointstate_msg.name.resize(8);
    jointstate_msg.position.resize(8);
    spinner.start();
    this->joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    this->visual_tools.deleteAllMarkers();
    this->visual_tools.loadRemoteControl();
    ///要做避障的东西放在这里
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "box1";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.32;
    primitive.dimensions[1] = 0.27;
    primitive.dimensions[2] = 1;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.47;
    box_pose.position.y = -0.21;
    box_pose.position.z = -0.5;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 1;
    primitive.dimensions[2] = 2;

    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.7;
    box_pose.position.y = 0;
    box_pose.position.z = 0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

//    primitive.type = primitive.BOX;
//    primitive.dimensions.resize(3);
//    primitive.dimensions[0] = 0.1;
//    primitive.dimensions[1] = 0.1;
//    primitive.dimensions[2] = 2;
//
//    box_pose.orientation.w = 1.0;
//    box_pose.position.x = 0.6;
//    box_pose.position.y = 0;
//    box_pose.position.z = 0;
//
//    collision_object.primitives.push_back(primitive);
//    collision_object.primitive_poses.push_back(box_pose);
//    collision_object.operation = collision_object.ADD;

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1;
    primitive.dimensions[1] = 1;
    primitive.dimensions[2] = 0.01;

    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0;
    box_pose.position.y = 0;
    box_pose.position.z = 0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);
    planning_scene_interface.addCollisionObjects(collision_objects);
    ///

    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    //visual_tools.trigger();
    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    spinner.stop();
}


void planner::robot_pos_callback(const universal_msgs::RobotMsg &robot_pos)
{
    current_pose.clear();
    current_joints.clear();
    Eigen::Quaterniond quaternion;
    quaternion=this->RPY_2_Q(robot_pos.data.tool_vector[3]/180*pi,robot_pos.data.tool_vector[4]/180*pi,robot_pos.data.tool_vector[5]/180*pi);
    Eigen::Matrix3d matrix_temp;
    matrix_temp<<
    1,0,0,
    0,cos(pi), -sin(pi),
    0,       sin(pi), cos(pi);
    quaternion = Eigen::Quaterniond(quaternion.toRotationMatrix() * matrix_temp);///因为实际机器人的坐标系跟模型有区别，需要对末端坐标系进行修正
    current_pose.push_back(robot_pos.data.tool_vector[0]/1000);
    current_pose.push_back(robot_pos.data.tool_vector[1]/1000);
    current_pose.push_back(robot_pos.data.tool_vector[2]/1000);
    current_pose.push_back( quaternion.x());
    current_pose.push_back( quaternion.y());
    current_pose.push_back( quaternion.z());
    current_pose.push_back(quaternion.w());

    current_joints.push_back(robot_pos.data.q_actual[0]/180*pi);///因为我实际要控制的机器人关节角度与moveit模型中的关节角度正好相反，因此要做一下转换
    current_joints.push_back(-robot_pos.data.q_actual[1]/180*pi);
    current_joints.push_back(-robot_pos.data.q_actual[2]/180*pi);
    current_joints.push_back(-robot_pos.data.q_actual[3]/180*pi);
    current_joints.push_back(robot_pos.data.q_actual[4]/180*pi);
    current_joints.push_back(-robot_pos.data.q_actual[5]/180*pi);
   // std::cout<<"get the robot msg"<<std::endl;
    jointstate_msg.header.stamp = ros::Time::now();
    jointstate_msg.name[0]="joint_1";
    jointstate_msg.position[0] = current_joints[0];
    jointstate_msg.name[1] ="joint_2";
    jointstate_msg.position[1] = current_joints[1];
    jointstate_msg.name[2] ="joint_3";
    jointstate_msg.position[2] = current_joints[2];
    jointstate_msg.name[3] ="joint_4";
    jointstate_msg.position[3] = current_joints[3];
    jointstate_msg.name[4] ="joint_5";
    jointstate_msg.position[4] = current_joints[4];
    jointstate_msg.name[5] ="joint_6";
    jointstate_msg.position[5] = current_joints[5];

    jointstate_msg.name[6] ="jaka_finger_joint1";
    jointstate_msg.position[6] = 0.0;
    jointstate_msg.name[7] ="jaka_finger_joint2";
    jointstate_msg.position[7] = 0.0;
    joints_state_publisher.publish(jointstate_msg);
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"planner");
    ros::NodeHandle nh;
    planner plan_move(nh);
    ros::MultiThreadedSpinner www(4);
    www.spin();
//    while(ros::ok())
//    {
//        ros::spinOnce();
//    }
//    plan_move.spinner.start();
    //ros::waitForShutdown();
    return 0;
}