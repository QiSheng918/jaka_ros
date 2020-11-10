//
// Created by aemc on 2020/10/14.
//

#ifndef SRC_TEST_MOVIT_H
#define SRC_TEST_MOVIT_H

#include <ros/ros.h>
#include <moveit_client/plan_and_move.h>
#include <universal_msgs/Command.h>
class test_movit {
public:
    test_movit();
    ros::ServiceClient test_client;
    ros::NodeHandle nh;

    moveit_client::plan_and_move msg;

};


#endif //SRC_TEST_MOVIT_H
