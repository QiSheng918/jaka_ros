//
// Created by aemc on 2020/10/14.
//

#include "../include/test_movit.h"
test_movit::test_movit()
{
    test_client = nh.serviceClient<moveit_client::plan_and_move>("target");
}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"test_client");

    test_movit ts;
    ts.msg.request.joints ={242.312, 452.898, 633.537, -1.31219, -2.23935, -179.408};
    ts.msg.request.vel = 10;
    while(ros::ok())
    {
        if(ts.test_client.call(ts.msg))
        {
            std::cout<<"finish once"<<std::endl;
            sleep(2000);
        }
    }

}