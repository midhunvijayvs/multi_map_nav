#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <multi_map_nav/WormholeJumpAction.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigate_wormhole_client");

    actionlib::SimpleActionClient<multi_map_nav::WormholeJumpAction> ac("navigate_wormhole", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    multi_map_nav::WormholeJumpGoal goal;
    goal.source_map = "room1";
    goal.target_map = "room2";

    ROS_INFO("Waiting 2 seconds before sending goal...");
    ros::Duration(2.0).sleep();  // Delay to let action server & TF system initialize
    ROS_INFO("Sending goal.");
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Navigation succeeded: %s", ac.getResult()->message.c_str());
    else
        ROS_INFO("Navigation failed");

    return 0;
}

