#include "ros/ros.h"
#include "moveit/move_group_interface/move_group_interface.h"

std::vector<double> get_rest_pose(){
    double restPose[6] = {-0.36, -0.86, -1.4, -2.61, 0.22, 1.80};
    std::vector<double> joint_group_rest_pose(6);
    joint_group_rest_pose[0] = restPose[0];
    joint_group_rest_pose[1] = restPose[1];
    joint_group_rest_pose[2] = restPose[2];
    joint_group_rest_pose[3] = restPose[3];
    joint_group_rest_pose[4] = restPose[4];
    joint_group_rest_pose[5] = restPose[5];
    return joint_group_rest_pose;
}

void move_2_rest(moveit::planning_interface::MoveGroupInterface &arm){
    double restPose[6] = {-0.36, -0.86, -1.4, -2.61, 0.22, 1.80};
    std::vector<double> joint_group_rest_pose(6);
    joint_group_rest_pose[0] = restPose[0];
    joint_group_rest_pose[1] = restPose[1];
    joint_group_rest_pose[2] = restPose[2];
    joint_group_rest_pose[3] = restPose[3];
    joint_group_rest_pose[4] = restPose[4];
    joint_group_rest_pose[5] = restPose[5];

    arm.setJointValueTarget(joint_group_rest_pose);
    arm.move();
    sleep(1);
    ROS_INFO("Just back in rest pose");
}





int main(int argc, char * argv[]){
    ros::init(argc,argv,"moveit_fk_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    arm.setGoalJointTolerance(0.001);
    arm.setMaxAccelerationScalingFactor(0.01);
    arm.setMaxVelocityScalingFactor(0.01);


    std::vector<double> joint_current_pose(6);
    joint_current_pose = arm.getCurrentJointValues();
    int count = 0;
    for(auto element : joint_current_pose){
        count++;
        // ROS_INFO("element: %.2f (in degree)",element * 180 / M_PI);
        ROS_INFO("current joint Pos No.%d: %.2f (in degree)",count,element * 180 / M_PI);
    }

    std::vector<double> joint_target_pose(6);
    // joint_target_pose = get_rest_pose();
    
    // count = 0;
    // for(auto element : joint_target_pose){
    //     count++;
    //     // ROS_INFO("element: %.2f (in degree)",element * 180 / M_PI);
    //     ROS_INFO("target joint Pos No.%d: %.2f (in degree)",count,element  * 180 / M_PI);
    // }

    // arm.setJointValueTarget(joint_target_pose);
    // arm.move();
    // sleep(1);
    // ROS_INFO("Just back in rest pose");
    move_2_rest(arm);

    ros::shutdown();

    return 0;
}