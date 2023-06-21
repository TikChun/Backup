#include "ros/ros.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"
#include "moveit_msgs/OrientationConstraint.h"

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

int main(int argc, char *argv[]){
    ros::init(argc,argv,"moveit_join_traj_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    
    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    arm.setGoalJointTolerance(0.01);
    arm.setGoalOrientationTolerance(0.01);
    arm.setGoalPositionTolerance(0.01);

    arm.setMaxAccelerationScalingFactor(0.1);
    arm.setMaxVelocityScalingFactor(0.1);

    // move_2_rest(arm);
    geometry_msgs::Pose startPose;
    startPose = arm.getCurrentPose().pose;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(startPose); 
    
    geometry_msgs::Pose nextPose;
    nextPose = startPose;
    nextPose.position.x += 0.5;
    waypoints.push_back(nextPose);
    nextPose.position.y += 0.5;
    waypoints.push_back(nextPose);
    nextPose.position.z += 0.5;
    waypoints.push_back(nextPose);

    moveit_msgs::RobotTrajectory trajectory_1;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = 0;
    int maxtries = 100;
    int attempts = 0;

    while(fraction< 1.0 && attempts < maxtries)
    {
        fraction = arm.computeCartesianPath(waypoints,eef_step,jump_threshold,trajectory_1);
        attempts ++;
        if(attempts % 10 == 0) ROS_INFO("Attempts No %d, success: %.4f",attempts,fraction);
    }
    if(fraction == 1){
        ROS_INFO("Successfully planed");
        
    }
    else{
        ROS_INFO("Planning Failed, only %.2f percent success",fraction*100);
        ros::shutdown();
        return 0;
    }

    double centerA = startPose.position.y;   // Make the height not change, which means only change x and y, while leaving z untouched
    double centerB = startPose.position.z;   
    double radius = 0.1;    // draw a circle with the radius of 5 cm
    geometry_msgs::Pose target_pose;
    target_pose = startPose;
    for(double th = 0.0; th < 2*M_PI; th+=0.001)
    {
        target_pose.position.y = centerA + radius * cos(th);
        target_pose.position.z = centerB + radius * sin(th);
        // target_pose.position.z = startPose.position.z - 0.2;
        waypoints.push_back(target_pose);
    }
    ROS_INFO("amount of points: %d",waypoints.size());
    moveit_msgs::RobotTrajectory trajectory_2;
    fraction = 0;
    attempts = 0;
    while(fraction < 1 && attempts < maxtries)
    {
        fraction = arm.computeCartesianPath(waypoints,eef_step,jump_threshold,trajectory_2);
        attempts ++;
        if(attempts % 10 == 0) ROS_INFO("Attempts No %d, success: %.4f",attempts,fraction);
    }
    if(fraction == 1)
    {
        ROS_INFO("Successfully planed");
    }
    else{
        ROS_INFO("Planning Failed, only %.2f percent success",fraction*100);
        ros::shutdown();
        return 0;
    }

    moveit_msgs::RobotTrajectory final_trajectory;
    final_trajectory.joint_trajectory.joint_names = trajectory_1.joint_trajectory.joint_names;
    final_trajectory.joint_trajectory.points = trajectory_1.joint_trajectory.points;
    for(int i = 0; i < trajectory_2.joint_trajectory.points.size(); i++){
        final_trajectory.joint_trajectory.points.push_back(trajectory_2.joint_trajectory.points[i]);
    }

    moveit::planning_interface::MoveGroupInterface::Plan finalPlan;
    robot_trajectory::RobotTrajectory rt(arm.getCurrentState()->getRobotModel(),"manipulator");
    

    ros::shutdown();
    return 0;
}