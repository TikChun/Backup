#include "ros/ros.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "string"
#include "Eigen/Geometry"

int main(int argc, char ** argv){
    ros::init(argc,argv,"moveit_ik");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    
    std::string end_effector_name = arm.getEndEffectorLink();

    std::string reference_frame = "base link";
    arm.setPoseReferenceFrame(reference_frame);

    arm.allowReplanning(true);

    arm.setGoalPositionTolerance(0.01);
    arm.setGoalOrientationTolerance(0.01);

    arm.setMaxAccelerationScalingFactor(0.01);
    arm.setMaxVelocityScalingFactor(0.01);

    sleep(1);

    geometry_msgs::PoseStamped current_pose = arm.getCurrentPose();
    ROS_INFO("Current_orientation.x = %.2f",current_pose.pose.orientation.x);
    ROS_INFO("Current_orientation.y = %.2f",current_pose.pose.orientation.y);
    ROS_INFO("Current_orientation.z = %.2f",current_pose.pose.orientation.z);
    ROS_INFO("Current_orientation.w = %.2f",current_pose.pose.orientation.w);
    
    ROS_INFO("Current_position.x = %.2f",current_pose.pose.position.x );
    ROS_INFO("Current_position.y = %.2f",current_pose.pose.position.y );
    ROS_INFO("Current_position.z = %.2f",current_pose.pose.position.z );
    


    
    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = current_pose.pose.orientation.x;
    target_pose.orientation.y = current_pose.pose.orientation.y;
    target_pose.orientation.z = current_pose.pose.orientation.z;
    target_pose.orientation.w = current_pose.pose.orientation.w;

    target_pose.position.x = current_pose.pose.position.x;
    target_pose.position.y = current_pose.pose.position.y;
    target_pose.position.z = current_pose.pose.position.z;

    ROS_INFO("Target_orientation.x = %.2f", target_pose.orientation.x);
    ROS_INFO("Target_orientation.y = %.2f", target_pose.orientation.y);
    ROS_INFO("Target_orientation.z = %.2f", target_pose.orientation.z);
    ROS_INFO("Target_orientation.w = %.2f", target_pose.orientation.w);

    ROS_INFO("Target_position.x = %.2f", target_pose.position.x + 0.05);
    ROS_INFO("Target_position.y = %.2f", target_pose.position.y + 0.05);
    ROS_INFO("Target_position.z = %.2f", target_pose.position.z);

    arm.setStartStateToCurrentState();

    arm.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode success = arm.plan(plan);

    ROS_INFO("Plan state: %s", success?"":"Failed");

    if(success){
        ROS_INFO("successful planing");
        // arm.execute(plan);
        sleep(1);
    } 
    // ROS_INFO("finish execuation");

    ros::shutdown();

    return 0;
}