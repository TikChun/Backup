#include "ros/ros.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"


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

void set_traj_speed(moveit::planning_interface::MoveGroupInterface::Plan &plan, double scale){
    int num = plan.trajectory_.joint_trajectory.joint_names.size();
    for(int count = 0; count < plan.trajectory_.joint_trajectory.points.size(); count++)
    {
        plan.trajectory_.joint_trajectory.points[count].time_from_start *= 1/scale;
        for(int i = 0; i < num; i++)
        {
            plan.trajectory_.joint_trajectory.points[count].velocities[i] *= scale;
            plan.trajectory_.joint_trajectory.points[count].accelerations[i] *= scale * scale;
        }
    }
}



int main(int argc, char *argv[]){
    ros::init(argc,argv,"moveit_carterian_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    std::string end_effector_link = arm.getEndEffectorLink();

    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

    arm.allowReplanning(true);

    arm.setGoalOrientationTolerance(0.01);
    arm.setGoalPositionTolerance(0.01);

    arm.setMaxVelocityScalingFactor(0.05);
    arm.setMaxAccelerationScalingFactor(0.05);

    geometry_msgs::Pose start_pose = arm.getCurrentPose(end_effector_link).pose;
    geometry_msgs::Pose start_pose_backup = arm.getCurrentPose(end_effector_link).pose;

    // ROS_INFO("Target_orientation.x = %.2f", start_pose_backup.orientation.x);
    // ROS_INFO("Target_orientation.y = %.2f", start_pose_backup.orientation.y);
    // ROS_INFO("Target_orientation.z = %.2f", start_pose_backup.orientation.z);
    // ROS_INFO("Target_orientation.w = %.2f", start_pose_backup.orientation.w);

    // ROS_INFO("Target_position.x = %.2f", start_pose_backup.position.x);
    // ROS_INFO("Target_position.y = %.2f", start_pose_backup.position.y);
    // ROS_INFO("Target_position.z = %.2f", start_pose_backup.position.z);






    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(start_pose);

    start_pose.position.z += 0.1;
    waypoints.push_back(start_pose);

    start_pose.position.x += 0.1;
    waypoints.push_back(start_pose);

    start_pose.position.y += 0.1;
    waypoints.push_back(start_pose);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = 0;
    int maxtries = 100;
    int attempts = 0;

    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = arm.computeCartesianPath(waypoints,eef_step,jump_threshold,trajectory);
        attempts++;
        ROS_INFO("Trying after attempts No %d",attempts);
    }

    if(fraction == 1){
        ROS_INFO("Successfully computed path");

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        set_traj_speed(plan,0.2);

        // arm.execute(plan);
        sleep(1);
    }
    else{
        ROS_INFO("Failed to compute path with onlt %.2f success",fraction);
    }

    // std::vector<double> joint_target_pose(6);
    
    // joint_target_pose = get_rest_pose();
    
    // int count = 0;
    // for(auto element : joint_target_pose){
    //     count++;
    //     // ROS_INFO("element: %.2f (in degree)",element * 180 / M_PI);
    //     ROS_INFO("target joint Pos No.%d: %.2f (in degree)",count,element  * 180 / M_PI);
    // }

    // arm.setJointValueTarget(joint_target_pose);
    // arm.move();
    // sleep(1);
    // ROS_INFO("Just back in rest pose");
    ROS_INFO("eef_link: %s",arm.getEndEffectorLink().c_str());
    
    ros::shutdown();

    return 0;
}