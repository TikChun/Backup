#include "ros/ros.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_trajectory/robot_trajectory.h"

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

int main(int argc, char * argv[]){
    ros::init(argc,argv,"moveit_circle_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    ROS_INFO("update5");

    std::string end_effector_link = arm.getEndEffectorLink();

    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

    arm.allowReplanning(true);

    arm.setGoalOrientationTolerance(0.01);
    arm.setGoalPositionTolerance(0.01);

    arm.setMaxAccelerationScalingFactor(0.1);
    arm.setMaxVelocityScalingFactor(0.1);

    // move_2_rest(arm);

    geometry_msgs::Pose current_pose;

    current_pose = arm.getCurrentPose().pose;

    std::vector<geometry_msgs::Pose>waypoints;

    double centerA = current_pose.position.y;   // Make the height not change, which means only change x and y, while leaving z untouched
    double centerB = current_pose.position.z;   
    double radius = 0.1;    // draw a circle with the radius of 5 cm

    geometry_msgs::Pose target_pose;
    target_pose = current_pose;
    for(double th = 0.0; th < 2*M_PI; th+=0.001)
    {
        target_pose.position.y = centerA + radius * cos(th);
        target_pose.position.z = centerB + radius * sin(th);
        // target_pose.position.z = current_pose.position.z - 0.2;
        waypoints.push_back(target_pose);
    }
    ROS_INFO("amount of points: %d",waypoints.size());

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = 0.0;
    int maxtries = 100;
    int attempts = 0;

    while(fraction < 1 && attempts < maxtries)
    {
        fraction = arm.computeCartesianPath(waypoints,eef_step,jump_threshold,trajectory);
        attempts ++;
        if(attempts % 10 == 0) ROS_INFO("Attempts No %d, success: %.4f",attempts,fraction);
    }

    if(fraction == 1){
        ROS_INFO("Successfully planed");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        set_traj_speed(plan, 0.3);  // Slow down the trajectory, the original one is not bounded by the arm.setMax...
        arm.execute(plan);
        sleep(1);
        // ROS_INFO("Finish drawing circles, going back to rest");
        // move_2_rest(arm);
        // sleep(1);
        
    }
    else{
        ROS_INFO("Planning Failed, only %.2f percent success",fraction*100);
    }

    ros::shutdown();
    return 0;
}