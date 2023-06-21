#ifndef MATH_HELPER_H
#define MATH_HELPER_H

#include "TOGO.h"
#include "eigen3/Eigen/Core"
#include "unistd.h"
#include "iostream"
#include <vector>

/*
This file include all the math related helper functions involved in calculating TOGO trajectory
notice that some of the functions are not mathmatically rigorous, as might having some assumtion
Improved functions will be listed shortly
*/

// Calculate the vector in which pointed from start_point to end_point
Eigen::Vector3d calculateVector_bw_points(Eigen::Vector3d &start_point, Eigen::Vector3d &end_point);

// Calculate the angle between two vectors
double calculateAngle_bw_vectors(Eigen::Vector3d &first_vector, Eigen::Vector3d &second_vector);

// Calculate the distance between the waypoint and the point where the circle segment and the original line segment intercept
double calculateDistance_bw_waypoint_circleTouchingPoint(Eigen::Vector3d &preceding_point, Eigen::Vector3d &middle_point, Eigen::Vector3d &end_pointm, double delta, double angle);

// Calculate the position of the center of the circular segment
Eigen::Vector3d calculateCenter(Eigen::Vector3d &middle_point, Eigen::Vector3d &first_unit_vector, Eigen::Vector3d &second_unit_vector, double radius, double angle);

// Calculate the x_hat, which can be used together with y_hat to determine the plane that the circle lies on
Eigen::Vector3d calculateXhat(Eigen::Vector3d &middle_point, double distacne, Eigen::Vector3d &first_unit_vector, Eigen::Vector3d &center);

// Calculate the robot configuration f(s)
Eigen::Vector3d calculateRobotConfig(Eigen::Vector3d &center, double radius, Eigen::Vector3d &x_hat, double s, Eigen::Vector3d &y_hat);


Eigen::Vector3d calculate_firstDerivative_RobotConfig(Eigen::Vector3d &x_hat, Eigen::Vector3d &y_hat, double radius, double s);


void play_ground();
#endif