#include "unistd.h"
#include "iostream"
#include "eigen3/Eigen/Core"
#include "TOGO.h"
#include "math_helper.h"

/*
    This file implemented the preprocessing of the path, before actually generating the trajectory using TOGO
    This is a necessary step to ensure the successfuly generation of TOGO path.

    In general, this funtion will take in a set of waypoints. and making cirular segments connecting the two adjoining segments
    of each waypoint, to avoid sharp corner and make them differentiable.

    All circular segments will follow the following properties.
    1. Starts tengential to the linear path segment before the way points and ends tangential to the linear path segment after the waypoint.
    2. Circular segment does not replace more than half of each of the neighbouring segments.
    3. Allow rthe enforcement of a maximum deviation from the original path
*/

// path_preprocessing_withSimplePosition is not recommended

/*
    Calculate all the components needed for the circle segment in waypoint "middle_point", using simple_position 
*/
void path_preprocessing_withSimplePosition(simple_position &preceding_point, simple_position &middle_point, simple_position &end_point, double delta = -1){   
    Eigen::Vector3d first_straight_line_segment = calculateVector_bw_points(preceding_point, middle_point); // q_I - q_I-1
    Eigen::Vector3d second_straight_line_segment = calculateVector_bw_points(middle_point, end_point);      // q_I+1 - q_I
    Eigen::Vector3d first_unit_vector = first_straight_line_segment.normalized();   // Y_i      The unit vector of preceding_point pointed to middle_point
    Eigen::Vector3d second_unit_vector = second_straight_line_segment.normalized(); // Y_i+1    The unit vector of middle_point pointed to end_point
    double angle = calculateAngle_bw_vectors(first_unit_vector,second_unit_vector); // A_i      The angle between two adjoining segments of middle_point
    double distance = calculateDistance_bw_waypoint_circleTouchingPoint(preceding_point,middle_point,end_point,delta,angle);    // L_i  The distance between q_I and the point where
                                                                                                                                // the circle touches the line segments
    double radius = distance / tan(angle / 2);  // r_i  The radius of the circle
    simple_position center = calculateCenter(middle_point,first_unit_vector,second_straight_line_segment,radius,angle); // C_i  The center position of the circle
    Eigen::Vector3d y_hat = second_unit_vector;     // y_hat    Define the place in which the circle lies. This step is not necessary, as no calculation were done. But good for understanding
    Eigen::Vector3d x_hat = calculateXhat(middle_point,distance,first_unit_vector,center);
}

/*
    Calculate all the components needed for the circle segment in waypoint "middle_point", without using simple_position 
*/
void path_preprocessing(Eigen::Vector3d &preceding_point, Eigen::Vector3d &middle_point, Eigen::Vector3d &end_point, double delta = -1){   
    Eigen::Vector3d first_straight_line_segment = calculateVector_bw_points(preceding_point, middle_point); // q_I - q_I-1
    Eigen::Vector3d second_straight_line_segment = calculateVector_bw_points(middle_point, end_point);      // q_I+1 - q_I
    Eigen::Vector3d first_unit_vector = first_straight_line_segment.normalized();   // Y_i      The unit vector of preceding_point pointed to middle_point
    Eigen::Vector3d second_unit_vector = second_straight_line_segment.normalized(); // Y_i+1    The unit vector of middle_point pointed to end_point
    double angle = calculateAngle_bw_vectors(first_unit_vector,second_unit_vector); // A_i      The angle between two adjoining segments of middle_point
    double distance = calculateDistance_bw_waypoint_circleTouchingPoint(preceding_point,middle_point,end_point,delta,angle);    // L_i  The distance between q_I and the point where
                                                                                                                                // the circle touches the line segments
    double radius = distance / tan(angle / 2);  // r_i  The radius of the circle
    Eigen::Vector3d center = calculateCenter(middle_point,first_unit_vector,second_straight_line_segment,radius,angle); // C_i  The center position of the circle
    Eigen::Vector3d y_hat = second_unit_vector;     // y_hat    Define the place in which the circle lies. This step is not necessary, as no calculation were done. But good for understanding
    Eigen::Vector3d x_hat = calculateXhat(middle_point,distance,first_unit_vector,center);  // x_hat    Define the place in which the circle lies
}

int main(){
    // simple_position first_point(2,2,3);
    Eigen::Vector3d first_point(2,2,3);
    // simple_position secod_point(3,5,6);
    Eigen::Vector3d secod_point(3,5,6);
    // simple_position third_point(3,8,9);
    Eigen::Vector3d third_point(3,8,9);
    path_preprocessing(first_point,secod_point, third_point);
    play_ground();
    return 0;
}