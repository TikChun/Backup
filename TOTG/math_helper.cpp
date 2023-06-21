#include "unistd.h"
#include "iostream"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/StdVector"
#include "eigen3/Eigen/Dense"
#include "TOGO.h"
#include "math_helper.h"
#include "cmath"
#include <limits>

/*
    This page collects all the helper functions related to math when solving TOTG.
    Some functions are overloaded with multiple sets of inputs, the performance will variy, but the result should be the same.
    It's highly recommended to go over the comments written in the math_helper.h file, beofre actually using the functions.
    Most functions are not verified yet, some assumptions were made on the paper, misunderstood of the paper is possible. 
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Start of functions used to calculate constrains
/*
    Calculation of the first and second derivative of the robot configuration:
        Formula:    
                f'(s) = - x_hat * [sin(s / r_i) + y_hat * sin(s / r_i)]
                f''(s) = - (1 / r_i) * [(x_hat * sin(s / r_i) + y_hat * (cos(s / r_i)))]
        Note:
            x_hat and y_hat are two unit vectors that indicates the plane the circle lies on
            r_i is the radius of that circle
            s is the arc length travelled from the start, assume that s_i < s < s_i + a_i * r_i; s_i: the start of the circular segment
            x_hat, y_hat and r_i will be changed for every circles, which we need to identify which circle we in using s
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Corrected formula
// Recommended
Eigen::Vector3d calculateVector_bw_points(Eigen::Vector3d &start_point, Eigen::Vector3d &end_point)
{
    return end_point - start_point;
}

// Recommended
double calculateAngle_bw_vectors(Eigen::Vector3d &first_vector, Eigen::Vector3d &second_vector)
{
    double dotProduct = first_vector.dot(second_vector);
    double angle = std::acos(dotProduct / (first_vector.norm() * second_vector.norm()));
    return angle;
}

// Recommended
double calculateDistance_bw_waypoint_circleTouchingPoint(Eigen::Vector3d &preceding_point, Eigen::Vector3d &middle_point, Eigen::Vector3d &end_pointm, double delta, double angle)
{
    Eigen::Vector3d diff;
    diff = preceding_point - middle_point;
    double option_1 = diff.lpNorm<1>();
    diff = middle_point - end_pointm;
    double option_2 = diff.lpNorm<1>();

    if(delta == -1) return std::min<double>({option_1,option_2});    // if delta == -1, no distance contraint is set, so no option_3 should be considered

    double option_3 = (delta * sin(angle / 2)) / (1 - cos(angle / 2));
    return std::min<double>({option_1,option_2,option_3});
}

// Recommended
Eigen::Vector3d calculateCenter(Eigen::Vector3d &middle_point, Eigen::Vector3d &first_unit_vector, Eigen::Vector3d &second_unit_vector, double radius, double angle)
{
    Eigen::Vector3d result = (second_unit_vector - first_unit_vector) / ((second_unit_vector - first_unit_vector).norm());
    result = result * (radius / (cos(angle / 2)));
    result = result + middle_point;
    return result;
}

// Recommended
Eigen::Vector3d calculateXhat(Eigen::Vector3d &middle_point, double distacne, Eigen::Vector3d &first_unit_vector, Eigen::Vector3d &center)
{
    Eigen::Vector3d result;
    result = middle_point - (distacne * first_unit_vector) - center;
    result = result / (result.norm());
    return result;
}

// Recommended
Eigen::Vector3d calculateRobotConfig(Eigen::Vector3d &center, double radius, Eigen::Vector3d &x_hat, double s, Eigen::Vector3d &y_hat)
{
    Eigen::Vector3d result;
    result = radius * (x_hat * cos(s / radius)) + (y_hat * sin(s / radius));
    result += center;
    return result;
}

// Recommended
Eigen::Vector3d calculate_firstDerivative_RobotConfig(Eigen::Vector3d &x_hat, Eigen::Vector3d &y_hat, double radius, double s)
{
    Eigen::Vector3d result;
    result = (x_hat * (sin(s / radius))) * (-1);
    result += y_hat * cos( s / radius);
    return result;
}

// Recommended
Eigen::Vector3d calculate_secondDerivative_RobotConfig(Eigen::Vector3d &x_hat, Eigen::Vector3d &y_hat, double radius, double s)
{
    Eigen::Vector3d result;
    result = ((-1) / radius) * ((x_hat * sin(s / radius)) + (y_hat * cos(s / radius)));
    return result;
}

double calculateJointAccelLimit(Eigen::Vector3d &acceleration_limit_vec, Eigen::Vector3d &f_first_vec, Eigen::Vector3d &f_second_vec)
{
    double result = 10e+5;
    for(int dimension = 0; dimension < 3; dimension++)
    {
        double acceleration_limit_i = acceleration_limit_vec[dimension];
        double f_first_i = f_first_vec[dimension];
        double f_second_i = f_second_vec[dimension];

        // Left component
        if(f_first_i != 0)
        {
            for(int j = dimension+1; j < 3; j++)
            {
                double f_first_j = f_first_vec[j];
                if(f_first_j != 0)
                {   
                    double f_second_j = f_second_vec[j];
                    if(f_second_i/f_first_i - f_second_j/f_first_j)
                    {
                        double acceleration_limit_j = acceleration_limit_vec[j];
                        double upper = (acceleration_limit_i / std::abs(f_first_i)) + (acceleration_limit_j / std::abs(f_first_j));
                        double lower = std::abs((f_second_i / f_first_i) - (f_second_j / f_first_j));
                        double temp = std::sqrt(upper / lower);
                        result = std::min({result, temp});
                    }
                }
            }

        }

        // Right component
        else    // f_first_i == 0
        {
            if(f_second_i != 0)
            {
                double temp = std::sqrt(acceleration_limit_i / std::abs(f_second_i));
                result = std::min({result, temp});
            }
        }
    }
    return result;
}

double calculateJointVelocityLimit(Eigen::Vector3d &acceleration_limit_vec, Eigen::Vector3d &f_first_vec)
{
    double result = 10e+5;
    for(int dimension = 0; dimension < 3; dimension ++)
    {
        double f_first_i = f_first_vec[dimension];
        if(f_first_i != 0)
        {
            result = std::min({result, std::abs(f_first_i)});
        }
    }
    return result;
}

int calculate_i_forDerivative_s(Eigen::Vector3d &acceleration_limit_vec, Eigen::Vector3d &f_first_vec)
{
    int min_dimension = 0;
    double min_value = 10e+5;
    for(int dimension = 0; dimension < 3; dimension++)
    {
        double f_first_i = f_first_vec[dimension];
        if(f_first_i != 0)
        {
            double temp = acceleration_limit_vec[dimension] / std::abs(f_first_i);
            if(temp < min_value){
                min_value = temp;
                min_dimension = dimension;
            }
        }
    }
    return min_dimension;
}

double calculate_Derivative_s_max_vel(int i, Eigen::Vector3d &acceleration_limit_vec, Eigen::Vector3d &f_first_vec, Eigen::Vector3d &f_second_vec)
{
    double acceleration_limit = acceleration_limit_vec[i];
    double f_first_i = f_first_vec[i];
    double f_second_i = f_second_vec[i];
    double result = (-1) * ((acceleration_limit * f_second_i) / (f_first_i * std::abs(f_first_i)));
    return result;
}


// Use to do small tests
void play_ground(){
    std::vector<double> list_double;
    list_double.push_back(0.0);
    list_double.push_back(1.0);
    list_double.push_back(2.0);
    list_double.push_back(3.0);
    list_double.push_back(4.0);
    list_double.push_back(5.0);
    std::cout<<list_double[list_double.size() - 1]<<std::endl;
}