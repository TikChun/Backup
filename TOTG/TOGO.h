#ifndef TOGO_H
#define TOGO_H

#include <vector>

struct simple_position{
    double x;
    double y;
    double z;
    
    simple_position(){
        x = 0;
        y = 0;
        z = 0;
    };

    simple_position(double input_x, double input_y, double input_z){
        x = input_x;
        y = input_y;
        z = input_z;
    };
};

struct simple_circle{
    double s_start;
    double s_end;
    double radius;
    Eigen::Vector3d x_hat;
    Eigen::Vector3d y_hat;
    Eigen::Vector3d center;
    
    simple_circle();

    simple_circle(double input_start, double input_end, double input_radius, Eigen::Vector3d &input_x_hat, Eigen::Vector3d &input_y_hat, Eigen::Vector3d &input_center);

    bool is_valid();
};

struct circle_list{
    // This list is used to hold all the circles generated in the entire path
    std::vector<simple_circle> list_of_circle;
    int current_circle_count;   // Keep track of the num of circle that has passed
    circle_list();
    void add_circle_in_order(simple_circle circle); // Add circle in the end of the list (its position might be changed)
    void swap_circle(int num_1, int num_2);         // Swap the location of two circles in the list
    simple_circle get_circle_with_s(double current_s);  // Search the circle that current_s lies on, if no found, return a circle full of -1
    simple_circle get_next_circle();    // Get the next circle, based on the internal circle counter 
    double get_last_s();            // Return the s_end of the last circle
    double get_first_s();           // Return the s_start of the first circle
    void add_circle(simple_circle circle);  // Add a circle, does not have to be in-order
    void print_list_of_circle();    // Print all the info in this list_of_circle
};

#endif