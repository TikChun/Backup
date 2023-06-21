#include "TOGO.h"
#include "eigen3/Eigen/Core"
#include <iostream>

struct simple_circle{
    double s_start;
    double s_end;
    double radius;
    Eigen::Vector3d x_hat;
    Eigen::Vector3d y_hat;
    Eigen::Vector3d center;
    
    simple_circle(){
        s_start = -1;
        s_end = -1;
        radius = -1;
        x_hat = Eigen::Vector3d(-1,-1,-1);
        y_hat = Eigen::Vector3d(-1,-1,-1);
        center = Eigen::Vector3d(-1,-1,-1);
    };

    simple_circle(double input_start, double input_end, double input_radius, Eigen::Vector3d &input_x_hat, Eigen::Vector3d &input_y_hat, Eigen::Vector3d &input_center){
        s_start = input_start;
        s_end = input_end;
        x_hat = input_x_hat;
        y_hat = input_y_hat;
        center = input_center;
    };

    bool is_valid(){
        if(s_start == -1 || s_end == -1 || radius == -1) return false;
        if(s_end <= s_start) return false;
        return true;
    }
};

struct circle_list{
    // This list is used to hold all the circles generated in the entire path
    // THe s-based sorting is not implemented yet, therefore, we expect all circles are added in-order
    std::vector<simple_circle> list_of_circle;
    int current_circle_count;   // Keep track of the num of circle that has passed
    circle_list(){
        list_of_circle = std::vector<simple_circle>();
        current_circle_count = -1;
    }
    void add_circle_in_order(simple_circle circle){
        list_of_circle.push_back(circle);
    }
    simple_circle get_circle_with_s(double current_s){
        for(int i = 0; i < list_of_circle.size(); ++i)
        {
            if(list_of_circle[i].s_start <= current_s && list_of_circle[i].s_end >= current_s)
            {
                current_circle_count = i;
                return list_of_circle[i];
            }
        }
        return simple_circle();
    }
    void swap_circle(int num1, int num2){
        list_of_circle[num1], list_of_circle[num2] = list_of_circle[num2], list_of_circle[num1];
    }
    simple_circle get_next_circle(){
        if(current_circle_count >= list_of_circle.size())   return simple_circle();
        current_circle_count += 1;
        return list_of_circle[current_circle_count - 1];
    }
    double get_last_s(){
        return list_of_circle[list_of_circle.size() -1].s_end;
    }
    double get_first_s(){
        return list_of_circle[0].s_start;
    }
    void add_circle(simple_circle circle){
        if(!circle.is_valid()) return;
        if(list_of_circle.empty()) return add_circle_in_order(circle);
        if(circle.s_start >= get_last_s()) return add_circle_in_order(circle);
        
        // If it goes all the way here, it means that it's not in order
        list_of_circle.push_back(circle);
        // Check corner cases, is it the first circle ?
        if(circle.s_start < get_first_s()){
            if(list_of_circle.size() == 2){
                swap_circle(0,1);
                return ;
            }
            for(int i = (list_of_circle.size() - 1); i > 0; --i){
                swap_circle(i,i-1);
            }
            return;
        }  
        // Consider the case where the circle lies in the middle
        // Assume other than the new circle, the previous list_of_circle is well sorted
        for(int i = (list_of_circle.size() - 1); i > 0; --i){
            if(list_of_circle[i].s_start < list_of_circle[i-1].s_start){
                swap_circle(i,i-1);
            }
        }
        return ;
    }
    void print_list_of_circle(){
        for(int i = 0; i < list_of_circle.size(); ++i){
            if(i == 0)  std::cout<<"/////////////////////////////\n"<<"This is the first circle"<<std::endl;
            if(i == list_of_circle.size()-1)  std::cout<<"This is the last circle"<<std::endl;
            std::cout <<"Circle # "<<i<<std::endl;
            std::cout <<"S ranges from "<<list_of_circle[i].s_start<<" to "<<list_of_circle[i].s_end << std::endl;
            std::cout <<"Radius: "<<list_of_circle[i].radius << "\n/////////////////////////////" <<std::endl;
        }
    }
};
