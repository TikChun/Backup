// #include "ros/ros.h"
// #include "tf2/LinearMath/Quaternion.h"
// #include "tf2_ros/static_transform_broadcaster.h"
// #include "geometry_msgs/Transform.h"
// #include "geometry_msgs/PointStamped.h"
// #include "tf2/LinearMath/Vector3.h"
// #include "tf2/LinearMath/Matrix3x3.h"
#include "iostream"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "cmath"


Eigen::Vector3d hand_rotation_translate(Eigen::Vector3d input, double rotation_angle_degree, Eigen::Vector3d rotation_axis, bool debug=false){
    if(rotation_angle_degree == 0) return input;
    double rotation_angle_radian = rotation_angle_degree / 180 * M_PI;
    rotation_axis.stableNormalize();
    double u_x = rotation_axis[0];
    double u_y = rotation_axis[1];
    double u_z = rotation_axis[2];
    if(debug) printf("\n u_z: %.2f, u_y: %.2f, u_z: %.2f\n",u_x,u_y,u_z);

    double a = cos(0.5 * rotation_angle_radian);
    double b = sin(0.5 * rotation_angle_radian) * u_x;
    double c = sin(0.5 * rotation_angle_radian) * u_y;
    double d = sin(0.5 * rotation_angle_radian) * u_z;

    if(debug) printf("\n a: %.2f, b:%.2f, c:%.2f, d:%.2f \n", a, b, c, d);

    double x_x = 1 - 2 * pow(c,2) - 2 * pow(d,2);
    double x_y = 2 * b * c - 2 * a * d;
    double x_z = 2 * a * c + 2 * b * d;
    double y_x = 2 * b * c - 2 * a * d;
    double y_y = 1 - 2 * pow(b,2) - 2 * pow(d,2);
    double y_z = 2 * c * d - 2 * a * b;
    double z_x = 2 * b * d - 2 * a * c;
    double z_y = 2 * a * b + 2 * c * d;
    double z_z = 1 - 2 * pow(b,2) - 2 * pow(c,2); 

    Eigen::Matrix3d translation_matrix;
    translation_matrix <<   x_x, x_y, x_z,
                            y_x, y_y, y_z,
                            z_x, z_y, z_z;
    if(debug) std::cout << translation_matrix << std::endl;


    if(debug) std::cout << input <<std::endl;

    Eigen::Vector3d v = translation_matrix*input;

    if(debug) std::cout << v << std::endl; 

    return v;

}

Eigen::Quaterniond euler_2_quaternion(double roll, double pitch, double yaw, bool debug=false){
    int w = cos(roll / 2.0) * cos(pitch / 2.0) * cos(yaw / 2.0) + sin(roll / 2.0) * sin(pitch / 2.0) * sin(yaw / 2.0);
    int x = sin(roll / 2.0) * cos(pitch / 2.0) * cos(yaw / 2.0) - cos(roll / 2.0) * sin(pitch / 2.0) * sin(yaw / 2.0);
    int y = cos(roll / 2.0) * sin(pitch / 2.0) * cos(yaw / 2.0) + sin(roll / 2.0) * cos(pitch / 2.0) * sin(yaw / 2.0);
    int z = cos(roll / 2.0) * cos(pitch / 2.0) * sin(yaw / 2.0) - sin(roll / 2.0) * sin(pitch / 2.0) * cos(yaw / 2.0);
    Eigen::Quaterniond qtn(w,x,y,z);

    // // Smart approach, no need to handwrite all the formulas
    // Eigen::Vector3d euler0(roll,pitch,yaw);
    // Eigen::Matrix3d rotation_matrix;
    // rotation_matrix =   Eigen::AngleAxisd(euler0[0], Eigen::Vector3d::UnitZ())
    //                     * Eigen::AngleAxisd(euler0[1], Eigen::Vector3d::UnitY())
    //                     * Eigen::AngleAxisd(euler0[2], Eigen::Vector3d::UnitX());
    // // The reason of not using this matrix directly is due to its inconsistency with the result in textbook

    // if(debug) std::cout<< rotation_matrix << std::endl;
    // Eigen::Quaterniond qtn;
    // qtn = rotation_matrix;
    return qtn;
}

void vec_round(Eigen::Vector3d &vec, int decimal_place){
    double round_num = pow(10, decimal_place);
    for(int i = 0; i < 3; i++){
        vec[i] = round(vec[i] * round_num) / round_num; // round to two decimal place
        if(vec[i] == -0) vec[i] = 0;
    }
}

Eigen::Vector3d without_q_tf(Eigen::Vector3d initial_point, double roll=0.0, double pitch=0.0, double yaw=0.0, bool debug=false){
    Eigen::Vector3d result = initial_point;
    Eigen::Vector3d axis;
    if(roll != 0.0){
        axis = Eigen::Vector3d(1,0,0);
        result = hand_rotation_translate(result,360 * (roll / (2 * M_PI)),axis);
        if(debug) std::cout<<"after roll: \n" << result << std::endl;
    }
    if(pitch != 0.0){
        axis = Eigen::Vector3d(0,1,0);
        result = hand_rotation_translate(result,360 * (pitch / (2 * M_PI)),axis);
        if(debug) std::cout<<"after pitch: \n" << result << std::endl;
    }
    if(yaw != 0.0){
        axis = Eigen::Vector3d(0,0,1);
        result = hand_rotation_translate(result,360 * (yaw / (2 * M_PI)),axis);
        if(debug) std::cout<<"after yaw: \n" << result << std::endl;
    }

    return result;
}

void auto_tf(Eigen::Vector3d &initial_point, double roll=0.0, double pitch=0.0, double yaw=0.0,bool debug=false){
    // Eigen::Vector3d euler0(yaw,roll,pitch);
    // Eigen::Matrix3d rotation_matrix;
    // rotation_matrix =   Eigen::AngleAxisd(euler0[0], Eigen::Vector3d::UnitZ())
    //                     * Eigen::AngleAxisd(euler0[1], Eigen::Vector3d::UnitY())
    //                     * Eigen::AngleAxisd(euler0[2], Eigen::Vector3d::UnitX());
    
    if(debug) std::cout<<"roll: "<<360 * (roll / (2 * M_PI))<<" degree"<<std::endl;
    if(debug) std::cout<<"pitch: "<<360 * (pitch / (2 * M_PI))<<" degree"<<std::endl;
    if(debug) std::cout<<"yaw: "<<360 * (yaw / (2 * M_PI))<<" degree"<<std::endl;

    
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
    // Eigen::Matrix3d rotationMatrix = q.matrix();
    Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();

    
    initial_point = rotationMatrix * initial_point;
    if(debug) std::cout<<"result:\n"<<initial_point<<std::endl;
}

Eigen::Vector3d creative_tf(Eigen::Vector3d &initial_point, Eigen::Vector3d &world_offset, double roll=0.0, double pitch=0.0, double yaw=0.0,bool debug=false, bool combined=true){
    Eigen::Vector3d result = initial_point;
    if(combined){
        auto_tf(result,roll,pitch,yaw,debug);   // The is the correct way to go, unless specify, all rotations will be calculated at once
    }
    else{
        if(roll != 0) auto_tf(result,roll,0,0,debug);
        if(pitch != 0) auto_tf(result,0,pitch,0,debug);
        if(yaw != 0) auto_tf(result,0,0,yaw,debug);
    }
    
    if(debug) std::cout<<"before apply offset, translated coordinate: \n"<<result<<std::endl;

    // Now we have calculated the position relative to the new world frame, but until now, 
    // we have assumed that the new workld frame shared the same origin with the old one

    if(world_offset[0] == 0 && world_offset[1] == 0 && world_offset[2] == 0)   return result;   // no offset

    Eigen::Vector3d world_offset_tf = world_offset;
    if(combined){
        auto_tf(world_offset_tf,roll,pitch,yaw,debug);  // The is the correct way to go, unless specify, all rotations will be calculated at once
    }
    else{
        if(roll != 0) auto_tf(world_offset_tf,roll,0,0,debug);
        if(pitch != 0) auto_tf(world_offset_tf,0,pitch,0,debug);
        if(yaw != 0) auto_tf(world_offset_tf,0,0,yaw,debug); 
    }

    if(debug) std::cout<<"world offset translated coordinate: \n"<<world_offset_tf<<std::endl;

    result -= world_offset_tf;

    return result;
}

// example: ./tf01 1 1 1 1 1 1 debug 
int main(int argc, char *argv[]){

    // The new world frame's rotation relative to the current frame
    double roll = std::stod(argv[1]);
    double pitch = std::stod(argv[2]);
    double yaw = std::stod(argv[3]);

    bool debug = false;
    if(argc > 7 && strcmp(argv[7],"debug")==0) debug = true;
    if(debug)   printf("debug mode\n");

    std::cout<<"roll: "<<360 * (roll / (2 * M_PI))<<" degree"<<std::endl;
    std::cout<<"pitch: "<<360 * (pitch / (2 * M_PI))<<" degree"<<std::endl;
    std::cout<<"yaw: "<<360 * (yaw / (2 * M_PI))<<" degree"<<std::endl;

    // Compute quarternion, not mandatory in transform (outdated comment)
    // Eigen::Quaterniond qtn = euler_2_quaternion(roll, pitch, yaw);
    
    // Setting up World 1
    Eigen::Vector3d world_offset(std::stod(argv[4]),std::stod(argv[5]),std::stod(argv[6]));
    std::cout<<"world offset: x: "<<world_offset[0]<<" y: "<<world_offset[1]<<" z: "<<world_offset[2]<<std::endl;
    
    // Setting a coordinate in world 0
    
    Eigen::Vector3d point_1(2,0,5);

    std::cout<<"initial point: \n" << point_1 << std::endl;

    Eigen::Vector3d result = creative_tf(point_1,world_offset,roll,pitch,yaw,debug,true);  // If set combine = false, will rotate x, y, z axises individually, gimbal lock
    
    vec_round(result,2);    // round to two decimal place

    std::cout << "result: \n" << result << std::endl;
    // rotation_translate();

    return 0;
}
// If anything goes sideways, replace the code with https://zhuanlan.zhihu.com/p/144032401