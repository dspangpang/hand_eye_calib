//
// Created by jzz on 23-2-4.
//

#ifndef HAND_EYE_CALIB_MATRIX_TRANSFORM_H
#define HAND_EYE_CALIB_MATRIX_TRANSFORM_H

#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>


/**
* @brief 把旋转矩阵与平移矩阵合并为[R T;0 1]矩阵
* @param R 旋转矩阵
* @param T 平移矩阵
* @return 坐标变换矩阵
*/
cv::Mat r_t2rt(cv::Mat &R, cv::Mat &T);

/**
* @brief 把[R T;0 1]矩阵分解为旋转矩阵与平移矩阵
* @param RT 坐标变换矩阵
* @param R  旋转矩阵
* @param T  平移矩阵
*/
void rt2r_t(cv::Mat &RT, cv::Mat &R, cv::Mat &T);


/**
* @brief 判断输入的矩阵是否为旋转矩阵
* @param R 旋转矩阵
* @return ture false
*/
bool is_rotation_matrix(const cv::Mat & R);

/**
* @brief 欧拉角 -> 3*3 的 旋转矩阵
* @param 	eulerAngle	角度值
* @param 	seq			指定欧拉角xyz的排列顺序如："xyz" "zyx"
* @return 3*3 的旋转矩阵
*/
cv::Mat euler_angle_to_rotated_matrix(const cv::Mat& euler_angle, const std::string& seq);

/**
* @brief 四元数转旋转矩阵
* @note  数据类型double； 四元数定义 q = w + x*i + y*j + z*k
* @param q 四元数输入{w,x,y,z}向量
* @return 返回3*3旋转矩阵
*/
cv::Mat quaternion_to_rotated_matrix(const cv::Vec4d& q);

/**
* @brief ((四元数||欧拉角||旋转向量) && 转移向量) -> 4*4 的Rt
* @param 	m				1*6 || 1*10的矩阵  -> 6  {x,y,z, rx,ry,rz}   10 {x,y,z, qw,qx,qy,qz, rx,ry,rz}
* @param 	is_quaternion	如果是1*10的矩阵，判断是否使用四元数计算旋转矩阵
* @param 	seq				如果通过欧拉角计算旋转矩阵，需要指定欧拉角xyz的排列顺序如："xyz" "zyx" 为空表示旋转向量
*/
cv::Mat attitude_vector_to_Matrix(cv::Mat& m, bool is_quaternion, const std::string& seq);


#endif //HAND_EYE_CALIB_MATRIX_TRANSFORM_H