# pragma once

#include <iostream>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <jsoncpp/json/json.h>
#include <opencv2/opencv.hpp>


/**
 * @brief 读取json文件,并把数据转换成字符串
 * @param file_path json文件的路径
 * @return json文件内容的字符串
*/
std::string parseJsonFile(const std::string& file_path);

/**
 * @brief 把json文件中的key,转换成eigen矩阵
 * @param file_path json文件的路径
 * @param key json文件中的键值
 * @return Eigen矩阵
*/
Eigen::MatrixXd parseJsonEigenMatrix(const std::string& file_path, const std::string& key);

/**
 * @brief 把json文件中的key,转换成cv::Mat
 * @param file_path json文件的路径
 * @param key json文件中的键值
 * @return cv::Mat矩阵
*/
cv::Mat parseJsonCVMatrix(const std::string& file_path, const std::string& key);

/**
 * @brief 把json文件中的key,转换成double数据
 * @param file_path json文件的路径
 * @param key json文件中的键值
 * @return double数据
*/
double parseJsonDouble(const std::string& file_path, const std::string& key);

/**
 * @brief 把json文件中的key,转换成int数据
 * @param file_path json文件的路径
 * @param key json文件中的键值
 * @return int数据
*/
int parseJsonInt(const std::string& file_path, const std::string& key);

/**
 * @brief 把json文件中的key,转换成bool数据
 * @param file_path json文件的路径
 * @param key json文件中的键值
 * @return bool数据
*/
bool parseJsonBool(const std::string& file_path, const std::string& key);

/**
 * @brief 把json文件中的key,转换成double数据
 * @param file_path json文件的路径
 * @param key json文件中的键值
 * @return double数据
*/
std::string parseJsonString(const std::string& file_path, const std::string& key);