//
// Created by jzz on 23-2-8.
//

#ifndef HAND_EYE_CALIB_HANDEYECALIB_H
#define HAND_EYE_CALIB_HANDEYECALIB_H

#include <iostream>
#include <vector>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>


#include "./jsonParser.h"
#include "./matrix_transform.h"

class HandEyeCalib {
private:
    // calib 配置文件信息
    std::string calibrationMethod;
    int imagesNum;
    std::string dataDectory;
    std::string armPoseFormat;
    std::string calPoseDataFile;
    std::string gripperPoseDataFile;
    bool usePicture;
    std::string imageDataDectory;

    std::string config_path;
    std::string calPosePath;
    std::string gripperPosePath;
    std::string jsonPath;
public:
    HandEyeCalib(const std::string &config_path);
    ~HandEyeCalib();
    
    int run();
private:

    // 定义手眼标定矩阵计算相关参数
    bool is_quaternion;

    std::vector<cv::Mat> R_gripper2base;
    std::vector<cv::Mat> t_gripper2base;
    std::vector<cv::Mat> R_base2gripper;
    std::vector<cv::Mat> t_base2gripper;
    std::vector<cv::Mat> R_target2cam;
    std::vector<cv::Mat> t_target2cam;
    std::vector<cv::Mat> vecH_gripper2base, vecH_target2cam, vecH_base2gripper;

    // 手眼标定结果
    cv::Mat R_cam2gripper;
    cv::Mat t_cam2gripper;
    cv::Mat R_cam2base;
    cv::Mat t_cam2base;
    cv::Mat H_cam2gripper;
    cv::Mat H_cam2base;

    // 从文件中读取的数据

    // 相机中标定板的位姿，x,y,z，rx,ry,rz,
    cv::Mat CalPose;
    // 机械臂末端位姿 x y z w x y z
    cv::Mat ToolPose;

private:

    /**
     * @brief 读取json文件
     * @param json_path json文件的路径
     */
    void get_calib_config_data(const std::string &json_path);

    /**
     * @brief 把数据格数转换为手眼标定需要的格式
     */
    void reform_hand_eye_data();

    /**
     * @brief 从文件里读取手眼标定需要的数据
     * @return 0:成功 else:失败
     */
    int get_hand_eye_data();

    /**
     * @brief 读取txt文件中的位姿数据并转换成vector
     * @return vector数据
     */
    std::vector<double> get_TXT_data(const std::string &path);

    /**
     * @brief cv::mat转vector
     * @tparam _Tp
     * @param mat
     * @return
     */
    template<typename Tp>
    std::vector<Tp> convertMat2Vector(const cv::Mat &mat);

    /**
     * @brief vector转cv::mat
     * @param v vector
     * @param channels 通道数
     * @param rows 行数
     * @return
     */
    template<typename Tp>
    cv::Mat convertVector2Mat(const std::vector<Tp> v, const int channels, const int rows);

    
};



#endif //HAND_EYE_CALIB_HANDEYECALIB_H
