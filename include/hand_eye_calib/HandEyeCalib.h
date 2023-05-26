//
// Created by jzz on 23-2-8.
//

#ifndef HAND_EYE_CALIB_HANDEYECALIB_H
#define HAND_EYE_CALIB_HANDEYECALIB_H

#include "ros/ros.h"
#include <ros/package.h>
#include "iostream"
#include "../matrix_transform/matrix_transform.h"
#include "../ini_parser/INIParser.h"
#include "vector"

#define EYE_IN_HAND 0
#define EYE_TO_HAND 1

class HandEyeCalib {

public:
    HandEyeCalib();
    ~HandEyeCalib();
    //定义手眼标定矩阵计算相关参数
    std::vector<cv::Mat> R_gripper2base;
    std::vector<cv::Mat> t_gripper2base;
    std::vector<cv::Mat> R_base2gripper;
    std::vector<cv::Mat> t_base2gripper;
    std::vector<cv::Mat> R_target2cam;
    std::vector<cv::Mat> t_target2cam;
    std::vector<cv::Mat> vecH_gripper2base, vecH_target2cam, vecH_base2gripper;

    //手眼标定结果
    cv::Mat R_cam2gripper;
    cv::Mat t_cam2gripper;
    cv::Mat R_cam2base;
    cv::Mat t_cam2base;
    cv::Mat H_cam2gripper;
    cv::Mat H_cam2base;

    // config文件读取的数据
    // 相机中标定板的位姿，x,y,z，rx,ry,rz,
    cv::Mat_<double> CalPose;
    // 机械臂末端位姿
    cv::Mat_<double> ToolPose;
    // 存储图片的数量
    size_t num_images;

    // 是否使用四元数
    bool is_quaternion;

    // 是否是使用图片进行手眼标定
    bool is_picture;

    int hand_eye_calib(const std::string &INI_dir, int calib_type);
    int result_test(int calib_type);

private:

    void get_data(const std::string &INI_dir);

    void reform_data_hand_in_eye();

    void reform_data_hand_to_eye();

    std::string get_INI_data(const std::string &path,
                             const std::string &root,
                             const std::string &key);

    std::vector<double> get_TXT_data(const std::string &path);

    /*!
     * @brief 从标定板图片中获得计算pnp获取位姿信息
     * @tparam _Tp
     * @param mat
     * @return
     */
    std::vector<double> get_cal_pnp_data(const std::string &path);


    /*!
     * @brief mat转vector
     * @tparam _Tp
     * @param mat
     * @return
     */
    template<typename Tp>
    std::vector<Tp> convertMat2Vector(const cv::Mat &mat);

    /*!
     * @brief vector转mat
     * @tparam _Tp
     * @param v
     * @param channels
     * @param rows
     * @return
     */
    template<typename Tp>
    cv::Mat convertVector2Mat(const std::vector<Tp> v, const int channels, const int rows);
};



#endif //HAND_EYE_CALIB_HANDEYECALIB_H
