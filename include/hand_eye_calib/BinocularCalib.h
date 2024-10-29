#ifndef MONOCULAR_CALIB_MONOCULARCALIB_H
#define MONOCULAR_CALIB_MONOCULARCALIB_H

#include <iostream>
#include <vector>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "./jsonparser.hpp"

class BinocularCalib {
private:
    // calib 配置文件信息
    double scale;
    bool visulation;
    int imagesNum;
    std::string config_path;
    std::string json_path;
    std::string image_dir_1;
    std::string image_dir_2;
    std::pair<int, int> board_squre_num;
    double square_size;

public:
    BinocularCalib(
        const std::string &config_path, 
        const std::string &calib_board_type);
    
    ~BinocularCalib();

    void run();
    
private:

    /**
     * @brief 读取json文件
     * @param json_path json文件的路径
     */
    void get_calib_config_data(const std::string &json_path);

    /**
     * @brief 开始执行相机标定的步骤
     */
    int setup_calibration();
};



#endif //MONOCULAR_CALIB_MONOCULARCALIB_H
