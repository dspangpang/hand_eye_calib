//
// Created by jzz on 23-2-7.
//
#include "../include/hand_eye_calib/HandEyeCalib.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "hand_eye_calib_node");
    ros::NodeHandle nh;

    auto config_dir = ros::package::getPath("hand_eye_calib") + "/config";

    HandEyeCalib hec(config_dir);
    hec.run();

    return 0;

}