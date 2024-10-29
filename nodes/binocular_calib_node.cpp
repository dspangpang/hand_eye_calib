#include "../include/hand_eye_calib/BinocularCalib.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "binocular_calib_node");
    ros::NodeHandle nh;

    std::string calibration_board_type = "checkerboard";
    auto config_dir = ros::package::getPath("hand_eye_calib") + "/config";

    BinocularCalib mc(config_dir, calibration_board_type);
    mc.run();

    return 0;

}