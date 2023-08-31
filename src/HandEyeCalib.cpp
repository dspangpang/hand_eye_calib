//
// Created by jzz on 23-2-8.
//

#include "../include/hand_eye_calib/HandEyeCalib.h"

HandEyeCalib::HandEyeCalib(const std::string &config_path) {
    this->config_path = config_path;
    this->jsonPath = config_path + "/calib_config.json";
    std::cout << "jsonPath : " << jsonPath << std::endl;
    get_calib_config_data(jsonPath);
}

HandEyeCalib::~HandEyeCalib() = default;

void HandEyeCalib::get_calib_config_data(const std::string &json_path){

    this->calibrationMethod = parseJsonString(json_path, "calibrationMethod");
    this->imagesNum = parseJsonInt(json_path, "imagesNum");
    this->dataDectory = parseJsonString(json_path, "dataDectory");
    this->armPoseFormat = parseJsonString(json_path, "armPoseFormat");
    this->calPoseDataFile = parseJsonString(json_path, "calPoseDataFile");
    this->gripperPoseDataFile = parseJsonString(json_path, "gripperPoseDataFile");
    this->usePicture = parseJsonBool(json_path, "usePicture");
    this->imageDataDectory = parseJsonString(json_path, "imageDataDectory");

    this->is_quaternion = armPoseFormat == "quaternion";
    this->calPosePath = config_path + "/" + dataDectory + "/" + calPoseDataFile;
    this->gripperPosePath = config_path + "/" + dataDectory + "/" + gripperPoseDataFile;

    std::cout << "calibrationMethod : " << calibrationMethod << std::endl;
    std::cout << "imagesNum : " << imagesNum << std::endl;
    std::cout << "dataDectory : " << dataDectory << std::endl;
    std::cout << "armPoseFormat : " << armPoseFormat << std::endl;
    std::cout << "calPoseDataFile : " << calPoseDataFile << std::endl;
    std::cout << "gripperPoseDataFile : " << gripperPoseDataFile << std::endl;
    std::cout << "usePicture : " << usePicture << std::endl;
    std::cout << "imageDataDectory : " << imageDataDectory << std::endl;
    std::cout << "is_quaternion : " << is_quaternion << std::endl;
    std::cout << "calPosePath : " << calPosePath << std::endl;
    std::cout << "gripperPosePath : " << gripperPosePath << std::endl;
}

void HandEyeCalib::reform_hand_eye_data() {
    cv::Mat tempR, tempT;
    for (int i = 0; i < this->imagesNum; i++)//计算标定板位姿
    {
        cv::Mat tmp;
        cv::Mat board_pose;
        cv::Mat end_pose;
        board_pose = CalPose.row(int(i)).clone();
        end_pose = ToolPose.row(int(i)).clone();

        // 相机标定板转移向量转旋转矩阵
        tmp = attitude_vector_to_Matrix(board_pose, false, ""); 
        vecH_target2cam.push_back(tmp);
        rt2r_t(tmp, tempR, tempT);
        R_target2cam.push_back(tempR);
        t_target2cam.push_back(tempT);

        // 机械臂末端位姿转旋转矩阵
        tmp = attitude_vector_to_Matrix(end_pose, is_quaternion, "xyz");
        vecH_gripper2base.push_back(tmp);
        rt2r_t(tmp, tempR, tempT);
        R_gripper2base.push_back(tempR);
        t_gripper2base.push_back(tempT);
    }
}

int HandEyeCalib::get_hand_eye_data(){
    auto cal_pose_tmp = get_TXT_data(calPosePath);
    auto gripper_pose_tmp = get_TXT_data(gripperPosePath);
    if(cal_pose_tmp.empty() || gripper_pose_tmp.empty()){
        return -1;
    }

    CalPose = convertVector2Mat<double>(cal_pose_tmp, 1, imagesNum);
    ToolPose = convertVector2Mat<double>(gripper_pose_tmp, 1, imagesNum);
    return 0;
}

std::vector<double> HandEyeCalib::get_TXT_data(const std::string &path) {
    //创建流对象
    std::ifstream f_in;
    //指定打开方式
    f_in.open(path, std::ios::in);

    double tmp;
    std::vector<double> data;
    if (!f_in.is_open()) {
        std::cout << "Could not open txt data" << std::endl;
        return data;
    }
    while (f_in >> std::setprecision(8) >> tmp) {
        data.push_back(tmp);
    }

    f_in.close();
    return data;
}

template<typename Tp>
cv::Mat HandEyeCalib::convertVector2Mat(std::vector<Tp> v, int channels, int rows) {
    cv::Mat mat = cv::Mat(v);//将vector变成单列的mat
    cv::Mat dest = mat.reshape(channels, rows).clone();//PS：必须clone()一份，否则返回出错
    return dest;
}

template<typename Tp>
std::vector<Tp> HandEyeCalib::convertMat2Vector(const cv::Mat &mat) {
    return (std::vector<Tp>) (mat.reshape(1, 1));//通道数不变，按行转为一行
}

int HandEyeCalib::run(){
    get_hand_eye_data();
    reform_hand_eye_data();
    //手眼标定，CALIB_HAND_EYE_TSAI法为11ms，最快
    cv::calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam,
                        t_target2cam, R_cam2gripper, t_cam2gripper,
                        cv::CALIB_HAND_EYE_TSAI);
    H_cam2gripper = r_t2rt(R_cam2gripper, t_cam2gripper);//矩阵合并

    //结果评估
    //用第一组和第二组进行对比验证
    std::cout << "第一组和第二组手眼数据验证：" << std::endl;
    std::cout << vecH_gripper2base[0] * H_cam2gripper * vecH_target2cam[0] << std::endl
                << vecH_gripper2base[1] * H_cam2gripper * vecH_target2cam[1] << std::endl << std::endl;//.inv()

    std::cout << "标定板在相机中的位姿：" << std::endl;
    std::cout << vecH_target2cam[1] << std::endl;
    std::cout << "手眼系统反演的位姿为：" << std::endl;
    //用手眼系统预测第一组数据中标定板相对相机的位姿，是否与vecH_target2cam[1]相同
    cv::Mat H_target2cam_ = H_cam2gripper.inv()* vecH_gripper2base[1].inv() * vecH_gripper2base[0]
                    * H_cam2gripper * vecH_target2cam[0];
    std::cout << H_target2cam_ << std::endl << std::endl;
    // 误差
    std::cout << "误差：" << std::endl;
    cv::Mat H_error = cv::abs(vecH_target2cam[1] - H_target2cam_);
    std::cout << H_error << std::endl << std::endl;

    std::cout << "----手眼系统测试----" << std::endl;
    std::cout << "机械臂下标定板XYZ为:" << std::endl;
    for (int i = 0; i < vecH_target2cam.size(); ++i)
    {
        cv::Mat cheesePos{ 0.0,0.0,0.0,1.0 };//4*1矩阵，单独求机械臂下，标定板的xyz
        cv::Mat worldPos = vecH_gripper2base[i] * H_cam2gripper * vecH_target2cam[i] * cheesePos;
        std::cout << i + 1 << ": " << worldPos.t() << std::endl;
    }
    std::cout << "手眼标定结果：" << H_cam2gripper << std::endl;
    Eigen::Matrix3d R_cam2gripper_eigen = Eigen::Matrix3d::Identity();
    cv::cv2eigen(R_cam2gripper, R_cam2gripper_eigen);
    Eigen::Quaterniond Q(R_cam2gripper_eigen);

    std::cout << "位移量: " << t_cam2gripper.at<double>(0, 0) << ","
                        << t_cam2gripper.at<double>(0, 1) << ","
                        << t_cam2gripper.at<double>(0, 2) << std::endl;
    
    std::cout << "四元数: " << Q.w() << ","
                            << Q.x() << ","
                            << Q.y() << ","
                            << Q.z() << std::endl;
    return 0;
}

