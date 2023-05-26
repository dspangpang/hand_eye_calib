//
// Created by jzz on 23-2-8.
//

#include "../include/hand_eye_calib/HandEyeCalib.h"

HandEyeCalib::HandEyeCalib() {
    is_quaternion = false;
    is_picture = false;
}

HandEyeCalib::~HandEyeCalib() = default;

std::string HandEyeCalib::get_INI_data(const std::string &path,
                                       const std::string &root,
                                       const std::string &key) {
    auto *p = new INIParser();
    p->ReadINI(path);
    auto data = p->GetValue(root, key);
    delete p;
    return data;
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

void HandEyeCalib::get_data(const std::string &INI_dir) {
    std::string root = "CalibData";
    std::string key_cal = "calPoseData";
    std::string key_gripper = "gripperPoseData";
    std::string key_image_num = "imagesNum";
    std::string pose_format = "poseFormat";
    std::string cal_info = "isPicture";

    auto cal_pose_path = get_INI_data(INI_dir + "/Setting.ini", root, key_cal);
    auto gripper_pose_path = get_INI_data(INI_dir + "/Setting.ini", root, key_gripper);
    auto image_num = get_INI_data(INI_dir + "/Setting.ini", root, key_image_num);
    auto arm_format = get_INI_data(INI_dir + "/Setting.ini", root, pose_format);
    auto is_use_picture = get_INI_data(INI_dir + "/Setting.ini", root, cal_info);

    if(arm_format == "quaternion"){
        is_quaternion = true;
    }

    if(is_use_picture == "true"){
        is_picture = true;
    }

    std::cout << "pose format :" << arm_format << std::endl;

    num_images = std::stoi(image_num);
    auto cal_pose = get_TXT_data(INI_dir + "/" + cal_pose_path);
    auto gripper_pose = get_TXT_data(INI_dir + "/" + gripper_pose_path);


    CalPose = convertVector2Mat<double>(cal_pose, 1, int(num_images));
    ToolPose = convertVector2Mat<double>(gripper_pose, 1, int(num_images));

}

void HandEyeCalib::reform_data_hand_in_eye() {
    cv::Mat tempR, tempT;
    for (size_t i = 0; i < num_images; i++)//计算标定板位姿
    {
        cv::Mat tmp;
        cv::Mat board_pose;
        cv::Mat end_pose;
        board_pose = CalPose.row(int(i)).clone();
        end_pose = ToolPose.row(int(i)).clone();

        tmp = matrix_transform::attitude_vector_to_Matrix(board_pose, false, ""); //转移向量转旋转矩阵
        vecH_target2cam.push_back(tmp);
        matrix_transform::rt2r_t(tmp, tempR, tempT);
        R_target2cam.push_back(tempR);
        t_target2cam.push_back(tempT);

        tmp = matrix_transform::attitude_vector_to_Matrix(end_pose, is_quaternion, "xyz");
        vecH_gripper2base.push_back(tmp);
        matrix_transform::rt2r_t(tmp, tempR, tempT);
        R_gripper2base.push_back(tempR);
        t_gripper2base.push_back(tempT);
    }
}

void HandEyeCalib::reform_data_hand_to_eye() {
    cv::Mat tempR, tempT;
    for (size_t i = 0; i < num_images; i++)//计算标定板位姿
    {
        cv::Mat tmp;
        cv::Mat board_pose;
        cv::Mat end_pose;
        board_pose = CalPose.row(int(i)).clone();
        end_pose = ToolPose.row(int(i)).clone();

        tmp = matrix_transform::attitude_vector_to_Matrix(board_pose, false, ""); //转移向量转旋转矩阵
        vecH_target2cam.push_back(tmp);
        matrix_transform::rt2r_t(tmp, tempR, tempT);
        R_target2cam.push_back(tempR);
        t_target2cam.push_back(tempT);

        tmp = matrix_transform::attitude_vector_to_Matrix(end_pose, is_quaternion, "xyz");
        tmp = tmp.inv();
        vecH_base2gripper.push_back(tmp);
        matrix_transform::rt2r_t(tmp, tempR, tempT);
        R_base2gripper.push_back(tempR);
        t_base2gripper.push_back(tempT);
    }
}

int HandEyeCalib::hand_eye_calib(const std::string &INI_dir, const int calib_type) {
    get_data(INI_dir);
    if (calib_type == EYE_IN_HAND) {
        reform_data_hand_in_eye();
        //手眼标定，CALIB_HAND_EYE_TSAI法为11ms，最快
        cv::calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam,
                             t_target2cam, R_cam2gripper, t_cam2gripper,
                             cv::CALIB_HAND_EYE_TSAI);
        H_cam2gripper = matrix_transform::r_t2rt(R_cam2gripper, t_cam2gripper);//矩阵合并
    }
    if (calib_type == EYE_TO_HAND) {
        reform_data_hand_to_eye();
        //手眼标定，CALIB_HAND_EYE_TSAI法为11ms，最快
        cv::calibrateHandEye(R_base2gripper, t_base2gripper, R_target2cam,
                             t_target2cam, R_cam2base, t_cam2base,
                             cv::CALIB_HAND_EYE_TSAI);
        H_cam2base = matrix_transform::r_t2rt(R_cam2base, t_cam2base);//矩阵合并
    }
    return 0;
}

int HandEyeCalib::result_test(int calib_type) {

    if (calib_type == EYE_IN_HAND){
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
        matrix_transform::rotated_matrix_to_quaternion(R_cam2gripper);
    }
    if (calib_type == EYE_TO_HAND){
        //用第一组和第二组进行对比验证
        std::cout << "第一组和第二组手眼数据验证：" << std::endl;
        std::cout << vecH_base2gripper[0] * H_cam2base * vecH_target2cam[0] << std::endl
                  << vecH_base2gripper[1] * H_cam2base * vecH_target2cam[1] << std::endl << std::endl;

        std::cout << "标定板在相机中的位姿：" << std::endl;
        std::cout << vecH_target2cam[1] << std::endl;
        std::cout << "手眼系统反演的位姿为：" << std::endl;
        //用手眼系统预测第一组数据中标定板相对相机的位姿，是否与vecHc[1]相同
        cv::Mat H_target2cam_ = H_cam2gripper.inv()* vecH_gripper2base[1].inv() * vecH_gripper2base[0]
                     * H_cam2gripper * vecH_target2cam[0];
        std::cout << H_target2cam_ << std::endl << std::endl;
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
            cv::Mat worldPos = H_cam2base * vecH_target2cam[i] * cheesePos;
            std::cout << i + 1 << ": " << worldPos.t() << std::endl;
        }
    }
    getchar();
    return 0;
}

std::vector<double> HandEyeCalib::get_cal_pnp_data(const std::string &path){
    
    std::string fileName;
    std::vector<double> out;
    for (int i = 0; i < num_images; i++) {
        fileName = path + "/img/" + std::to_string(i) + ".bmp";
        auto img = cv::imread(fileName, cv::IMREAD_UNCHANGED);
        std::vector<cv::Point2d> img_point;

        auto found = cv::findChessboardCornersSB(img, cv::Size(17, 15), img_point,
			cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY);

        if (found) {
			cv::drawChessboardCorners(img, cv::Size(17, 15), cv::Mat(img_point), found);
            cv::namedWindow("image", cv::WINDOW_NORMAL);
		    cv::imshow("image", img);
		    auto key = cv::waitKey();
        }

    }

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
