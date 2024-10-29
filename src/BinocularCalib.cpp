#include "../include/hand_eye_calib/BinocularCalib.h"

BinocularCalib::BinocularCalib(
    const std::string &config_path,
    const std::string &calib_board_type) 
{
    this->config_path = config_path;
    if (calib_board_type == "checkerboard") {
        this->json_path = config_path + "/binocular_calib_check.json";
    }
    std::cout << "jsonPath : " << json_path << std::endl;
    get_calib_config_data(json_path);
}

BinocularCalib::~BinocularCalib() = default;

void BinocularCalib::run() {
    setup_calibration();
}

void BinocularCalib::get_calib_config_data(const std::string &json_path) {
    std::string const_scale = parseJsonString(json_path, "const_scale");
    if (const_scale == "mm") {
        this->scale = 1.0;
    } else if (const_scale == "cm") {
        this->scale = 10.0;
    } else if (const_scale == "m") {
        this->scale = 1000.0;
    } else {
        std::cerr << "Invalid scale unit" << std::endl;
        return;
    } 
    this->imagesNum = parseJsonInt(json_path, "imagesNum");
    this->image_dir_1 = parseJsonString(json_path, "image_1_dir");
    this->image_dir_2 = parseJsonString(json_path, "image_2_dir");
    this->board_squre_num = std::make_pair(parseJsonInt(json_path, "board_squre_num_x"), parseJsonInt(json_path, "board_squre_num_y"));
    this->square_size = parseJsonDouble(json_path, "square_size") * this->scale;
    this->visulation = parseJsonBool(json_path, "visulation");
}


int BinocularCalib::setup_calibration(){

    std::vector<cv::Mat> imgs_1(imagesNum);
    std::vector<cv::Mat> imgs_2(imagesNum);
    // 读取图片
    for (int i = 0; i < imagesNum; i++) {
        std::string img_1_path = image_dir_1 + "/" + std::to_string(i) + ".bmp";
        std::cout << "img_path : " << img_1_path << std::endl;
        imgs_1[i] = cv::imread(img_1_path);
        if (imgs_1[i].empty()) {
            std::cerr << "Can't read image " << img_1_path << std::endl;
            return -1;
        }
    }
    for (int i = 0; i < imagesNum; i++) {
        std::string img_2_path = image_dir_2 + "/" + std::to_string(i) + ".bmp";
        std::cout << "img_path : " << img_2_path << std::endl;
        imgs_2[i] = cv::imread(img_2_path);
        if (imgs_2[i].empty()) {
            std::cerr << "Can't read image " << img_2_path << std::endl;
            return -1;
        }

    }

    // 从图片中提取角点
    std::vector<std::vector<cv::Point2f>> image_points_1;
    std::vector<std::vector<cv::Point2f>> image_points_2;
    std::vector<std::vector<cv::Point3f>> object_points;
    cv::Size board_size(board_squre_num.first, board_squre_num.second);

    // 生成标定板的三维坐标
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < board_size.height; i++) {
        for (int j = 0; j < board_size.width; j++) {
            obj.push_back(cv::Point3f(j * square_size, i * square_size, 0));
        }
    }

    int flag = cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY;
    for (int i = 0; i < imagesNum; i++) {
        std::vector<cv::Point2f> corners_1;
        std::vector<cv::Point2f> corners_2;
        // Accurate Detection and Localization of Checkerboard Corners for Calibration(2018)
        bool found_1 = cv::findChessboardCornersSB(imgs_1[i], board_size, corners_1, flag);
        bool found_2 = cv::findChessboardCornersSB(imgs_2[i], board_size, corners_2, flag);

        if(!found_1 || !found_2){
            std::cout << "Chessboard find error!" << "imagesNum: " << i << std::endl;
            continue;
        }

        if (this->visulation)
        {
            // 绘制显示结果
            cv::Mat img_1 = imgs_1[i].clone();
            cv::drawChessboardCorners(img_1, board_size, corners_1, found_1);
            cv::Mat img_2 = imgs_2[i].clone();
            cv::drawChessboardCorners(img_2, board_size, corners_2, found_1);
            cv::namedWindow("coners1", cv::WINDOW_NORMAL);
            cv::resizeWindow("coners1", 1280, 800);
            cv::namedWindow("coners2", cv::WINDOW_NORMAL);
            cv::resizeWindow("coners2", 1280, 800);    
            cv::imshow("coners1", img_1);
            cv::imshow("coners2", img_2);
            cv::waitKey(0);
            cv::destroyWindow("coners1");
            cv::destroyWindow("coners2");
        }

        cv::Mat gray_1;
        cv::cvtColor(imgs_1[i], gray_1, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(gray_1, corners_1, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
        image_points_1.push_back(corners_1);

        cv::Mat gray_2;
        cv::cvtColor(imgs_2[i], gray_2, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(gray_2, corners_2, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
        image_points_2.push_back(corners_2);

        object_points.push_back(obj);
    }

    // 双目相机标定
    cv::Mat camera_matrix_1, dist_coeffs_1;
    cv::Mat camera_matrix_2, dist_coeffs_2;

    // 首先标定每个相机的内参和畸变参数
    cv::calibrateCamera(
        object_points, 
        image_points_1, 
        imgs_1[0].size(), 
        camera_matrix_1, 
        dist_coeffs_1, 
        cv::noArray(), 
        cv::noArray());
    
    cv::calibrateCamera(
        object_points, 
        image_points_2, 
        imgs_2[0].size(), 
        camera_matrix_2, 
        dist_coeffs_2, 
        cv::noArray(), 
        cv::noArray());
    

    cv::Mat R, T;

    std::vector<cv::Mat> rvecs, tvecs;
    cv::stereoCalibrate(
        object_points, 
        image_points_1, 
        image_points_2, 
        camera_matrix_1,
        dist_coeffs_1, 
        camera_matrix_2, 
        dist_coeffs_2, 
        imgs_1[0].size(), 
        R, 
        T, 
        cv::noArray(), 
        cv::noArray(),
        cv::CALIB_FIX_INTRINSIC);

    // 输出标定结果
    std::cout << "camera_matrix_1 : " << camera_matrix_1 << std::endl;
    std::cout << "dist_coeffs_1 : " << dist_coeffs_1 << std::endl;
    std::cout << "camera_matrix_2 : " << camera_matrix_2 << std::endl;
    std::cout << "dist_coeffs_2 : " << dist_coeffs_2 << std::endl;
    std::cout << "R : " << R << std::endl;
    std::cout << "T : " << T << std::endl;

    // 双目相机校正
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(
        camera_matrix_1, 
        dist_coeffs_1, 
        camera_matrix_2, 
        dist_coeffs_2, 
        imgs_1[0].size(), 
        R, 
        T, 
        R1, 
        R2, 
        P1, 
        P2, 
        Q);
    
    // 显示校正后的图片
    if (visulation) {
        cv::Mat map1x, map1y, map2x, map2y;
        cv::initUndistortRectifyMap(
            camera_matrix_1, 
            dist_coeffs_1, 
            R1, 
            P1, 
            imgs_1[0].size(), 
            CV_32FC1, 
            map1x, 
            map1y);
        cv::initUndistortRectifyMap(
            camera_matrix_2, 
            dist_coeffs_2, 
            R2, 
            P2, 
            imgs_2[0].size(), 
            CV_32FC1, 
            map2x, 
            map2y);

        for (int i = 0; i < imagesNum; i++) {
            cv::Mat rectify_img_1, rectify_img_2;
            cv::remap(imgs_1[i], rectify_img_1, map1x, map1y, cv::INTER_LINEAR);
            cv::remap(imgs_2[i], rectify_img_2, map2x, map2y, cv::INTER_LINEAR);

            cv::namedWindow("rectify_img_1", cv::WINDOW_NORMAL);
            cv::resizeWindow("rectify_img_1", 1280, 800);
            cv::namedWindow("rectify_img_2", cv::WINDOW_NORMAL);
            cv::resizeWindow("rectify_img_2", 1280, 800);   
            cv::imshow("rectify_img_1", imgs_1[i]);
            cv::imshow("rectify_img_2", imgs_2[i]);
            cv::waitKey(0);
            cv::destroyWindow("rectify_img_1");
            cv::destroyWindow("rectify_img_2");
        }
    }

    // 保存标定结果
    std::string res_file = config_path + "/../output/BinocularCalib.yml";
    cv::FileStorage fs(res_file, cv::FileStorage::WRITE);
    fs << "camera_matrix_1" << camera_matrix_1;
    fs << "dist_coeffs_1" << dist_coeffs_1;
    fs << "camera_matrix_2" << camera_matrix_2;
    fs << "dist_coeffs_2" << dist_coeffs_2;
    fs << "R" << R;
    fs << "T" << T;
    // 保存齐次变换矩阵
    cv::Mat H = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(H(cv::Rect(0, 0, 3, 3)));
    T.copyTo(H(cv::Rect(3, 0, 1, 3)));
    fs << "H" << H;

    // 关闭文件
    fs.release();

    return 0;
}

