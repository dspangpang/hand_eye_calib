#include "../include/hand_eye_calib/MonocularCalib.h"

MonocularCalib::MonocularCalib(
    const std::string &config_path,
    const std::string &calib_board_type) 
{
    this->config_path = config_path;
    if (calib_board_type == "checkerboard"){
        this->json_path = config_path + "/monocular_calib_check.json";
    }
    std::cout << "jsonPath : " << json_path << std::endl;
    get_calib_config_data(json_path);
}

MonocularCalib::~MonocularCalib() = default;

void MonocularCalib::run() {
    setup_calibration();
}

void MonocularCalib::get_calib_config_data(const std::string &json_path) {
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
    this->image_dir = parseJsonString(json_path, "image_dir");
    this->board_squre_num = std::make_pair(parseJsonInt(json_path, "board_squre_num_x"), parseJsonInt(json_path, "board_squre_num_y"));
    this->square_size = parseJsonDouble(json_path, "square_size") * this->scale;
    this->visulation = parseJsonBool(json_path, "visulation");
}


int MonocularCalib::setup_calibration(){

    std::vector<cv::Mat> imgs(imagesNum);
    // 读取图片
    for (int i = 0; i < imagesNum; i++) {
        std::string img_path = image_dir + "/" + std::to_string(i) + ".bmp";
        std::cout << "img_path : " << img_path << std::endl;
        imgs[i] = cv::imread(img_path);
        if (imgs[i].empty()) {
            std::cerr << "Can't read image " << img_path << std::endl;
            return -1;
        }
    }

    // 从图片中提取角点
    std::vector<std::vector<cv::Point2f>> image_points;
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
        std::vector<cv::Point2f> corners;
        // Accurate Detection and Localization of Checkerboard Corners for Calibration(2018)
        bool found = cv::findChessboardCornersSB(imgs[i], board_size, corners, flag);
        if (!found) {
            std::cerr << "Can't find chessboard corners in image " << i << std::endl;
            continue;
        }else{
            if (this->visulation)
            {
                // 绘制显示结果
                cv::Mat img = imgs[i].clone();
                cv::drawChessboardCorners(img, board_size, corners, found);
                cv::namedWindow("coners", cv::WINDOW_NORMAL);
                cv::resizeWindow("coners", 1280, 800);  
                cv::imshow("coners", img);
                cv::waitKey(0);
                cv::destroyWindow("coners");
            }
        }
        cv::Mat gray;
        cv::cvtColor(imgs[i], gray, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
        image_points.push_back(corners);
        object_points.push_back(obj);
    }

    // 相机标定
    cv::Mat camera_matrix, dist_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    cv::calibrateCamera(object_points, image_points, imgs[0].size(), camera_matrix, dist_coeffs, rvecs, tvecs);

    // 保存标定结果
    std::string camera_matrix_path = image_dir + "/camera_matrix.txt";
    std::string dist_coeffs_path = image_dir + "/dist_coeffs.txt";
    std::ofstream ofs_camera(camera_matrix_path);
    std::ofstream ofs_dist(dist_coeffs_path);

    // 假设 camera_matrix 和 dist_coeffs 是 cv::Mat 类型
    for (int i = 0; i < camera_matrix.rows; i++) {
        for (int j = 0; j < camera_matrix.cols; j++) {
            ofs_camera << camera_matrix.at<double>(i, j);
            if (j < camera_matrix.cols - 1) {
                ofs_camera << " ";
            }
        }
        ofs_camera << std::endl;
    }
    ofs_camera.close();

    for (int i = 0; i < dist_coeffs.rows; i++) {
        for (int j = 0; j < dist_coeffs.cols; j++) {
            ofs_dist << dist_coeffs.at<double>(i, j);
            if (j < dist_coeffs.cols - 1) {
                ofs_dist << " ";
            }
        }
        ofs_dist << std::endl;
    }
    ofs_dist.close();

    // 把 rvecs 和 tvecs 保存到文件 每行代表每个图片的 rvecs 和 tvecs
    std::string vecs_path = image_dir + "/vecs.txt";
    std::ofstream ofs(vecs_path);
    
    double local_scale = this->scale / 1000.0;

    for (int i = 0; i < imagesNum; i++) {
        ofs << rvecs[i].at<double>(0) << " " << rvecs[i].at<double>(1) << " " << rvecs[i].at<double>(2);
        ofs << " " << tvecs[i].at<double>(0) * local_scale << " " << tvecs[i].at<double>(1) * local_scale 
        << " " << tvecs[i].at<double>(2) * local_scale << std::endl;
    }
    ofs.close();


    std::cout << "camera_matrix : " << camera_matrix << std::endl;
    std::cout << "dist_coeffs : " << dist_coeffs << std::endl;

    // 计算重投影误差
    double total_err = 0.0;
    double err;
    std::vector<cv::Point2f> image_points2;
    for (int i = 0; i < imagesNum; i++) {
        cv::projectPoints(object_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs, image_points2);
        err = cv::norm(image_points[i], image_points2, cv::NORM_L2);
        total_err += err /= object_points[i].size();
        std::cout << "image " << i << " error : " << err << std::endl;
    }
    std::cout << "total error : " << total_err << std::endl;

    // 保存标定结果
    std::string res_file = config_path + "/../output/MonocularCalib.yml";
    cv::FileStorage fs(res_file, cv::FileStorage::WRITE);
    fs << "camera_matrix" << camera_matrix;
    fs << "dist_coeffs" << dist_coeffs;
    fs.release();

    return 0;
}

