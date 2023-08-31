
#include "../include/hand_eye_calib/jsonParser.h"

std::string parseJsonFile(const std::string& file_path){
    
    // 读取json文件
    std::ifstream jsonFile(file_path);
    if (!jsonFile.is_open()) {
        std::cerr << "Failed to open JSON file." << std::endl;
        return "";
    }

    // 读取文件内容到字符串
    std::string jsonData((std::istreambuf_iterator<char>(jsonFile)),
                         std::istreambuf_iterator<char>());
    jsonFile.close();
    
    return jsonData;

}

Eigen::MatrixXd parseJsonEigenMatrix(const std::string& file_path, const std::string& key){
    
    std::string tmp_data = parseJsonFile(file_path);

    // 解析 JSON 字符串
    Json::CharReaderBuilder jsonReader;
    Json::Value root;
    JSONCPP_STRING errs;
    std::istringstream jsonStream(tmp_data);
    Json::parseFromStream(jsonReader, jsonStream, &root, &errs);

    // 获取 matrix 数组
    Json::Value matrixArray = root[key];
    
    // 确保 matrixArray 是数组
    if (!matrixArray.isArray()) {
        std::cerr << "Invalid matrix data in JSON." << std::endl;
        return Eigen::MatrixXd::Zero(0,0);
    }

    // 获取matrixArray的行数
    int rows = matrixArray.size();
    // 获取matrixArray的列数
    int cols = matrixArray[0].size();

    Eigen::MatrixXd matrix(rows, cols);

    // 读取并处理矩阵数据
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++){
            matrix(i, j) = matrixArray[i][j].asDouble();
        }
    }

    return matrix;
}

cv::Mat parseJsonCVMatrix(const std::string& file_path, const std::string& key){
    
    std::string tmp_data = parseJsonFile(file_path);

    // 解析 JSON 字符串
    Json::CharReaderBuilder jsonReader;
    Json::Value root;
    JSONCPP_STRING errs;
    std::istringstream jsonStream(tmp_data);
    Json::parseFromStream(jsonReader, jsonStream, &root, &errs);

    // 获取 matrix 数组
    Json::Value matrixArray = root[key];
    
    // 确保 matrixArray 是数组
    if (!matrixArray.isArray()) {
        std::cerr << "Invalid matrix data in JSON." << std::endl;
        return cv::Mat::zeros(0,0,CV_64FC1);
    }

    // 获取matrixArray的行数
    int rows = matrixArray.size();
    // 获取matrixArray的列数
    int cols = matrixArray[0].size();

    cv::Mat matrix = cv::Mat::zeros(rows, cols, CV_64FC1);

    // 读取并处理矩阵数据
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++){
            matrix.at<double>(i, j) = matrixArray[i][j].asDouble();
        }
    }

    return matrix;
}


double parseJsonDouble(const std::string& file_path, const std::string& key){
    
    std::string tmp_data = parseJsonFile(file_path);

    // 解析 JSON 字符串
    Json::CharReaderBuilder jsonReader;
    Json::Value root;
    JSONCPP_STRING errs;
    std::istringstream jsonStream(tmp_data);
    Json::parseFromStream(jsonReader, jsonStream, &root, &errs);

    // 读取 double 数据
    if (root.isMember(key) && root[key].isDouble()) {
        double value = root[key].asDouble();
        return value;
    } else {
        std::cout << "parse Json double WRONG!!" << std::endl;
        return 0;
    }
}


int parseJsonInt(const std::string& file_path, const std::string& key){
    
    std::string tmp_data = parseJsonFile(file_path);

    // 解析 JSON 字符串
    Json::CharReaderBuilder jsonReader;
    Json::Value root;
    JSONCPP_STRING errs;
    std::istringstream jsonStream(tmp_data);
    Json::parseFromStream(jsonReader, jsonStream, &root, &errs);

    // 读取 int 数据
    if (root.isMember(key) && root[key].isInt()) {
        int value = root[key].asInt();
        return value;
    } else {
        std::cout << "parse Json int WRONG!!" << std::endl;
        return 0;
    }
}


bool parseJsonBool(const std::string& file_path, const std::string& key){
    
    std::string tmp_data = parseJsonFile(file_path);

    // 解析 JSON 字符串
    Json::CharReaderBuilder jsonReader;
    Json::Value root;
    JSONCPP_STRING errs;
    std::istringstream jsonStream(tmp_data);
    Json::parseFromStream(jsonReader, jsonStream, &root, &errs);

    // 读取 int 数据
    if (root.isMember(key) && root[key].isBool()) {
        int value = root[key].asBool();
        return value;
    } else {
        std::cout << "parse Json int WRONG!!" << std::endl;
        return 0;
    }
}


std::string parseJsonString(const std::string& file_path, const std::string& key){
    
    std::string tmp_data = parseJsonFile(file_path);

    // 解析 JSON 字符串
    Json::CharReaderBuilder jsonReader;
    Json::Value root;
    JSONCPP_STRING errs;
    std::istringstream jsonStream(tmp_data);
    Json::parseFromStream(jsonReader, jsonStream, &root, &errs);

    // 读取 double 数据
    if (root.isMember(key) && root[key].isString()) {
        std::string value = root[key].asString();
        return value;
    } else {
        std::cout << "parse Json String WRONG!!" << std::endl;
        return "";
    }
}