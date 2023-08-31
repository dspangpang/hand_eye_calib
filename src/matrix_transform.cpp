#include "../include/hand_eye_calib/matrix_transform.h"

cv::Mat r_t2rt(cv::Mat &R, cv::Mat &T){
    cv::Mat RT;
    cv::Mat_<double> R1 = (cv::Mat_<double>(4, 3) <<
            R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
            0.0, 0.0, 0.0);

    cv::Mat_<double> T1 = (cv::Mat_<double>(4, 1) <<
            T.at<double>(0, 0),
            T.at<double>(1, 0),
            T.at<double>(2, 0),
            1.0);

    cv::hconcat(R1, T1, RT);

    return RT;
}


void rt2r_t(cv::Mat &RT, cv::Mat &R, cv::Mat &T){
    cv::Rect R_rect(0, 0, 3, 3);
    cv::Rect T_rect(3, 0, 1, 3);
    R = RT(R_rect);
    T = RT(T_rect);
}


bool is_rotation_matrix(const cv::Mat & R){
    cv::Mat tmp33 = R({ 0,0,3,3 });
    cv::Mat is_identity;

    is_identity = tmp33.t() * tmp33;

    cv::Mat I = cv::Mat::eye(3, 3, is_identity.type());

    return  cv::norm(I, is_identity) < 1e-6;
}


cv::Mat euler_angle_to_rotated_matrix(const cv::Mat& euler_angle, const std::string& seq){
        CV_Assert(euler_angle.rows == 1 && euler_angle.cols == 3);

    euler_angle /= 180 / CV_PI;
    cv::Matx13d m(euler_angle);
    auto rx = m(0, 0), ry = m(0, 1), rz = m(0, 2);
    auto xs = std::sin(rx), xc = std::cos(rx);
    auto ys = std::sin(ry), yc = std::cos(ry);
    auto zs = std::sin(rz), zc = std::cos(rz);

    cv::Mat rot_x = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, xc, -xs, 0, xs, xc);
    cv::Mat rot_y = (cv::Mat_<double>(3, 3) << yc, 0, ys, 0, 1, 0, -ys, 0, yc);
    cv::Mat rot_z = (cv::Mat_<double>(3, 3) << zc, -zs, 0, zs, zc, 0, 0, 0, 1);

    cv::Mat rotMat;

    if (seq == "zyx")		rotMat = rot_x * rot_y * rot_z;
    else if (seq == "yzx")	rotMat = rot_x * rot_z * rot_y;
    else if (seq == "zxy")	rotMat = rot_y * rot_x * rot_z;
    else if (seq == "xzy")	rotMat = rot_y * rot_z * rot_x;
    else if (seq == "yxz")	rotMat = rot_z * rot_x * rot_y;
    else if (seq == "xyz")	rotMat = rot_z * rot_y * rot_x;
    else {
        cv::error(cv::Error::StsAssert, "Euler angle sequence std::string is wrong.",
                  __FUNCTION__, __FILE__, __LINE__);
    }

    if (!is_rotation_matrix(rotMat)) {
        cv::error(cv::Error::StsAssert, "Euler angle can not convert to rotated matrix",
                  __FUNCTION__, __FILE__, __LINE__);
    }

    return rotMat;
}


cv::Mat quaternion_to_rotated_matrix(const cv::Vec4d& q){
    
    double w = q[0], x = q[1], y = q[2], z = q[3];

    double x2 = x * x, y2 = y * y, z2 = z * z;
    double xy = x * y, xz = x * z, yz = y * z;
    double wx = w * x, wy = w * y, wz = w * z;

    cv::Matx33d res{
            1 - 2 * (y2 + z2),	2 * (xy - wz),		2 * (xz + wy),
            2 * (xy + wz),		1 - 2 * (x2 + z2),	2 * (yz - wx),
            2 * (xz - wy),		2 * (yz + wx),		1 - 2 * (x2 + y2),
    };
    return cv::Mat(res);
}


cv::Mat attitude_vector_to_Matrix(cv::Mat& m, bool is_quaternion, const std::string& seq){
    CV_Assert(m.total() == 6 || m.total() == 10 || m.total() == 7);
    if (m.cols == 1)
        m = m.t();
    cv::Mat tmp = cv::Mat::eye(4, 4, CV_64FC1);

    //如果使用四元数转换成旋转矩阵则读取m矩阵的第四个成员，读4个数据
    if (is_quaternion)	// normalized vector, its norm should be 1.
    {
        cv::Vec4d quaternionVec = m({ 3, 0, 4, 1 });
        quaternion_to_rotated_matrix(quaternionVec).copyTo(tmp({ 0, 0, 3, 3 }));
        // std::cout << norm(quaternionVec) << std:endl;
    }
    else
    {
        cv::Mat rotVec;
        if (m.total() == 6)
            rotVec = m({ 3, 0, 3, 1 });		//6
        else
            rotVec = m({ 7, 0, 3, 1 });		//10

        //如果seq为空表示传入的是旋转向量，否则"xyz"的组合表示欧拉角
        if (seq.empty())
            cv::Rodrigues(rotVec, tmp({ 0, 0, 3, 3 }));
        else
            euler_angle_to_rotated_matrix(rotVec, seq).copyTo(tmp({ 0, 0, 3, 3 }));
    }
    tmp({ 3, 0, 1, 3 }) = m({ 0, 0, 3, 1 }).t();

    return tmp;
}
