
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include "../include/hand_eye_calib/jsonParser.h"

std::string end_effort_frame; 
std::string camera_color_frame;
std::string camera_depth_frame;
std::string json_path;

Eigen::Vector3f eye_to_hand_translation;
Eigen::Quaternionf eye_to_hand_quaternion;
Eigen::Vector3f depth_to_color_translation;
Eigen::Quaternionf depth_to_color_quaternion;

void pub_camera_color_frame() {

    // 创建 tf2 的广播对象
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    // 创建 tf2 要广播的静态坐标变换
    geometry_msgs::TransformStamped static_transform_stamped;

    // 对坐标变换初始化
    static_transform_stamped.header.stamp = ros::Time::now();
    // 父节点
    static_transform_stamped.header.frame_id = end_effort_frame;
    // 子节点
    static_transform_stamped.child_frame_id = camera_color_frame;

    static_transform_stamped.transform.translation.x = eye_to_hand_translation.x();
    static_transform_stamped.transform.translation.y = eye_to_hand_translation.y();
    static_transform_stamped.transform.translation.z = eye_to_hand_translation.z();

    static_transform_stamped.transform.rotation.w = eye_to_hand_quaternion.w();
    static_transform_stamped.transform.rotation.x = eye_to_hand_quaternion.x();
    static_transform_stamped.transform.rotation.y = eye_to_hand_quaternion.y();
    static_transform_stamped.transform.rotation.z = eye_to_hand_quaternion.z();

    static_broadcaster.sendTransform(static_transform_stamped);
}


void pub_camera_depth_frame() {

    // 创建 tf2 的广播对象
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    // 创建 tf2 要广播的静态坐标变换
    geometry_msgs::TransformStamped static_transform_stamped;

    // 对坐标变换初始化
    static_transform_stamped.header.stamp = ros::Time::now();
    // 父节点
    static_transform_stamped.header.frame_id = camera_color_frame;
    // 子节点
    static_transform_stamped.child_frame_id = camera_depth_frame;

    static_transform_stamped.transform.translation.x = depth_to_color_translation.x();
    static_transform_stamped.transform.translation.y = depth_to_color_translation.y();
    static_transform_stamped.transform.translation.z = depth_to_color_translation.z();


    static_transform_stamped.transform.rotation.w = depth_to_color_quaternion.w();
    static_transform_stamped.transform.rotation.x = depth_to_color_quaternion.x();
    static_transform_stamped.transform.rotation.y = depth_to_color_quaternion.y();
    static_transform_stamped.transform.rotation.z = depth_to_color_quaternion.z();

    static_broadcaster.sendTransform(static_transform_stamped);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "pub_hand_eye_frame_node");
    ros::NodeHandle nh;

    json_path = ros::package::getPath("hand_eye_calib") + "/config/frame_pub_config.json";

    Eigen::MatrixXd eye_to_hand_matrix = parseJsonEigenMatrix(json_path, "extrinsics_eye_to_hand");
    Eigen::MatrixXd depth_to_color_matrix = parseJsonEigenMatrix(json_path, "extrinsics_depth_to_color");
    end_effort_frame = parseJsonString(json_path, "end_effort_frame");
    camera_color_frame = parseJsonString(json_path, "camera_color_frame");
    camera_depth_frame = parseJsonString(json_path, "camera_depth_frame");

    depth_to_color_translation.x() = depth_to_color_matrix(0, 3);
    depth_to_color_translation.y() = depth_to_color_matrix(1, 3);
    depth_to_color_translation.z() = depth_to_color_matrix(2, 3);


    // 取depth_to_color_matrix的前三行三列作为旋转矩阵
    Eigen::Matrix3f tmp_rotation_matrix;
    tmp_rotation_matrix << depth_to_color_matrix(0, 0), depth_to_color_matrix(0, 1), depth_to_color_matrix(0, 2),
                           depth_to_color_matrix(1, 0), depth_to_color_matrix(1, 1), depth_to_color_matrix(1, 2),
                           depth_to_color_matrix(2, 0), depth_to_color_matrix(2, 1), depth_to_color_matrix(2, 2);
    

    depth_to_color_quaternion = tmp_rotation_matrix;

    eye_to_hand_translation.x() = eye_to_hand_matrix(0, 0);
    eye_to_hand_translation.y() = eye_to_hand_matrix(0, 1);
    eye_to_hand_translation.z() = eye_to_hand_matrix(0, 2);

    eye_to_hand_quaternion.w() = eye_to_hand_matrix(0, 3);
    eye_to_hand_quaternion.x() = eye_to_hand_matrix(0, 4);
    eye_to_hand_quaternion.y() = eye_to_hand_matrix(0, 5);
    eye_to_hand_quaternion.z() = eye_to_hand_matrix(0, 6);
    
    pub_camera_color_frame();
    sleep(1);
    pub_camera_depth_frame();

    ros::spin();
    
    return 0;

}
