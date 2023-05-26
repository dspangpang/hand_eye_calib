
#include "../include/hand_eye_calib/HandEyeCalib.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>


void pub_camera_frame() {

    // 创建 tf2 的广播对象
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    // 创建 tf2 要广播的静态坐标变换
    geometry_msgs::TransformStamped static_transform_stamped;

    // 对坐标变换初始化
    static_transform_stamped.header.stamp = ros::Time::now();
    // 父节点
    static_transform_stamped.header.frame_id = "dummy_gripper";
    // 子节点
    static_transform_stamped.child_frame_id = "camera_clolr";

    static_transform_stamped.transform.rotation.w = 0.983093;
    static_transform_stamped.transform.rotation.x = 0.066061;
    static_transform_stamped.transform.rotation.y = -0.0473449;
    static_transform_stamped.transform.rotation.z = -0.164081;

    static_transform_stamped.transform.translation.x = -0.00342111369316337;
    static_transform_stamped.transform.translation.y = -0.1068635727676046;
    static_transform_stamped.transform.translation.z = -0.3041156208837277;

    static_broadcaster.sendTransform(static_transform_stamped);
}


int main(int argc, char** argv){

    ros::init(argc, argv, "pub_hand_eye_frame_node");
    ros::NodeHandle nh;

    pub_camera_frame();

    ros::spin();
    
    return 0;

}
