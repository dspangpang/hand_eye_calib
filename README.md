# hand_eye_calib

## 手眼标定功能

1. 在config文件夹中打开数据信息配置文件[calib_config.json](config/calib_config.json)
2. * calibrationMethod 对应手眼标定的种类(暂时只有EYE_IN_HAND)
   * imagesNum 拍摄标定板的图片的数目
   * dataDectory 放置标定数据的文件夹
   * armPoseFormat 机械臂末端位姿的存储格式
   * calPoseDataFile 相机拍摄标定板数据的文件名
   * gripperPoseDataFile 对应机械臂示教器对应机械臂末端位姿数据文件名
   * usePicture 是否使用Image文件夹中的图通过相机拍摄标定板的图片解算位姿(未完成)
   * imageDataDectory 图片文件夹的名称

## 发布标定后坐标系功能
1. 在config文件夹中打开数据信息配置文件[frame_pub_config.json](config/frame_pub_config.json)
2. * extrinsics_depth_to_color 相机深度图到彩色图的外参
   * extrinsics_eye_to_hand 手眼标定的结果
   * camera_color_frame 要发布相机彩色图的坐标系名称
   * camera_depth_frame 要发布的深度图的坐标系名称
   * end_effort_frame 机械臂末端的坐标系名称

# 原理参考链接
1. https://blog.csdn.net/m0_52785249/article/details/126094330?spm=1001.2101.3001.6661.1&utm_medium=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-126094330-blog-83960141.pc_relevant_multi_platform_whitelistv3&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-126094330-blog-83960141.pc_relevant_multi_platform_whitelistv3&utm_relevant_index=1#t5
2. https://www.bilibili.com/video/BV1By4y1b7Q7/?spm_id_from=333.337.search-card.all.click&vd_source=34057f0f5191f4e39071bc7170f9fe7a
3. https://www.bilibili.com/video/BV1ey4y1b76c/?spm_id_from=333.337.search-card.all.click&vd_source=34057f0f5191f4e39071bc7170f9fe7a

