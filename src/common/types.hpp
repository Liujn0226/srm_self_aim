#pragma once

#include <opencv2/core.hpp>

#include <array>
#include <chrono>
#include <cstdint>

namespace srm
{

using Clock = std::chrono::steady_clock;

// 存储一张图像本身，时间戳，第几张
struct CameraFrame
{
    cv::Mat bgr;

    Clock::time_point received_at{};

    std::uint64_t frame_id = 0;
};

// 装甲板四个角，置信度，类别
struct ArmorDetection
{
    std::array<cv::Point2f, 4> points{};

    float confidence = 0.0f;

    int class_id = -1;
};

// PnP解算
struct ArmorPose
{
    //对方装甲板的旋转向量
    cv::Vec3d rvec{0, 0, 0};
    //装甲板在相机坐标系下的坐标
    cv::Vec3d position_camera_m{0, 0, 0};
    //重投影误差
    double reprojection_error_px = 0.0;
};

}  // namespace srm