#pragma once

#include "common/types.hpp"
#include "communication/protocol.hpp"

#include <cmath>
#include <optional>
#include <stdexcept>

namespace srm
{

// 由外参和姿态对齐模块生成。
//
// 控制参考系必须约定为：
// X：零位朝前
// Y：向左
// Z：向上
//
// 目标点变换：
// p_control = R_control_camera * p_camera + t_control_camera

//相机坐标系到云台坐标系
struct CameraToControl
{
    cv::Matx33d rotation = cv::Matx33d::eye();
    cv::Vec3d translation_m{0, 0, 0};

    std::uint64_t frame_id = 0;

    bool valid = false;
};


//拍照这一瞬间，相机对于云台的T和R
class TransformProvider
{
public:
    virtual ~TransformProvider() = default;

    virtual std::optional<CameraToControl> for_frame(
        const CameraFrame& frame
    ) = 0;
};

// 上位机对零点，正负到下位机
struct AngleConvention
{
    bool confirmed = false;

    // 暂定
    double yaw_sign = 1.0;
    double pitch_sign = 1.0;

    double yaw_zero_deg = 0.0;
    double pitch_zero_deg = 0.0;
};

// 角度补偿
struct AimCorrection
{
    double yaw_deg = 0.0;
    double pitch_deg = 0.0;
};

class Aimer
{
public:
    explicit Aimer(AngleConvention convention)
        : convention_(convention)
    {
    }

    std::optional<Gimbal_Receive_s> aim(
        const ArmorPose& pose,
        const CameraToControl& transform,
        const AimCorrection& correction
    ) const
    {
        if (!convention_.confirmed || !transform.valid) {
            return std::nullopt;
        }

        if (std::abs(convention_.yaw_sign) != 1.0 ||
            std::abs(convention_.pitch_sign) != 1.0) {
            throw std::runtime_error("Angle signs must be +1 or -1");
        }

        const cv::Vec3d p =
            transform.rotation * pose.position_camera_m +
            transform.translation_m;

        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(p[i])) {
                return std::nullopt;
            }
        }

        const double horizontal = std::hypot(p[0], p[1]);

        if (horizontal < 1e-6) {
            return std::nullopt;
        }

        //算yaw
        const double yaw_deg =
            std::atan2(p[1], p[0]) * 180.0 / CV_PI;

        //算pitch
            const double pitch_deg =
            std::atan2(p[2], horizontal) * 180.0 / CV_PI;

        //云台yaw（正负号，零点偏移，静态补偿）
        const double mapped_yaw =
            convention_.yaw_sign * yaw_deg +
            convention_.yaw_zero_deg +
            correction.yaw_deg;

        //云台pitch（正负号，零点偏移，静态补偿）
        const double mapped_pitch =
            convention_.pitch_sign * pitch_deg +
            convention_.pitch_zero_deg +
            correction.pitch_deg;

        if (!std::isfinite(mapped_yaw) ||
            !std::isfinite(mapped_pitch)) {
            return std::nullopt;
        }

        Gimbal_Receive_s command;

        command.yaw = static_cast<float>(mapped_yaw);
        command.pitch = static_cast<float>(mapped_pitch);

        if (!std::isfinite(command.yaw) ||
            !std::isfinite(command.pitch)) {
            return std::nullopt;
        }

        return command;
    }

private:
    AngleConvention convention_;
};

}  // namespace srm