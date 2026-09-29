#pragma once

#include "aiming/aimer.hpp"
#include "aiming/pose_solver.hpp"
#include "detection/armor_detector.hpp"

#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

namespace srm
{

//打包装甲信息，装甲位姿，云台指令，射击指令，当前帧号
struct AimResult
{
    ArmorDetection armor;
    ArmorPose pose;

    Gimbal_Receive_s gimbal;
    Shoot_Receive_s shoot;

    std::uint64_t frame_id = 0;
};

class Pipeline
{
public:
    Pipeline(
        ArmorDetector& detector,
        PoseSolver& solver,
        TransformProvider& transforms,
        Aimer& aimer,
        float minimum_confidence,
        double maximum_age_ms,
        double association_distance_px
    )
        : detector_(detector),
          solver_(solver),
          transforms_(transforms),
          aimer_(aimer),
          minimum_confidence_(minimum_confidence),
          maximum_age_ms_(maximum_age_ms),
          association_distance_px_(association_distance_px)
    {
        if (!std::isfinite(minimum_confidence_) ||
            minimum_confidence_ < 0 ||
            minimum_confidence_ > 1 ||
            !std::isfinite(maximum_age_ms_) ||
            maximum_age_ms_ <= 0 ||
            !std::isfinite(association_distance_px_) ||
            association_distance_px_ <= 0) {
            throw std::invalid_argument("Invalid pipeline parameters");
        }
    }

    std::optional<AimResult> process(
        const CameraFrame& frame,
        const AimCorrection& correction
    )
    {
        if (frame.bgr.empty() || expired(frame)) {
            previous_center_.reset();
            return std::nullopt;
        }

        const auto detections = detector_.detect(frame.bgr);

        const cv::Point2f reference =
            previous_center_.value_or(
                cv::Point2f(
                    frame.bgr.cols * 0.5f,
                    frame.bgr.rows * 0.5f
                )
            );

        const ArmorDetection* selected = nullptr;
        cv::Point2f selected_center;

        double best_distance =
            std::numeric_limits<double>::infinity();

        for (const auto& armor : detections) {
            if (!std::isfinite(armor.confidence) ||
                armor.confidence < minimum_confidence_) {
                continue;
            }

            cv::Point2f center(0, 0);
            bool finite = true;

            for (const auto& point : armor.points) {
                if (!std::isfinite(point.x) ||
                    !std::isfinite(point.y)) {
                    finite = false;
                    break;
                }

                center += point;
            }

            if (!finite) {
                continue;
            }

            center *= 0.25f;

            const double distance = cv::norm(center - reference);

            // 计算两个中心点像素距离，防止检测到远处装甲板
            if (previous_center_ &&
                distance > association_distance_px_) {
                continue;
            }

            if (distance < best_distance) {
                best_distance = distance;
                selected = &armor;
                selected_center = center;
            }
        }

        if (selected == nullptr) {
            previous_center_.reset();
            return std::nullopt;
        }

        const auto pose =
            solver_.solve(*selected, frame.bgr.size());

        if (!pose) {
            previous_center_.reset();
            return std::nullopt;
        }

        const auto transform = transforms_.for_frame(frame);

        if (!transform ||
            !transform->valid ||
            transform->frame_id != frame.frame_id) {
            previous_center_.reset();
            return std::nullopt;
        }

        const auto command =
            aimer_.aim(*pose, *transform, correction);

        // 二次校验，防止超时
        if (!command || expired(frame)) {
            previous_center_.reset();
            return std::nullopt;
        }

        previous_center_ = selected_center;

        AimResult result;

        result.armor = *selected;
        result.pose = *pose;
        result.gimbal = *command;
        result.frame_id = frame.frame_id;

        result.shoot.fire_flag = 0;

        return result;
    }

private:
    ArmorDetector& detector_;
    PoseSolver& solver_;
    TransformProvider& transforms_;
    Aimer& aimer_;

    float minimum_confidence_;
    double maximum_age_ms_;
    double association_distance_px_;

    std::optional<cv::Point2f> previous_center_;

    bool expired(const CameraFrame& frame) const
    {
        const double age_ms =
            std::chrono::duration<double, std::milli>(
                Clock::now() - frame.received_at
            ).count();

        return age_ms < 0 || age_ms > maximum_age_ms_;
    }
};

}  // namespace srm