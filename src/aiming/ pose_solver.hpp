#pragma once

#include "common/types.hpp"

#include <opencv2/calib3d.hpp>

#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace srm
{

class PoseSolver
{
public:
    PoseSolver(
        const std::string& intrinsics_file,
        double feature_width_m,
        double feature_height_m,
        double max_error_px
    )
        : max_error_px_(max_error_px)
    {
        if (!std::isfinite(feature_width_m) ||
            !std::isfinite(feature_height_m) ||
            !std::isfinite(max_error_px) ||
            feature_width_m <= 0 ||
            feature_height_m <= 0 ||
            max_error_px <= 0) {
            throw std::invalid_argument("Invalid PnP parameters");
        }

        cv::FileStorage file(
            intrinsics_file,
            cv::FileStorage::READ
        );

        if (!file.isOpened()) {
            throw std::runtime_error("Cannot open camera intrinsics");
        }

        file["camera_matrix"] >> camera_matrix_;
        file["distortion_coefficients"] >> distortion_;

        file["image_width"] >> image_size_.width;
        file["image_height"] >> image_size_.height;

        if (camera_matrix_.rows != 3 ||
            camera_matrix_.cols != 3 ||
            distortion_.empty() ||
            image_size_.empty() ||
            !cv::checkRange(camera_matrix_) ||
            !cv::checkRange(distortion_)) {
            throw std::runtime_error("Invalid camera intrinsics");
        }

        camera_matrix_.convertTo(camera_matrix_, CV_64F);
        distortion_.convertTo(distortion_, CV_64F);

        if (camera_matrix_.at<double>(0, 0) <= 0 ||
            camera_matrix_.at<double>(1, 1) <= 0) {
            throw std::runtime_error("Invalid focal length");
        }

        const float half_w =
            static_cast<float>(feature_width_m / 2);

        const float half_h =
            static_cast<float>(feature_height_m / 2);

        //算中心点
        object_points_ = {
            {-half_w, -half_h, 0},
            { half_w, -half_h, 0},
            { half_w,  half_h, 0},
            {-half_w,  half_h, 0}
        };
    }

    std::optional<ArmorPose> solve(
        const ArmorDetection& armor,
        cv::Size actual_image_size
    ) const
    {
        // 与相机标定时分辨率一样
        if (actual_image_size != image_size_) {
            throw std::runtime_error(
                "Image resolution differs from calibration"
            );
        }

        std::vector<cv::Point2f> image_points(
            armor.points.begin(),
            armor.points.end()
        );

        for (const auto& point : image_points) {
            if (!std::isfinite(point.x) ||
                !std::isfinite(point.y)) {
                return std::nullopt;
            }
        }

        // pnp两个候选解（真实和镜像）
        std::vector<cv::Mat> rotations;
        std::vector<cv::Mat> translations;

        //调用pnp
        const int count = cv::solvePnPGeneric(
            object_points_,
            image_points,
            camera_matrix_,
            distortion_,
            rotations,
            translations,
            false,
            cv::SOLVEPNP_IPPE
        );

        //最优解
        std::optional<ArmorPose> best;
        double best_error = std::numeric_limits<double>::infinity();

        for (int i = 0; i < count; ++i) {
            if (!cv::checkRange(rotations[i]) ||
                !cv::checkRange(translations[i])) {
                continue;
            }

            const cv::Mat r = rotations[i].reshape(1, 3);
            const cv::Mat t = translations[i].reshape(1, 3);

            const cv::Vec3d rvec(
                r.at<double>(0, 0),
                r.at<double>(1, 0),
                r.at<double>(2, 0)
            );

            const cv::Vec3d tvec(
                t.at<double>(0, 0),
                t.at<double>(1, 0),
                t.at<double>(2, 0)
            );

            cv::Mat rotation_matrix;
            cv::Rodrigues(rvec, rotation_matrix);

            //判断候选解是不是在相机前方
            bool in_front = true;

            for (const auto& p : object_points_) {
                const double z =
                    rotation_matrix.at<double>(2, 0) * p.x +
                    rotation_matrix.at<double>(2, 1) * p.y +
                    rotation_matrix.at<double>(2, 2) * p.z +
                    tvec[2];

                if (z <= 0) {
                    in_front = false;
                    break;
                }
            }

            if (!in_front) {
                continue;
            }

            std::vector<cv::Point2f> projected;

            //三维到二维
            cv::projectPoints(
                object_points_,
                rvec,
                tvec,
                camera_matrix_,
                distortion_,
                projected
            );

            double sum = 0;

            for (std::size_t j = 0; j < projected.size(); ++j) {
                const auto delta = projected[j] - image_points[j];
                sum += delta.dot(delta);
            }

            const double error =
                std::sqrt(sum / projected.size());

            if (!std::isfinite(error) ||
                error > max_error_px_ ||
                error >= best_error) {
                continue;
            }

            best_error = error;

            ArmorPose result;
            result.rvec = rvec;
            result.position_camera_m = tvec;
            result.reprojection_error_px = error;

            best = result;
        }

        //返回最优解
        return best;
    }

private:
    cv::Mat camera_matrix_;
    cv::Mat distortion_;

    cv::Size image_size_;

    std::vector<cv::Point3f> object_points_;

    double max_error_px_;
};

}  // namespace srm