#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char** argv)
{
    if (argc != 6) {
        std::cerr
            << "Usage: calibrate_camera "
            << "\"images/*.png\" COLS ROWS SQUARE_M output.yaml\n";
        return 1;
    }

    try {
        //棋盘格内角点列数
        const int cols = std::stoi(argv[2]);
        //棋盘格内角点行数
        const int rows = std::stoi(argv[3]);
        // 棋盘格边长
        const float square_m = std::stof(argv[4]);

        if (cols < 2 || rows < 2 ||
            !std::isfinite(square_m) || square_m <= 0) {
            throw std::runtime_error("Invalid board parameters");
        }

        const cv::Size pattern(cols, rows);

        std::vector<cv::String> files;
        cv::glob(argv[1], files, false);

        // 棋盘格位于Z=0平面。
        std::vector<cv::Point3f> board_points;

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                board_points.emplace_back(
                    col * square_m,
                    row * square_m,
                    0.0f
                );
            }
        }

        //3D坐标
        std::vector<std::vector<cv::Point3f>> object_points;
        //2D坐标
        std::vector<std::vector<cv::Point2f>> image_points;
        //棋盘格图片名
        std::vector<std::string> used_files;

        cv::Size image_size;

        //所有图的分辨率一样
        for (const auto& file : files) {
            const cv::Mat image =
                cv::imread(file, cv::IMREAD_GRAYSCALE);

            if (image.empty()) {
                continue;
            }

            if (image_size.empty()) {
                image_size = image.size();
            }

            if (image.size() != image_size) {
                throw std::runtime_error(
                    "Calibration images have different resolutions"
                );
            }

            //存放棋盘格角点
            std::vector<cv::Point2f> corners;

            const bool found = cv::findChessboardCorners(
                image,
                pattern,
                corners,
                cv::CALIB_CB_ADAPTIVE_THRESH |
                cv::CALIB_CB_NORMALIZE_IMAGE
            );

            if (!found) {
                std::cout << "Skipped: " << file << '\n';
                continue;
            }

            // 细化角点位置，提高标定精度
            cv::cornerSubPix(
                image,
                corners,
                cv::Size(11, 11),
                cv::Size(-1, -1),
                cv::TermCriteria(
                    cv::TermCriteria::EPS |
                    cv::TermCriteria::COUNT,
                    30,
                    0.001
                )
            );

            object_points.push_back(board_points);
            image_points.push_back(corners);
            used_files.push_back(file);
        }

        // 最少10张
        if (image_points.size() < 10) {
            throw std::runtime_error(
                "Need at least 10 usable and varied calibration images"
            );
        }

        //相机内参
        cv::Mat camera_matrix;
        //相机畸变系数
        cv::Mat distortion;
        //棋盘格旋转向量
        std::vector<cv::Mat> rvecs;
        //棋盘格平移向量
        std::vector<cv::Mat> tvecs;

        //rms误差
        const double rms = cv::calibrateCamera(
            object_points,
            image_points,
            image_size,
            camera_matrix,
            distortion,
            rvecs,
            tvecs
        );

        if (!cv::checkRange(camera_matrix) ||
            !cv::checkRange(distortion) ||
            !std::isfinite(rms)) {
            throw std::runtime_error("Invalid calibration result");
        }

        // 输出每张图片的重投影误差，
        // 方便定位模糊、误检或姿态覆盖不合理的样本。
        for (std::size_t i = 0; i < image_points.size(); ++i) {
            std::vector<cv::Point2f> projected;

            //3D到2D，与真实角点比较，算距离
            cv::projectPoints(
                object_points[i],
                rvecs[i],
                tvecs[i],
                camera_matrix,
                distortion,
                projected
            );

            double squared_error = 0;

            for (std::size_t j = 0; j < projected.size(); ++j) {
                const auto delta =
                    projected[j] - image_points[i][j];

                squared_error += delta.dot(delta);
            }

            const double image_rms =
                std::sqrt(squared_error / projected.size());

            std::cout
                << used_files[i]
                << " RMS=" << image_rms << " px\n";
        }

        //在YAML文件中写入
        cv::FileStorage output(
            argv[5],
            cv::FileStorage::WRITE
        );

        if (!output.isOpened()) {
            throw std::runtime_error("Cannot open output file");
        }

        output << "image_width" << image_size.width;
        output << "image_height" << image_size.height;
        output << "camera_matrix" << camera_matrix;
        output << "distortion_coefficients" << distortion;

        output << "board_cols" << cols;
        output << "board_rows" << rows;
        output << "square_m" << square_m;
        output << "rms_px" << rms;

        std::cout << "Overall RMS=" << rms << " px\n";
    }
    //报错
    catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}