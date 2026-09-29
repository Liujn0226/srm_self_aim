#include "camera/hik_camera.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    //参数
    if (argc != 5) {
        std::cerr
            << "Usage: camera_view INDEX EXPOSURE_US GAIN SAVE_DIR\n";
        return 1;
    }

    try {
        const auto index =
            static_cast<unsigned int>(std::stoul(argv[1]));

        const float exposure_us = std::stof(argv[2]);
        const float gain = std::stof(argv[3]);

        const std::filesystem::path directory = argv[4];

        std::filesystem::create_directories(directory);

        srm::HikCamera camera(index, exposure_us, gain);

        //读取图片，（s保存，q退出）
        while (true) {
            const auto frame = camera.read();

            cv::imshow("Hikrobot: S=save, Q=quit", frame.bgr);

            const int key = cv::waitKey(1) & 0xFF;

            if (key == 'q' || key == 27) {
                break;
            }

            if (key == 's') {
                const auto time_id =
                    std::chrono::duration_cast<
                        std::chrono::microseconds
                    >(
                        std::chrono::system_clock::now()
                            .time_since_epoch()
                    ).count();

                    //自动保存路径的名字
                const auto path =
                    directory / (std::to_string(time_id) + ".png");

                // 保存原始分辨率图像
                if (!cv::imwrite(path.string(), frame.bgr)) {
                    throw std::runtime_error("Failed to save image");
                }

                std::cout << "Saved: " << path << '\n';
            }
        }
    }
    //报错
    catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}