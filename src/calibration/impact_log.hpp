#pragma once

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

//写入时间戳，距离，弹速，弹速是否有效，yaw指令，pitch指令，横向偏移，纵向偏移，弹数
namespace srm
{

struct ImpactSample
{
    double distance_m = 0;

    double bullet_speed_mps = 0;

    bool bullet_speed_verified = false;

    double yaw_command_deg = 0;
    double pitch_command_deg = 0;

    // 测量约定：
    // dx：从发射位看靶面，向右为正；
    // dy：向上为正
    double mean_dx_mm = 0;
    double mean_dy_mm = 0;

    int measured_impacts = 0;
};

class ImpactLog
{
public:
    explicit ImpactLog(const std::string& filename)
    {
        const std::filesystem::path path(filename);

        if (!path.parent_path().empty()) {
            std::filesystem::create_directories(
                path.parent_path()
            );
        }

        const bool need_header =
            !std::filesystem::exists(path) ||
            std::filesystem::file_size(path) == 0;

        file_.open(path, std::ios::app);

        if (!file_) {
            throw std::runtime_error("Cannot open impact log");
        }

        if (need_header) {
            file_
                << "time_ms,distance_m,bullet_speed_mps,"
                << "bullet_speed_verified,"
                << "yaw_command_deg,pitch_command_deg,"
                << "mean_dx_mm,mean_dy_mm,measured_impacts\n";
        }
    }

    void append(const ImpactSample& sample)
    {
        if (sample.measured_impacts <= 0) {
            throw std::invalid_argument(
                "Impact sample needs measured impacts"
            );
        }

        const auto time_ms =
            std::chrono::duration_cast<
                std::chrono::milliseconds
            >(
                std::chrono::system_clock::now()
                    .time_since_epoch()
            ).count();

        file_ << std::setprecision(10)
              << time_ms << ','
              << sample.distance_m << ','
              << sample.bullet_speed_mps << ','
              << sample.bullet_speed_verified << ','
              << sample.yaw_command_deg << ','
              << sample.pitch_command_deg << ','
              << sample.mean_dx_mm << ','
              << sample.mean_dy_mm << ','
              << sample.measured_impacts << '\n';

        file_.flush();

        if (!file_) {
            throw std::runtime_error("Failed to write impact log");
        }
    }

private:
    std::ofstream file_;
};

}  // namespace srm