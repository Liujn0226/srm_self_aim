#pragma once

#include "common/types.hpp"

namespace srm
{

class HikCamera
{
public:
    //相机
    HikCamera(
        unsigned int device_index,
        float exposure_us,
        float gain
    );

    //析构函数
    ~HikCamera();

    //禁止重复
    HikCamera(const HikCamera&) = delete;
    HikCamera& operator=(const HikCamera&) = delete;

    //获取图像
    CameraFrame read(unsigned int timeout_ms = 1000);

private:
    void* handle_ = nullptr;

    bool opened_ = false;
    bool grabbing_ = false;

    std::uint64_t frame_id_ = 0;

    void close() noexcept;
};

}  // namespace srm