#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace srm
{

// 上->下，18字节
inline constexpr std::size_t kCommandFrameSize = 18;

// 下->上，30字节
inline constexpr std::size_t kFeedbackFrameSize = 30;


// 程序内部的数据结构

// 下 -> 上
struct GimbalFeedback
{
    float yaw_deg = 0.0f;
    float pitch_deg = 0.0f;
    float roll_deg = 0.0f;

    // 当前分支固定发送 `0`，保留字段 
    std::int32_t mode = 0;

    // 当前分支写入机器人 ID；字段名沿用源码
    std::int32_t robot_id = 0;

    float bullet_speed_mps = 0.0f;
};

// 上-> 下
struct GimbalCommand
{
    float yaw_deg = 0.0f;
    float pitch_deg = 0.0f;

    //  `0` 表示不请求开火；非零表示请求开火
    std::int32_t fire_flag = 0;
};


// 发送编码接口
//上->下，结构化数据->字节
std::array

<std::uint8_t, kCommandFrameSize>
encode_command(const GimbalCommand& command);


// 接收解析接口
//下->上，字节->结构化数据
class FeedbackParser
{
public:
    std::vector<GimbalFeedback> feed(
        const std::uint8_t* data,
        std::size_t size
    );

    void reset();

    std::size_t buffered_bytes() const;

    std::size_t discard_count() const;

private:
    std::vector<std::uint8_t> buffer_;

    std::size_t discard_count_ = 0;
};

}  