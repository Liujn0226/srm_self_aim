#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace srm
{

// 上位机 -> 下位机
inline constexpr std::size_t kCommandFrameSize = 18;

// 下位机 -> 上位机
inline constexpr std::size_t kFeedbackFrameSize = 30;


// 上位机发送给下位机
struct Gimbal_Receive_s
{
    // 云台 yaw 目标，角度制
    float yaw = 0.0f;

    // 云台 pitch 目标，角度制
    float pitch = 0.0f;
};


struct Shoot_Receive_s
{
    // `0` 表示不请求开火；非零表示请求开火
    std::int32_t fire_flag = 0;
};


// 下位机发送给上位机

struct Gimbal_Send_s
{
    //当前 yaw，角度制
    float yaw = 0.0f;

    // 当前 pitch，角度制
    float pitch = 0.0f;

    // 当前 roll，角度制
    float roll = 0.0f;

    // 当前分支固定发送 `0`，保留字段
    std::int32_t mode = 0;

    //当前分支写入机器人 ID；字段名沿用源码
    std::int32_t color = 0;
};


struct Shoot_Send_s
{
    //裁判系统提供的弹丸初速度，m/s
    float bullet_speed = 0.0f;
};


// 一帧下位机反馈

struct FeedbackFrame
{
    Gimbal_Send_s gimbal;
    Shoot_Send_s shoot;
};


// 报文编码

// 将ID1和ID2两个数据体编码成一帧18字节报文。
std::array<std::uint8_t, kCommandFrameSize>
encode_command(
    const Gimbal_Receive_s& gimbal,
    const Shoot_Receive_s& shoot
);


// 报文解析

class FeedbackParser
{
public:
    // 输入本次从USB CDC收到的字节。
    //
    // 一次可能收到：
    // 1. 半帧
    // 2. 一帧
    // 3. 多帧
    //
    // 函数会自动缓存不完整数据。
    std::vector<FeedbackFrame> feed(
        const std::uint8_t* data,
        std::size_t size
    );

    // 清空未解析的缓存。
    void reset();

    // 当前缓存中还有多少字节。
    std::size_t buffered_bytes() const;

    // 因非法长度或非法ID丢弃数据的次数。
    std::size_t discard_count() const;

private:
    std::vector<std::uint8_t> buffer_;
    std::size_t discard_count_ = 0;
};

}  // namespace srm