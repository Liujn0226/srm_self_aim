#include "protocol.hpp"

#include <climits>    // CHAR_BIT
#include <cmath>      // std::isfinite
#include <cstring>    // std::memcpy
#include <limits>     // std::numeric_limits
#include <stdexcept>  // std::invalid_argument

namespace srm
{

namespace
{

// 1. 编译时检查基础数据类型

static_assert(
    CHAR_BIT == 8,
    "Protocol requires 8-bit bytes"
);

static_assert(
    sizeof(float) == 4 &&
    std::numeric_limits<float>::is_iec559,
    "Protocol requires IEEE-754 float32"
);


// 2. 上位机->下位机，数字->字节（整数）

//16位整数
void write_u16_le(
    std::uint8_t* destination,
    std::uint16_t value
)
{
    destination[0] =
        static_cast<std::uint8_t>(value & 0xFFu);

    destination[1] =
        static_cast<std::uint8_t>((value >> 8) & 0xFFu);
}


// 32位整数
void write_u32_le(
    std::uint8_t* destination,
    std::uint32_t value
)
{
    destination[0] =
        static_cast<std::uint8_t>(value & 0xFFu);

    destination[1] =
        static_cast<std::uint8_t>((value >> 8) & 0xFFu);

    destination[2] =
        static_cast<std::uint8_t>((value >> 16) & 0xFFu);

    destination[3] =
        static_cast<std::uint8_t>((value >> 24) & 0xFFu);
}


// 3. 下位机->上位机，字节->数字（整数）

//16位整数
std::uint16_t read_u16_le(const std::uint8_t* source)
{
    const std::uint32_t value =
        static_cast<std::uint32_t>(source[0]) |
        (static_cast<std::uint32_t>(source[1]) << 8);

    return static_cast<std::uint16_t>(value);
}


std::uint32_t read_u32_le(const std::uint8_t* source)
{
    return
        static_cast<std::uint32_t>(source[0]) |
        (static_cast<std::uint32_t>(source[1]) << 8) |
        (static_cast<std::uint32_t>(source[2]) << 16) |
        (static_cast<std::uint32_t>(source[3]) << 24);
}


//正负数处理
std::int32_t read_i32_le(const std::uint8_t* source)
{
    const std::uint32_t bits = read_u32_le(source);

    if (bits <= 0x7FFFFFFFu) {
        return static_cast<std::int32_t>(bits);
    }

    const std::int64_t signed_value =
        static_cast<std::int64_t>(bits) - 0x100000000LL;

    return static_cast<std::int32_t>(signed_value);
}


// 4. 浮点数

void write_f32_le(
    std::uint8_t* destination,
    float value
)
{
    std::uint32_t bits = 0;

    std::memcpy(&bits, &value, sizeof(bits));

    write_u32_le(destination, bits);
}

float read_f32_le(const std::uint8_t* source)
{
    const std::uint32_t bits = read_u32_le(source);

    float value = 0.0f;

    std::memcpy(&value, &bits, sizeof(value));

    return value;
}

}  // 匿名命名空间结束


// 5. 编码上位机发送给下位机的报文
// 整帧偏移    字段
// 0～1        body_len = 16
// 2～3        ID = 1
// 4～7        yaw
// 8～11       pitch
// 12～13      ID = 2
// 14～17      fire_flag

std::array<std::uint8_t, kCommandFrameSize>
encode_command(
    const Gimbal_Receive_s& gimbal,
    const Shoot_Receive_s& shoot
)
{

    if (!std::isfinite(gimbal.yaw) ||
        !std::isfinite(gimbal.pitch)) {
        throw std::invalid_argument(
            "Command yaw and pitch must be finite"
        );
    }

    std::array<std::uint8_t, kCommandFrameSize> frame{};

    // body_len只计算正文，不包含自身的2字节。
    write_u16_le(frame.data() + 0, 16);

    // ID 1：云台目标。
    write_u16_le(frame.data() + 2, 1);

    write_f32_le(frame.data() + 4, gimbal.yaw);
    write_f32_le(frame.data() + 8, gimbal.pitch);

    // ID 2：开火请求。
    write_u16_le(frame.data() + 12, 2);

    write_u32_le(
        frame.data() + 14,
        static_cast<std::uint32_t>(shoot.fire_flag)
    );

    return frame;
}


// 6. 解析下位机发送给上位机的字节流

// 整帧偏移    字段
// 0～1        body_len = 28
// 2～3        ID = 1
// 4～7        yaw
// 8～11       pitch
// 12～15      roll
// 16～19      mode
// 20～23      color
// 24～25      ID = 2
// 26～29      bullet_speed
std::vector<FeedbackFrame> FeedbackParser::feed(
    const std::uint8_t* data,
    std::size_t size
)
{
    // 有没有完整的云台反馈
    std::vector<FeedbackFrame> results;

    // 没有
    if (size == 0) {
        return results;
    }

    // 有，data不为0
    if (data == nullptr) {
        throw std::invalid_argument(
            "Input data cannot be null when size is nonzero"
        );
    }

    // 把新收到的字节追加到已有缓存末尾。
    buffer_.insert(buffer_.end(), data, data + size);

    while (true) {
        if (buffer_.size() < 2) {
            break;
        }

        const std::uint16_t body_len =
            read_u16_le(buffer_.data());

        // 反馈正文固定为28字节。
        if (body_len != 28) {
            // 丢弃当前全部缓存。
            buffer_.clear();

            // 统计的是清空缓存事件次数，不是丢失帧数。
            ++discard_count_;

            break;
        }

        // 长度正确，但完整30字节还没有到齐。
        if (buffer_.size() < kFeedbackFrameSize) {
            break;
        }

        const std::uint8_t* frame = buffer_.data();

        // 固定偏移处检查两个记录ID。
        const std::uint16_t first_id =
            read_u16_le(frame + 2);

        const std::uint16_t second_id =
            read_u16_le(frame + 24);

        if (first_id != 1 || second_id != 2) {
            buffer_.clear();
            ++discard_count_;
            break;
        }

        FeedbackFrame feedback{};

        // ID 1：云台状态

        feedback.gimbal.yaw =
            read_f32_le(frame + 4);

        feedback.gimbal.pitch =
            read_f32_le(frame + 8);

        feedback.gimbal.roll =
            read_f32_le(frame + 12);

        feedback.gimbal.mode =
            read_i32_le(frame + 16);

        feedback.gimbal.color =
            read_i32_le(frame + 20);

        // ID 2：发射状态

        feedback.shoot.bullet_speed =
            read_f32_le(frame + 26);

        // 保存ID 1和ID 2
        results.push_back(feedback);

        // 删除已经处理完的30字节。
        buffer_.erase(
            buffer_.begin(),
            buffer_.begin() + kFeedbackFrameSize
        );

    }

    return results;
}


// 7. 清空接收缓存

void FeedbackParser::reset()
{
    buffer_.clear();

}

// 8. 查询缓存字节数

std::size_t FeedbackParser::buffered_bytes() const
{
    return buffer_.size();
}


// 9. 查询丢弃缓存的累计次数

std::size_t FeedbackParser::discard_count() const
{
    return discard_count_;
}

}  // namespace srm