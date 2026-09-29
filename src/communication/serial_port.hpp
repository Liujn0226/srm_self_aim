#pragma once

#ifndef __linux__
#error "This SerialPort implementation requires Linux"
#endif

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <system_error>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

//打开串口
namespace srm
{

class SerialPort
{
public:
    explicit SerialPort(const std::string& device)
    {
        fd_ = ::open(
            device.c_str(),
            O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC
        );

        if (fd_ < 0) {
            throw_system_error("open " + device);
        }

        termios options{};

        if (::tcgetattr(fd_, &options) != 0) {
            const int saved_errno = errno;
            ::close(fd_);
            fd_ = -1;

            throw std::system_error(
                saved_errno,
                std::generic_category(),
                "tcgetattr"
            );
        }


        ::cfmakeraw(&options);

        options.c_cflag |= CLOCAL | CREAD;
        options.c_cflag &= ~CRTSCTS;

        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0;

        if (::tcsetattr(fd_, TCSANOW, &options) != 0) {
            const int saved_errno = errno;
            ::close(fd_);
            fd_ = -1;

            throw std::system_error(
                saved_errno,
                std::generic_category(),
                "tcsetattr"
            );
        }
    }

    //析构函数
    ~SerialPort()
    {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }

    // 禁止复制，避免两个对象重复关闭同一个设备
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // 读取函数
    std::size_t read_some(
        std::uint8_t* destination,
        std::size_t capacity,
        int timeout_ms
    )
    {
        if (capacity == 0) {
            return 0;
        }

        if (destination == nullptr || timeout_ms < 0) {
            throw std::invalid_argument("Invalid read arguments");
        }

        if (!wait_event(POLLIN, timeout_ms)) {
            return 0;
        }

        const ssize_t count =
            ::read(fd_, destination, capacity);

        if (count > 0) {
            return static_cast<std::size_t>(count);
        }

        if (count < 0) {
            if (errno == EAGAIN ||
                errno == EWOULDBLOCK ||
                errno == EINTR) {
                return 0;
            }

            throw_system_error("read");
        }

        throw std::runtime_error("Serial device returned EOF");
    }

    //发送函数
    void write_all(
        const std::uint8_t* data,
        std::size_t size,
        int timeout_ms
    )
    {
        if (size == 0) {
            return;
        }

        if (data == nullptr || timeout_ms <= 0) {
            throw std::invalid_argument("Invalid write arguments");
        }

        const auto deadline =
            std::chrono::steady_clock::now() +
            std::chrono::milliseconds(timeout_ms);

        std::size_t offset = 0;

        while (offset < size) {
            const auto now = std::chrono::steady_clock::now();

            if (now >= deadline) {
                throw std::runtime_error("Serial write timeout");
            }

            const auto remaining =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    deadline - now
                ).count();

            if (!wait_event(
                    POLLOUT,
                    std::max(1, static_cast<int>(remaining)))) {
                throw std::runtime_error("Serial write timeout");
            }

            const ssize_t count = ::write(
                fd_,
                data + offset,
                size - offset
            );

            if (count > 0) {
                offset += static_cast<std::size_t>(count);
                continue;
            }

            if (count < 0 &&
                (errno == EINTR ||
                 errno == EAGAIN ||
                 errno == EWOULDBLOCK)) {
                continue;
            }

            if (count == 0) {
                throw std::runtime_error("Serial write made no progress");
            }

            throw_system_error("write");
        }
    }

private:
    int fd_ = -1;

    //统一报错
    [[noreturn]]
    static void throw_system_error(const std::string& operation)
    {
        throw std::system_error(
            errno,
            std::generic_category(),
            operation
        );
    }

    //监听串口有没有数据，能不能发数据
    bool wait_event(short event, int timeout_ms)
    {
        pollfd descriptor{};
        descriptor.fd = fd_;
        descriptor.events = event;

        const int result = ::poll(&descriptor, 1, timeout_ms);

        if (result < 0) {
            if (errno == EINTR) {
                return false;
            }

            throw_system_error("poll");
        }

        if (result == 0) {
            return false;
        }

        if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            throw std::runtime_error("Serial device disconnected or failed");
        }

        return (descriptor.revents & event) != 0;
    }
};

}  // namespace srm