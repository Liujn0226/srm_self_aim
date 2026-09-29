#include "communication/protocol.hpp"
#include "communication/serial_port.hpp"

#include <array>
#include <csignal>
#include <iostream>

//信号处理
namespace
{
volatile std::sig_atomic_t stopped = 0;

void on_signal(int)
{
    stopped = 1;
}
}

int main(int argc, char** argv)
{
    //参数是不是程序加小电脑侧设备名
    if (argc != 2) {
        std::cerr
            << "Usage: communication_check /dev/ttyACM0\n";
        return 1;
    }

    //control c停止
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    //终端打印yaw,pitch等等五个
    try {
        srm::SerialPort serial(argv[1]);

        srm::FeedbackParser parser;

        std::array<std::uint8_t, 512> buffer{};

        while (!stopped) {
            const auto count = serial.read_some(
                buffer.data(),
                buffer.size(),
                100
            );

            if (count == 0) {
                continue;
            }

            const auto frames =
                parser.feed(buffer.data(), count);

            for (const auto& frame : frames) {
                std::cout
                    << "yaw=" << frame.gimbal.yaw
                    << " pitch=" << frame.gimbal.pitch
                    << " roll=" << frame.gimbal.roll
                    << " mode=" << frame.gimbal.mode
                    << " robot_id=" << frame.gimbal.color
                    << " speed=" << frame.shoot.bullet_speed
                    << '\n';
            }
        }
    }

    //异常
    catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}