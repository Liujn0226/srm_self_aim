#include "camera/hik_camera.hpp"

#include <MvCameraControl.h>

#include <limits>
#include <sstream>
#include <stdexcept>

namespace srm
{

namespace
{

//SDK有没有执行成功
void check_sdk(int result, const char* operation)
{
    if (result == MV_OK) {
        return;
    }

    std::ostringstream message;

    message << operation
            << " failed, SDK code=0x"
            << std::hex
            << static_cast<unsigned int>(result);

    throw std::runtime_error(message.str());
}

//析构函数
struct ImageBufferGuard
{
    void* handle;
    MV_FRAME_OUT* frame;

    ~ImageBufferGuard()
    {
        MV_CC_FreeImageBuffer(handle, frame);
    }
};

}

//相机函数
HikCamera::HikCamera(
    unsigned int device_index,
    float exposure_us,
    float gain
)
{
    try {
        MV_CC_DEVICE_INFO_LIST devices{};

        // 列举相机
        check_sdk(
            MV_CC_EnumDevices(MV_USB_DEVICE, &devices),
            "MV_CC_EnumDevices"
        );

        if (device_index >= devices.nDeviceNum) {
            throw std::runtime_error("Requested USB camera not found");
        }

        //相机句柄
        check_sdk(
            MV_CC_CreateHandle(
                &handle_,
                devices.pDeviceInfo[device_index]
            ),
            "MV_CC_CreateHandle"
        );

        //打开相机
        check_sdk(
            MV_CC_OpenDevice(handle_),
            "MV_CC_OpenDevice"
        );

        opened_ = true;

        // 连续采集
        check_sdk(
            MV_CC_SetEnumValue(handle_, "TriggerMode", 0),
            "Set TriggerMode"
        );

        //关闭自动曝光
        check_sdk(
            MV_CC_SetEnumValue(handle_, "ExposureAuto", 0),
            "Disable ExposureAuto"
        );

        //关闭自动增益
        check_sdk(
            MV_CC_SetEnumValue(handle_, "GainAuto", 0),
            "Disable GainAuto"
        );

       //设置曝光时间
        check_sdk(
            MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_us),
            "Set ExposureTime"
        );

        //设置增益
        check_sdk(
            MV_CC_SetFloatValue(handle_, "Gain", gain),
            "Set Gain"
        );

        //相机开始采集
        check_sdk(
            MV_CC_StartGrabbing(handle_),
            "MV_CC_StartGrabbing"
        );

        grabbing_ = true;
    }
    //异常
    catch (...) {
        close();
        throw;
    }
}

//析构函数
HikCamera::~HikCamera()
{
    close();
}

//关闭相机
void HikCamera::close() noexcept
{
    if (handle_ == nullptr) {
        return;
    }

    if (grabbing_) {
        MV_CC_StopGrabbing(handle_);
        grabbing_ = false;
    }

    if (opened_) {
        MV_CC_CloseDevice(handle_);
        opened_ = false;
    }

    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
}

//读取图像
CameraFrame HikCamera::read(unsigned int timeout_ms)
{
    MV_FRAME_OUT raw{};

    //从缓冲区拿到图像
    check_sdk(
        MV_CC_GetImageBuffer(handle_, &raw, timeout_ms),
        "MV_CC_GetImageBuffer"
    );

    //释放内存
    ImageBufferGuard guard{handle_, &raw};

    //新建对象
    CameraFrame result;

    // 时间戳
    result.received_at = Clock::now();
    //第几张
    result.frame_id = frame_id_++;

    //图像长宽
    const int width =
        static_cast<int>(raw.stFrameInfo.nWidth);

    const int height =
        static_cast<int>(raw.stFrameInfo.nHeight);

    if (width <= 0 || height <= 0) {
        throw std::runtime_error("Invalid camera image size");
    }

    // 使SDK缓冲内存释放后，对象仍然有效。
    result.bgr.create(height, width, CV_8UC3);

    const std::size_t output_bytes =
        result.bgr.total() * result.bgr.elemSize();

    if (output_bytes >
        std::numeric_limits<unsigned int>::max()) {
        throw std::runtime_error("Camera frame is too large");
    }

    //把图像格式转为opencv专用格式
    MV_CC_PIXEL_CONVERT_PARAM conversion{};

    conversion.nWidth = raw.stFrameInfo.nWidth;
    conversion.nHeight = raw.stFrameInfo.nHeight;

    conversion.pSrcData = raw.pBufAddr;
    conversion.nSrcDataLen = raw.stFrameInfo.nFrameLen;
    conversion.enSrcPixelType = raw.stFrameInfo.enPixelType;

    conversion.pDstBuffer = result.bgr.data;
    conversion.nDstBufferSize =
        static_cast<unsigned int>(output_bytes);

    conversion.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

    check_sdk(
        MV_CC_ConvertPixelType(handle_, &conversion),
        "MV_CC_ConvertPixelType"
    );

    return result;
}

}  // namespace srm