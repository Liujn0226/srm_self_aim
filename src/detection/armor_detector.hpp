#pragma once

#include "common/types.hpp"

#include <vector>

namespace srm
{

class ArmorDetector
{
public:
    virtual ~ArmorDetector() = default;

    // 输入海康相机输出的BGR图像。
    //
    // 实际模型适配类需要完成：
    // 1. resize、归一化等预处理；
    // 2. 模型推理；
    // 3. 置信度过滤和必要的NMS；
    // 4. 提取四个物理关键点；
    // 5. 撤销缩放、补边和ROI偏移；
    // 6. 按约定排列四个关键点。
    //
    // 返回值中的点必须是原始图像上的像素坐标。
    virtual std::vector<ArmorDetection> detect(
        const cv::Mat& bgr
    ) = 0;
};

}  // namespace srm