#pragma once
#include "utils/OpenCVUtil.h"
class cColorDepthFixer
{
public:
    static cv::Mat FixColorDepth(const cv::Mat &color, const cv::Mat &depth, double alpha = 1);
    static void FixColorDepth(std::string color_path, std::string depth_path, double alpha = 1);
};