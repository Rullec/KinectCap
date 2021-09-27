#pragma once
#include <opencv2/opencv.hpp>

class cOpencvUtil
{
public:
    static cv::Mat ConvertFloatArrayToRGBMat(int height, int width, float *array);
    static cv::Mat ConvertFloatArrayToGrayscaleMat(int height, int width, int input_channels, float *array);
};