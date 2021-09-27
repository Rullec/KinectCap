#include "OpenCVUtil.h"
#include "utils/LogUtil.h"
using namespace cv;

cv::Mat cOpencvUtil::ConvertFloatArrayToRGBMat(int height, int width, float *array)
{
    Mat new_mat(height, width, CV_8UC3);
    for (int row = 0; row < height; row++)
        for (int col = 0; col < width; col++)
        {
            auto &pixel = new_mat.at<Vec3b>(row, col);
            int buffer_idx = ((height - 1 - row) * width + col) * 3;
            pixel[0] = uint8_t(array[buffer_idx + 2] * 255.99); // B
            pixel[1] = uint8_t(array[buffer_idx + 1] * 255.99); // G
            pixel[2] = uint8_t(array[buffer_idx + 0] * 255.99); // G
        }
    // imshow("hello", new_mat);
    // waitKey(0);
    // exit(1);
    return new_mat;
}

cv::Mat cOpencvUtil::ConvertFloatArrayToGrayscaleMat(int height, int width, int input_channels, float *array)
{
    Mat new_mat(height, width, CV_8UC1);
    for (int row = 0; row < height; row++)
        for (int col = 0; col < width; col++)
        {
            int buf_idx = ((height - 1 - row) * width + col) * input_channels;
            int val = int(array[buf_idx] * 255.99);
            if (val > 255)
                val = 255;
            new_mat.at<uint8_t>(row, col) = val;
        }
    return new_mat;
}