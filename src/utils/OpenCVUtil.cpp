#include "OpenCVUtil.h"
#include "utils/LogUtil.h"
using namespace cv;

uint8_t Clamp(int val)
{
    uint8_t new_val = 0;

    if(val < 0 ) new_val = 0;

    else if(val > 255) new_val = 255;
    else new_val = val;
    return new_val;
}
cv::Mat cOpencvUtil::ConvertFloatArrayToRGBMat(int height, int width, float *array)
{
    Mat new_mat(height, width, CV_8UC3);
    for (int row = 0; row < height; row++)
        for (int col = 0; col < width; col++)
        {
            auto &pixel = new_mat.at<Vec3b>(row, col);
            int buffer_idx = ((height - 1 - row) * width + col) * 3;
            int val_B = array[buffer_idx + 2] * 255.99;
            int val_G = array[buffer_idx + 1] * 255.99;
            int val_R = array[buffer_idx + 0] * 255.99;
            
            pixel[0] = Clamp(val_B); // B
            pixel[1] = Clamp(val_G); // G
            pixel[2] = Clamp(val_R); // R
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

void cOpencvUtil::ConvertRGBMatToFloatArray(const cv::Mat &mat, int &height, int &width, std::vector<float> &array)
{
    height = mat.rows;
    width = mat.cols;
    array.resize(3 * height * width);

    for (int row = 0; row < height; row++)
        for (int col = 0; col < width; col++)
        {
            const auto &pixel = mat.at<Vec3b>(row, col);
            int buffer_idx = ((height - 1 - row) * width + col) * 3;
            array[buffer_idx + 0] = float(pixel[2]) / 255; // R
            array[buffer_idx + 1] = float(pixel[1]) / 255; // G
            array[buffer_idx + 2] = float(pixel[0]) / 255; // B
        }
}