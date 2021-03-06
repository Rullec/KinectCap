#include "KinectResource.h"
#include "utils/LogUtil.h"
cKinectImageResource::cKinectImageResource()
{
    mPresentHeight = 0;
    mPresentWidth = 0;
    mChannels = -1;
    mPresentData.resize(0);
    mRawData.resize(0);
    mRawHeight = 0;
    mRawWidth = 0;
    mEnableDownsampling = true;
}
void cKinectImageResource::Reset()
{
    mPresentHeight = 0;
    mPresentWidth = 0;
    mRawHeight = 0;
    mRawWidth = 0;
    mChannels = -1;
    mPresentData.resize(0);
    mRawData.resize(0);
}
void cKinectImageResource::ConvertFromKinect(k4a_image_t image, bool enable_downsampling /* = true*/, float value_adjust /*= 0*/)
{
    mValueAdjust = value_adjust;
    mEnableDownsampling = enable_downsampling;
    if (image == NULL)
    {
        Reset();
        return;
    }

    mRawHeight = k4a_image_get_height_pixels(image);
    mRawWidth = k4a_image_get_width_pixels(image);
    mPresentHeight = mRawHeight;
    mPresentWidth = mRawWidth;
    if (mEnableDownsampling)
    {
        mPresentHeight /= 2;
        mPresentWidth /= 2;
    }
    k4a_image_format_t cur_format = k4a_image_get_format(image);
    switch (cur_format)
    {
    case K4A_IMAGE_FORMAT_DEPTH16:
    case K4A_IMAGE_FORMAT_IR16:
        ConvertFromDepthImage(image);
        break;
    case K4A_IMAGE_FORMAT_COLOR_BGRA32:
        ConvertFromColorImage(image);
        break;

    default:
        SIM_ERROR("unsupported format {}", cur_format);
        break;
    }
}
#include <iostream>
void cKinectImageResource::ConvertFromDepthImage(k4a_image_t image)
{
    auto image_format = k4a_image_get_format(image);
    SIM_ASSERT(image_format == K4A_IMAGE_FORMAT_DEPTH16 || image_format == K4A_IMAGE_FORMAT_IR16);
    uint16_t *kinect_buffer = (uint16_t *)(k4a_image_get_buffer(image));
    mChannels = 3; // RGB format
    int present_data_size = mPresentHeight * mPresentWidth * mChannels;
    if (mPresentData.size() != present_data_size)
        mPresentData.resize(present_data_size, 1);

    // std::cout << "convert from depth, depth adjust = " << mValueAdjust << std::endl;
    for (int row_id = 0; row_id < mPresentHeight; row_id++)
    {
        for (int col_id = 0; col_id < mPresentWidth; col_id++)
        {
            // (row, col) pixel value
            int kinect_idx = 0;
            if (mEnableDownsampling)
            {
                kinect_idx = row_id * 2 * mPresentWidth * 2 + col_id * 2;
            }
            else
            {
                kinect_idx = row_id * mPresentWidth + col_id;
            }

            // set up present data
            {
                int present_output_idx = (mPresentHeight - row_id - 1) * mPresentWidth + col_id;
                float pixel_value = (float(kinect_buffer[kinect_idx]) + mValueAdjust) / 1e3;
                if (pixel_value < 0)
                    pixel_value = 0;
                mPresentData[3 * present_output_idx + 0] = pixel_value;
                mPresentData[3 * present_output_idx + 1] = pixel_value;
                mPresentData[3 * present_output_idx + 2] = pixel_value;
            }
        }
    }
    ConvertFromDepthImageRaw(image);
}

void cKinectImageResource::ConvertFromDepthImageRaw(k4a_image_t image)
{
    auto image_format = k4a_image_get_format(image);
    SIM_ASSERT(image_format == K4A_IMAGE_FORMAT_DEPTH16 || image_format == K4A_IMAGE_FORMAT_IR16);
    uint16_t *kinect_buffer = (uint16_t *)(k4a_image_get_buffer(image));
    mChannels = 3; // RGB format
    int raw_data_size = mRawHeight * mRawWidth * mChannels;
    if (mRawData.size() != raw_data_size)
        mRawData.resize(raw_data_size, 1);

    for (int row_id = 0; row_id < mRawHeight; row_id++)
    {
        for (int col_id = 0; col_id < mRawWidth; col_id++)
        {
            // (row, col) pixel value
            int kinect_idx = row_id * mRawWidth + col_id;

            // set up raw data
            {
                int raw_output_idx = (mRawHeight - row_id - 1) * mRawWidth + col_id;
                float pixel_value = (float(kinect_buffer[kinect_idx]) + mValueAdjust) / 1e3;
                if (pixel_value < 0)
                    pixel_value = 0;
                mRawData[3 * raw_output_idx + 0] = pixel_value;
                mRawData[3 * raw_output_idx + 1] = pixel_value;
                mRawData[3 * raw_output_idx + 2] = pixel_value;
            }
        }
    }
}
void cKinectImageResource::ConvertFromColorImage(k4a_image_t image)
{
    uint8_t *image_data = (uint8_t *)(void *)k4a_image_get_buffer(image);
    auto image_format = k4a_image_get_format(image);
    SIM_ASSERT(image_format == K4A_IMAGE_FORMAT_COLOR_BGRA32);

    mChannels = 3;
    mPresentData.resize(3 * mPresentHeight * mPresentWidth);
    for (int row = 0; row < mPresentHeight; row++)
        for (int col = 0; col < mPresentWidth; col++)
        {
            int kinect_idx = 0;
            if (mEnableDownsampling == false)
            {
                kinect_idx = (row * mPresentWidth + col) * 4;
            }
            else
            {
                kinect_idx = (row * 2 * (mPresentWidth * 2) + 2 * col) * 4;
            }
            int buffer_idx = ((mPresentHeight - 1 - row) * mPresentWidth + col) * 3;
            // R
            mPresentData[buffer_idx + 0] = float(image_data[kinect_idx + 2]) / 255;
            // G
            mPresentData[buffer_idx + 1] = float(image_data[kinect_idx + 1]) / 255;
            // B
            mPresentData[buffer_idx + 2] = float(image_data[kinect_idx + 0]) / 255;
        }
    ConvertFromColorImageRaw(image);
}

void cKinectImageResource::ConvertFromColorImageRaw(k4a_image_t image)
{
    uint8_t *image_data = (uint8_t *)(void *)k4a_image_get_buffer(image);
    auto image_format = k4a_image_get_format(image);
    SIM_ASSERT(image_format == K4A_IMAGE_FORMAT_COLOR_BGRA32);

    mChannels = 3;
    mRawData.resize(3 * mRawHeight * mRawWidth);
    for (int row = 0; row < mRawHeight; row++)
        for (int col = 0; col < mRawWidth; col++)
        {
            int kinect_idx = (row * mRawWidth + col) * 4;
            int buffer_idx = ((mRawHeight - 1 - row) * mRawWidth + col) * 3;
            // R
            mRawData[buffer_idx + 0] = float(image_data[kinect_idx + 2]) / 255;
            // G
            mRawData[buffer_idx + 1] = float(image_data[kinect_idx + 1]) / 255;
            // B
            mRawData[buffer_idx + 2] = float(image_data[kinect_idx + 0]) / 255;
        }
}
#define STB_IMAGE_WRITE_IMPLEMETATION
#include "utils/stb_image_write.h"
#include "utils/FileUtil.h"
std::vector<uint8_t> stb_export_buf;

/**
 * \brief       
 *      Input raw_buf, unit is [m]
 *      Output png, unit is [m * 255.99]
 * 
*/
void ExportDepthToPng(float *raw_buf, int height, int width, int buf_channels, std::string output_name)
{
    if (cFileUtil::ValidateFilePath(output_name) == false)
    {
        SIM_ERROR("invalida file path when export depth to png {}", output_name);
        exit(1);
    }
    stb_export_buf.resize(height * width);
    // row major
    for (int row = 0; row < height; row++)
        for (int col = 0; col < width; col++)
        {
            int raw_buf_idx = (row * width + col) * buf_channels;
            int stb_buf_idx = ((height - 1 - row) * width + col);
            int val = raw_buf[raw_buf_idx] * 255.99f;
            if (val < 0)
                val = 0;
            if (val > 255)
                val = 0;
            stb_export_buf[stb_buf_idx] = uint8_t(val);
        }

    stbi_write_png(output_name.c_str(), width, height, 1, stb_export_buf.data(), width * sizeof(uint8_t));
}

/**
 * \brief   
 *      Output depth value as txt, unit is [m]
*/
#include <iomanip>
void ExportDepthToTxt(float *raw_buf, int height, int width, int buf_channels, std::string output_name)
{
    if (cFileUtil::ValidateFilePath(output_name) == false)
    {
        SIM_ERROR("invalida file path when export depth to png {}", output_name);
        exit(1);
    }
    std::ofstream fout(output_name, 'w');
    fout << "width " << width << " height " << height << " unit "
         << "meter\n";
    // row major
    fout << std::setprecision(6);
    for (int row = 0; row < height; row++)
    {
        for (int col = 0; col < width; col++)
        {
            int raw_buf_idx = ((height - 1 - row) * width + col) * buf_channels;
            fout << raw_buf[raw_buf_idx] << " ";
            // uint32_t val = raw_buf[raw_buf_idx] * 255.99f;
            // if (val > 255)
            //     val = 0;
            // stb_export_buf[stb_buf_idx] = uint8_t(val);
        }
        fout << std::endl;
    }
}

void ExportRGBColorToPng(float *buf, int height, int width, int buf_channels, std::string output_name)
{
    if (cFileUtil::ValidateFilePath(output_name) == false)
    {
        SIM_ERROR("invalida file path when export depth to png {}", output_name);
        exit(1);
    }
    stb_export_buf.resize(height * width * 3);
    // row major
    for (int row = 0; row < height; row++)
        for (int col = 0; col < width; col++)
        {
            int raw_buf_idx_st = (row * width + col) * buf_channels;
            int stb_buf_idx = ((height - 1 - row) * width + col) * 3;

            for (int j = 0; j < 3; j++)
            {
                stb_export_buf[stb_buf_idx + j] = uint8_t(buf[raw_buf_idx_st + j] * 255.99);
            }
        }

    stbi_write_png(output_name.c_str(), width, height, 3, stb_export_buf.data(), width * sizeof(uint8_t) * 3);
}
#include "utils/OpenCVUtil.h"
void cKinectImageResource::ConvertFromOpencv(const cv::Mat &image, bool enable_downsampling /*= true*/)
{
    mEnableDownsampling = enable_downsampling;
    // raw data fill
    mRawHeight = image.rows;
    mRawWidth = image.cols;
    mChannels = 3;
    SIM_ASSERT(image.type() == CV_8UC3);
    mValueAdjust = 0;
    mRawData.resize(mRawHeight * mRawWidth * mChannels);
    for (int row_id = 0; row_id < mRawHeight; row_id++)
        for (int col_id = 0; col_id < mRawWidth; col_id++)
        {
            auto pt = image.at<cv::Vec3b>(row_id, col_id);
            int output_idx = (mRawHeight - 1 - row_id) * mRawWidth + col_id;
            mRawData[output_idx * 3 + 0] = float(pt[2]) / 255.0;
            mRawData[output_idx * 3 + 1] = float(pt[1]) / 255.0;
            mRawData[output_idx * 3 + 2] = float(pt[0]) / 255.0;
        }

    // present data fill
    if (enable_downsampling == true)
    {
        mPresentWidth = mRawWidth / 2;
        mPresentHeight = mRawHeight / 2;
        mPresentData.resize(mPresentHeight * mPresentWidth * mChannels);
        for (int row_id = 0; row_id < mPresentHeight; row_id++)
            for (int col_id = 0; col_id < mPresentWidth; col_id++)
            {
                int output_idx = (mPresentHeight - 1 - row_id) * mPresentWidth + col_id;
                int buf_row_id = 2 * row_id;
                int buf_col_id = 2 * col_id;
                auto pt = image.at<cv::Vec3b>(buf_row_id, buf_col_id);

                mPresentData[output_idx * 3 + 0] = float(pt[2]) / 255.0;
                mPresentData[output_idx * 3 + 1] = float(pt[1]) / 255.0;
                mPresentData[output_idx * 3 + 2] = float(pt[0]) / 255.0;
            }
    }
    else
    {
        mPresentWidth = mRawWidth;
        mPresentHeight = mRawHeight;
        mPresentData = mRawData;
    }
}

void cKinectImageResource::DimmedByWindow(const tVector2i &st_pos_, const tVector2i &window_size_)
{
    // only dim the present data
    tVector2i st_pos = st_pos_;
    tVector2i window_size = window_size_;
    if (mEnableDownsampling)
    {
        st_pos /= 2;
        window_size /= 2;
    }
    double dim_scale = 0.5;
    // if enable down sampling, window_size /= 2, st_pos /= 2
    int height_st = st_pos[0], height_ed = height_st + window_size[0];
    int width_st = st_pos[1], width_ed = width_st + window_size[1];
    for (int row_id = 0; row_id < mPresentHeight; row_id++)
    {
        for (int col_id = 0; col_id < mPresentWidth; col_id++)
        {
            if (
                (row_id < height_st) ||
                (row_id > height_ed) ||
                (col_id < width_st) ||
                (col_id > width_ed))
            {
                int output_idx = (mPresentHeight - 1 - row_id) * mPresentWidth + col_id;
                mPresentData[3 * output_idx + 0] *= dim_scale;
                mPresentData[3 * output_idx + 1] *= dim_scale;
                mPresentData[3 * output_idx + 2] *= dim_scale;
            }
        }
    }
}

cv::Mat cKinectImageResource::ConvertToOpencvPresented(const tVector2i &window_st_,
                                                       const tVector2i &window_size_)
{
    tVector2i window_st = window_st_;
    tVector2i window_size = window_size_;
    if (mEnableDownsampling)
        window_st /= 2, window_size /= 2;
    int window_height = window_size[0];
    int window_width = window_size[1];
    cv::Mat cv_image(window_height, window_width, CV_8UC3);
    // for (int row_id = 0; row_id < mPresentHeight; row_id++)
    //     for (int col_id = 0; col_id < mPresentWidth; col_id++)
    //     {
    //         int buf_row_id = row_id;
    //         int buf_col_id = col_id;

    //         auto vec = cv_image.at<cv::Vec3b>(buf_row_id, buf_col_id);
    //         int output_idx = (mPresentHeight - 1 - row_id) * mPresentWidth + col_id;

    //         vec[2] = uint8_t(mPresentData[output_idx * 3 + 0] * 255);
    //         vec[1] = uint8_t(mPresentData[output_idx * 3 + 1] * 255);
    //         vec[0] = uint8_t(mPresentData[output_idx * 3 + 2] * 255);
    //     }
    for (int row_id = 0; row_id < window_height; row_id++)
    {
        for (int col_id = 0; col_id < window_width; col_id++)
        {
            // 1. get the target present image idx
            int present_row_id = row_id + window_st[0];
            int present_col_id = col_id + window_st[1];
            int present_buf_id = (mPresentHeight - present_row_id - 1) * mPresentWidth + present_col_id;
            cv::Vec3b &vec = cv_image.at<cv::Vec3b>(row_id, col_id);
            vec[2] = uint8_t(mPresentData[present_buf_id * 3 + 0] * 255);
            vec[1] = uint8_t(mPresentData[present_buf_id * 3 + 1] * 255);
            vec[0] = uint8_t(mPresentData[present_buf_id * 3 + 2] * 255);
        }
    }
    return cv_image;
}

void cKinectImageResource::ConvertFromAnotherResource(cKinectImageResourcePtr ptr)
{
    mPresentHeight = ptr->mPresentHeight, mPresentWidth = ptr->mPresentWidth;
    mRawHeight = ptr->mRawHeight, mRawWidth = ptr->mRawWidth;
    mChannels = ptr->mChannels;
    mEnableDownsampling = ptr->mEnableDownsampling;
    mValueAdjust = ptr->mValueAdjust;
    mPresentData = ptr->mPresentData;
    mRawData = ptr->mRawData;
}
void cKinectImageResource::ApplyValueMask(float mask_value)
{
    OMP_PARALLEL_FOR
    for (int i = 0; i < mPresentData.size(); i++)
    {
        if (mPresentData[i] > mask_value)
            mPresentData[i] = 0;
    }
    OMP_PARALLEL_FOR
    for (int i = 0; i < mRawData.size(); i++)
    {
        if (mRawData[i] > mask_value)
            mRawData[i] = 0;
    }
}