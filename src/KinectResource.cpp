#include "KinectResource.h"
#include "utils/LogUtil.h"
cKinectImageResource::cKinectImageResource()
{
    mHeight = 0;
    mWidth = 0;
    mChannels = -1;
    mData.resize(0);
    mEnableDownsampling = true;
}

void cKinectImageResource::ConvertFromKinect(k4a_image_t image, bool enable_downsampling /* = true*/)
{
    mEnableDownsampling = enable_downsampling;
    if (image == NULL)
    {
        mHeight = 0;
        mWidth = 0;
        mChannels = 0;
        mData.clear();
    }

    mHeight = k4a_image_get_height_pixels(image);
    mWidth = k4a_image_get_width_pixels(image);
    if (mEnableDownsampling)
    {
        mHeight /= 2;
        mWidth /= 2;
    }
    k4a_image_format_t cur_format = k4a_image_get_format(image);
    switch (cur_format)
    {
    case K4A_IMAGE_FORMAT_DEPTH16:
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

void cKinectImageResource::ConvertFromDepthImage(k4a_image_t image)
{
    auto image_format = k4a_image_get_format(image);
    SIM_ASSERT(image_format == K4A_IMAGE_FORMAT_DEPTH16);
    uint16_t *buffer_raw = (uint16_t *)(k4a_image_get_buffer(image));
    mChannels = 3; // RGB format
    int total_size = mHeight * mWidth * mChannels;
    if (mData.size() != total_size)
        mData.resize(total_size, 1);

    for (int row_id = 0; row_id < mHeight; row_id++)
    {
        for (int col_id = 0; col_id < mWidth; col_id++)
        {
            // (row, col) pixel value
            int kinect_idx = 0;
            if (mEnableDownsampling == false)
            {

                kinect_idx = row_id * mWidth + col_id;
            }
            else
            {
                kinect_idx = row_id * 2 * (mWidth * 2) + 2 * col_id;
            }

            int output_idx = (mHeight - row_id - 1) * mWidth + col_id;
            float pixel_value = float(buffer_raw[kinect_idx]) / 1e3;
            mData[3 * output_idx + 0] = pixel_value;
            mData[3 * output_idx + 1] = pixel_value;
            mData[3 * output_idx + 2] = pixel_value;
        }
    }
}
void cKinectImageResource::ConvertFromColorImage(k4a_image_t image)
{
    uint8_t *image_data = (uint8_t *)(void *)k4a_image_get_buffer(image);
    auto image_format = k4a_image_get_format(image);
    SIM_ASSERT(image_format == K4A_IMAGE_FORMAT_COLOR_BGRA32);

    mChannels = 3;
    mData.resize(3 * mHeight * mWidth);
    for (int row = 0; row < mHeight; row++)
        for (int col = 0; col < mWidth; col++)
        {
            int kinect_idx = 0;
            if (mEnableDownsampling == false)
            {
                kinect_idx = (row * mWidth + col) * 4;
            }
            else
            {
                kinect_idx = (row * 2 * (mWidth * 2) + 2 * col) * 4;
            }
            int buffer_idx = ((mHeight - 1 - row) * mWidth + col) * 3;
            // R
            mData[buffer_idx + 0] = float(image_data[kinect_idx + 2]) / 255;
            // G
            mData[buffer_idx + 1] = float(image_data[kinect_idx + 1]) / 255;
            // B
            mData[buffer_idx + 2] = float(image_data[kinect_idx + 0]) / 255;
        }
}
