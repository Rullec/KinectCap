#include "KinectManagerImGui.h"
#include "utils/LogUtil.h"

cKinectImageResource::cKinectImageResource()
{
    mHeight = 0;
    mWidth = 0;
    mChannels = -1;
    mData.resize(0);
}

const std::vector<const char *> gDisplayModeStr =
    {"only_color",
     "only_depth",
     "color_and_depth"};
cKinectManagerImGui::cKinectManagerImGui(std::string display_mode)
    : cKinectManager("720p_16_9", "nfov_unbinned")
{
    mDisplayMode = cKinectManagerImGui::BuildDisplayModeFromStr(
        display_mode);
    mCurDepthImage = std::make_shared<cKinectImageResource>();
    mCurColorImage = std::make_shared<cKinectImageResource>();
}
std::string cKinectManagerImGui::BuildStrFromDisplayMode(eDisplayMode mode)
{
    SIM_ASSERT(mode < eDisplayMode::NUM_OF_DISPLAY_MODE);
    return gDisplayModeStr[mode];
}
eDisplayMode cKinectManagerImGui::BuildDisplayModeFromStr(std::string str)
{
    for (size_t i = 0; i < eDisplayMode::NUM_OF_DISPLAY_MODE; i++)
    {
        // printf("%s == %s: %d\n", gDisplayModeStr[i].c_str(), str.c_str(), gDisplayModeStr[i] == str);
        if (gDisplayModeStr[i] == str)
        {
            return static_cast<eDisplayMode>(i);
        }
    }

    SIM_ERROR("unrecognized display mode {}", str);
    exit(1);
    return eDisplayMode::NUM_OF_DISPLAY_MODE;
}
#include "KinectMode.h"
void cKinectManagerImGui::Init()
{
    cKinectManager::Init();

    // begin to dump the string
    mDisplayModeStr = cKinectManagerImGui ::BuildStrFromDisplayMode(mDisplayMode);
    mDepthModeStr = cKinectMode::BuildStringFromDepthMode(mDepthMode);
    mColorModeStr = cKinectMode::BuildStringFromColorMode(mColorMode);
}

#include "imgui.h"
void cKinectManagerImGui::UpdateGui()
{
    // const char *items[] = {"0", "45", "90"};
    // ;
    static int color_mode = mColorMode;
    static int depth_mode = mDepthMode;
    static int display_mode = mDisplayMode;
    ImGui::Combo("display mode", &display_mode, gDisplayModeStr.data(),
                 gDisplayModeStr.size());
    ImGui::Combo("depth mode", &depth_mode, gDepthModeStr.data(),
                 gDepthModeStr.size());
    ImGui::Combo("color mode", &color_mode, gColorModeStr.data(),
                 gColorModeStr.size());

    SetDepthMode(static_cast<k4a_depth_mode_t>(depth_mode));
    ImGui::SameLine();
}

cKinectImageResourcePtr cKinectManagerImGui::GetDepthImageNew()
{
    if (mDepthMode == K4A_DEPTH_MODE_OFF)
    {
        return nullptr;
    }

    auto capture = GetCapture();
    k4a_image_t image = k4a_capture_get_depth_image(capture);
    mCurDepthImage->ConvertFromKinect(image);
    k4a_image_release(image);
    k4a_capture_release(capture);
    return mCurDepthImage;
}

void cKinectImageResource::ConvertFromKinect(k4a_image_t image)
{
    if (image == NULL)
    {
        mHeight = 0;
        mWidth = 0;
        mChannels = 0;
        mData.clear();
    }

    mHeight = k4a_image_get_height_pixels(image);
    mWidth = k4a_image_get_width_pixels(image);
    int stride_bytes = k4a_image_get_stride_bytes(image);
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
            int kinect_idx = row_id * mWidth + col_id;
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
    int height = k4a_image_get_height_pixels(image);
    int width = k4a_image_get_width_pixels(image);
    uint8_t *image_data = (uint8_t *)(void *)k4a_image_get_buffer(image);
    auto image_format = k4a_image_get_format(image);
    SIM_ASSERT(image_format == K4A_IMAGE_FORMAT_COLOR_BGRA32);

    mChannels = 3;
    mData.resize(3 * height * width);
    for (int row = 0; row < height; row++)
        for (int col = 0; col < width; col++)
        {
            int offset_buffer = (row * width + col) * 4;
            int offset_output_data = ((mHeight - 1 - row) * width + col) * 3;
            // R
            mData[offset_output_data + 0] = float(image_data[offset_buffer + 2]) / 255;
            // G
            mData[offset_output_data + 1] = float(image_data[offset_buffer + 1]) / 255;
            // B
            mData[offset_output_data + 2] = float(image_data[offset_buffer + 0]) / 255;
        }
}

cKinectImageResourcePtr cKinectManagerImGui::GetColorImageNew()
{
    if (mColorMode == K4A_COLOR_RESOLUTION_OFF)
    {
        return nullptr;
    }

    auto capture = GetCapture();
    k4a_image_t image = k4a_capture_get_color_image(capture);
    mCurColorImage->ConvertFromKinect(image);
    k4a_image_release(image);
    k4a_capture_release(capture);
    return mCurColorImage;
}

std::vector<cKinectImageResourcePtr> cKinectManagerImGui::GetRenderingResource()
{
    std::vector<cKinectImageResourcePtr> res = {};
    mDisplayMode = COLOR_AND_DEPTH;
    switch (mDisplayMode)
    {
    case eDisplayMode::COLOR_AND_DEPTH:
    {
        auto color = GetColorImageNew();
        auto depth = GetDepthImageNew();
        if (color != nullptr)
            res.push_back(color);
        if (depth != nullptr)
            res.push_back(depth);
        break;
    }
    case eDisplayMode::ONLY_COLOR:
    {
        auto color = GetColorImageNew();
        if (color != nullptr)
            res.push_back(color);

        break;
    }
    case eDisplayMode::ONLY_DEPTH:
    {
        auto depth = GetDepthImageNew();
        if (depth != nullptr)
            res.push_back(depth);
    }
    break;

    default:
        break;
    }
    return res;
}