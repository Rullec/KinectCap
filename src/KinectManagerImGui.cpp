#include "KinectManagerImGui.h"
#include "utils/FileUtil.h"
#include "utils/LogUtil.h"
#include "utils/TimeUtil.hpp"
#include "KinectResource.h"
#include <iostream>
const std::vector<const char *> gDisplayModeStr =
    {"only_color",
     "only_depth",
     "color_and_depth",
     "depth_to_color"};
cKinectManagerImGui::cKinectManagerImGui(std::string display_mode)
    : cKinectManager("720p_16_9", "nfov_unbinned")
{
    mDisplayMode = cKinectManagerImGui::BuildDisplayModeFromStr(
        display_mode);
    mCurDepthImage = std::make_shared<cKinectImageResource>();
    mCurColorImage = std::make_shared<cKinectImageResource>();
    mEnableDownsample = true;
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
    printf("begin to init calibration and transformation\n");
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(mDevice, mConfig.depth_mode,
                                   mConfig.color_resolution, &mCalibration))
    {
        printf("Failed to get calibration\n");
        exit(1);
    }

    mCalibratinTrans =
        k4a_transformation_create(&mCalibration);
}

#include "imgui.h"
extern bool SavePNGSingleChannel(const float *depth_pixels, int width, int height,
                                 const char *outfile_name);
int gOutputImageIdx = 0;
const std::string gOutputImagedir = "output";
void cKinectManagerImGui::UpdateGui()
{
    static int color_mode = mColorMode;
    static int depth_mode = mDepthMode;
    static int display_mode = mDisplayMode;
    ImGui::Combo("display mode", &display_mode, gDisplayModeStr.data(),
                 gDisplayModeStr.size());
    ImGui::Combo("depth mode", &depth_mode, gDepthModeStr.data(),
                 gDepthModeStr.size());
    ImGui::Combo("color mode", &color_mode, gColorModeStr.data(),
                 gColorModeStr.size());

    ImGui::Checkbox("downsample", &mEnableDownsample);
    bool capture = ImGui::Button("capture!");
    if (capture)
    {
        bool confirm_output = true;
        if (mEnableDownsample == true)
        {
            SIM_INFO("Downsample is enabled at this moment, and the output image will be effected. please push button again");
            mEnableDownsample = false;
            confirm_output = false;
        }
        if (
            mDisplayMode == eDisplayMode::ONLY_COLOR ||
            mDisplayMode == eDisplayMode::ONLY_DEPTH)
        {
            SIM_INFO("only_color/only_depth is enabled at this moment, please adjust");
            confirm_output = false;
        }
        if (confirm_output)
            ExportCapture();
    }

    mDisplayMode = static_cast<eDisplayMode>(display_mode);
    // SetDepthMode(static_cast<k4a_depth_mode_t>(depth_mode));
    SetColorAndDepthMode(
        static_cast<k4a_depth_mode_t>(depth_mode),
        static_cast<k4a_color_resolution_t>(color_mode));

    ImGui::SameLine();
}
void cKinectManagerImGui::ExportCapture()
{
    if (cFileUtil::ExistsDir(gOutputImagedir) == false)
        cFileUtil::CreateDir(gOutputImagedir.c_str());

    std::string depth_png_name = gOutputImagedir + "/depth" + std::to_string(gOutputImageIdx) + ".png";
    std::string depth_txt_name = gOutputImagedir + "/depth" + std::to_string(gOutputImageIdx) + ".txt";
    std::string color_png_name = gOutputImagedir + "/color" + std::to_string(gOutputImageIdx) + ".png";
    std::string conf_name = gOutputImagedir + "/conf" + std::to_string(gOutputImageIdx) + ".txt";
    ExportDepthToPng(mCurDepthImage->mData.data(), mCurDepthImage->mHeight, mCurDepthImage->mWidth, mCurDepthImage->mChannels, depth_png_name);
    ExportDepthToTxt(mCurDepthImage->mData.data(),
                     mCurDepthImage->mHeight, mCurDepthImage->mWidth, mCurDepthImage->mChannels, depth_txt_name);
    ExportRGBColorToPng(mCurColorImage->mData.data(),
                        mCurColorImage->mHeight, mCurColorImage->mWidth, mCurColorImage->mChannels, color_png_name);
    ExportKinectConfiguration(conf_name);
    SIM_INFO("depth png {}, depth txt {}, color png {}, kinect conf {}",
             depth_png_name,
             depth_txt_name,
             color_png_name,
             conf_name);
    gOutputImageIdx += 1;
}
void cKinectManagerImGui::ExportKinectConfiguration(std::string output_name)
{
    std::ofstream fout(output_name, 'w');
    fout << "display mode = " << gDisplayModeStr[mDisplayMode] << std::endl;
    fout << "color mode = " << gColorModeStr[mColorMode] << std::endl;
    fout << "depth mode = " << gDepthModeStr[mDepthMode] << std::endl;
    fout << "captured at " << cTimeUtil::GetSystemTime() << std::endl;
    fout.close();
}

cKinectImageResourcePtr cKinectManagerImGui::GetDepthImageNew()
{
    if (mDepthMode == K4A_DEPTH_MODE_OFF)
    {
        return nullptr;
    }

    auto capture = GetCapture();
    k4a_image_t image = k4a_capture_get_depth_image(capture);
    mCurDepthImage->ConvertFromKinect(image, mEnableDownsample);
    k4a_image_release(image);
    k4a_capture_release(capture);
    return mCurDepthImage;
}

std::vector<cKinectImageResourcePtr> cKinectManagerImGui::GetRenderingResource()
{
    // cTimeUtil::Begin("get_rendering_resource");
    std::vector<cKinectImageResourcePtr> res = {};
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
        break;
    }
    case eDisplayMode::DEPTH_TO_COLOR:
    {
        res = GetDepthToColorImageNew();
        break;
    }
    default:
        SIM_ERROR("unsupport display mode {}", mDisplayMode);
        exit(1);
        break;
    }

    // cTimeUtil::End("get_rendering_resource");
    return res;
}
#include "utils/TimeUtil.hpp"
std::vector<cKinectImageResourcePtr> cKinectManagerImGui::GetDepthToColorImageNew()
{
    // cTimeUtil::Begin("pre_depth_alignment");
    // cTimeUtil::Begin("pre_depth_alignment1");

    // cTimeUtil::End("pre_depth_alignment1");
    // cTimeUtil::Begin("pre_depth_alignment2");
    // Get a capture
    k4a_capture_t capture = GetCapture();

    // Get a depth image
    auto depth_image = k4a_capture_get_depth_image(capture);
    if (depth_image == 0)
    {
        printf("Failed to get depth image from capture\n");
        exit(1);
    }

    // Get a color image
    auto color_image = k4a_capture_get_color_image(capture);
    if (color_image == 0)
    {
        printf("Failed to get color image from capture\n");
        exit(1);
    }
    // cTimeUtil::End("pre_depth_alignment2");
    // Compute color point cloud by warping depth image into color camera
    // geometry
    // transform color image into depth camera geometry
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
    // cTimeUtil::End("pre_depth_alignment");
    // cTimeUtil::Begin("create_trans_resource");
    k4a_image_t transformed_depth_image = NULL;
    if (K4A_RESULT_SUCCEEDED !=
        k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, color_image_width_pixels,
                         color_image_height_pixels,
                         color_image_width_pixels * (int)sizeof(uint16_t),
                         &transformed_depth_image))
    {
        printf("Failed to create transformed depth image\n");
        exit(1);
    }

    k4a_image_t point_cloud_image = NULL;
    if (K4A_RESULT_SUCCEEDED !=
        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, color_image_width_pixels,
                         color_image_height_pixels,
                         color_image_width_pixels * 3 * (int)sizeof(int16_t),
                         &point_cloud_image))
    {
        printf("Failed to create point cloud image\n");
        exit(1);
    }

    // cTimeUtil::End("create_trans_resource");
    // cTimeUtil::Begin("do_trans");
    if (K4A_RESULT_SUCCEEDED !=
        k4a_transformation_depth_image_to_color_camera(
            mCalibratinTrans, depth_image, transformed_depth_image))
    {
        printf("Failed to compute transformed depth image\n");
        exit(1);
    }

    if (K4A_RESULT_SUCCEEDED !=
        k4a_transformation_depth_image_to_point_cloud(
            mCalibratinTrans, transformed_depth_image,
            K4A_CALIBRATION_TYPE_COLOR, point_cloud_image))
    {
        printf("Failed to compute point cloud\n");
        exit(1);
    }
    // cTimeUtil::End("do_trans");
    // cTimeUtil::Begin("post_depth_alignment");
    mCurDepthImage->ConvertFromKinect(transformed_depth_image, mEnableDownsample);
    mCurColorImage->ConvertFromKinect(color_image, mEnableDownsample);
    k4a_image_release(transformed_depth_image);
    k4a_image_release(point_cloud_image);
    // cTimeUtil::End("post_depth_alignment");
    return {mCurColorImage, mCurDepthImage};
}

cKinectImageResourcePtr cKinectManagerImGui::GetColorImageNew()
{
    if (mColorMode == K4A_COLOR_RESOLUTION_OFF)
    {
        return nullptr;
    }

    auto capture = GetCapture();
    k4a_image_t image = k4a_capture_get_color_image(capture);
    mCurColorImage->ConvertFromKinect(image, mEnableDownsample);
    k4a_image_release(image);
    k4a_capture_release(capture);
    return mCurColorImage;
}
