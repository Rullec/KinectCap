#include "KinectManagerImGui.h"
#include "utils/OpenCVUtil.h"
#include "utils/FileUtil.h"
#include "utils/LogUtil.h"
#include "utils/TimeUtil.hpp"
#include "utils/PoseEstimator.h"
#include "kinect/KinectResource.h"
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
    mDebugChessboardResource = std::make_shared<cKinectImageResource>();
    mDepthDiffResource = std::make_shared<cKinectImageResource>();
    mEnableDownsample = true;
    mEnablePoseEstimate = true;
    mEnableShowRaycast = true;
    mDebugDrawChessboard = false;
    mLockEstimation = false;
    mEnableDepthDiff = true;
    mRaycastFov = 60;
    mDepthAdjustBias = 0;
    InitRaycastDepthImage();
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
    mRaycastFov = GetCurDepthVFOV();
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
    ImGui::SameLine();
    ImGui::Checkbox("pose estimation", &mEnablePoseEstimate);
    ImGui::SameLine();
    ImGui::Checkbox("show raycast", &mEnableShowRaycast);
    ImGui::SameLine();
    ImGui::Checkbox("draw chessbox", &mDebugDrawChessboard);
    ImGui::NewLine();
    ImGui::Checkbox("depth diff", &mEnableDepthDiff);
    ImGui::SameLine();
    if (mLockEstimation == true)
    {
        ImGui::Checkbox("cam pos locked", &mLockEstimation);
    }
    else
    {
        ImGui::Checkbox("cam pos unlocked", &mLockEstimation);
    }
    if (mEnableShowRaycast)
    {
        ImGui::DragFloat("raycast fov", &mRaycastFov, 1e-1, 40, 80);
    }
    ImGui::DragFloat("depth adjust", &mDepthAdjustBias, 1, -50, 50);
    bool capture = ImGui::Button("capture!");
    ImGui::SameLine();
    ImGui::Text("recommend depth fov %.1f", GetCurDepthVFOV());
    // cTimeUtil::Begin("pose_est");
    if (mEnablePoseEstimate)
    {
        if (mLockEstimation == false)
        {
            PoseEstimate();
        }
    }
    else
    {
        mEstimatedCamPos.setZero();
        mEstimatedCamFocus.setZero();
        mEstimatedCamUp.setZero();
    }
    ImGui::Text("cam_pos = %.1f, %.1f, %.1f, ", cam_pos[0], cam_pos[1], cam_pos[2]);
    ImGui::Text("cam_focus = %.1f, %.1f, %.1f", cam_focus[0], cam_focus[1], cam_focus[2]);
    ImGui::Text("cam_up = %.2f, %.2f, %.2f", cam_up[0], cam_up[1], cam_up[2]);
    // cTimeUtil::End("pose_est");

    if (capture)
        ExportCapture();
    if (display_mode != mDisplayMode)
    {
        mDisplayMode = static_cast<eDisplayMode>(display_mode);
        mRaycastFov = GetCurDepthVFOV();
    }
    // SetDepthMode(static_cast<k4a_depth_mode_t>(depth_mode));
    SetColorAndDepthMode(
        static_cast<k4a_depth_mode_t>(depth_mode),
        static_cast<k4a_color_resolution_t>(color_mode));

    ImGui::SameLine();
}
void cKinectManagerImGui::ExportCapture()
{
    bool confirm_output = true;
    if (
        mDisplayMode == eDisplayMode::ONLY_COLOR ||
        mDisplayMode == eDisplayMode::ONLY_DEPTH)
    {
        SIM_INFO("only_color/only_depth is enabled at this moment, please adjust");
        confirm_output = false;
    }
    if (confirm_output)
    {
        if (cFileUtil::ExistsDir(gOutputImagedir) == false)
            cFileUtil::CreateDir(gOutputImagedir.c_str());

        std::string depth_png_name = gOutputImagedir + "/depth" + std::to_string(gOutputImageIdx) + ".png";
        std::string depth_txt_name = gOutputImagedir + "/depth" + std::to_string(gOutputImageIdx) + ".txt";
        std::string color_png_name = gOutputImagedir + "/color" + std::to_string(gOutputImageIdx) + ".png";
        std::string conf_name = gOutputImagedir + "/conf" + std::to_string(gOutputImageIdx) + ".txt";
        ExportDepthToPng(mCurDepthImage->mRawData.data(), mCurDepthImage->mRawHeight, mCurDepthImage->mRawWidth, mCurDepthImage->mChannels, depth_png_name);
        ExportDepthToTxt(mCurDepthImage->mRawData.data(),
                         mCurDepthImage->mRawHeight, mCurDepthImage->mRawWidth, mCurDepthImage->mChannels, depth_txt_name);
        ExportRGBColorToPng(mCurColorImage->mRawData.data(),
                            mCurColorImage->mRawHeight, mCurColorImage->mRawWidth, mCurColorImage->mChannels, color_png_name);
        ExportKinectConfiguration(conf_name);
        SIM_INFO("depth png {}, depth txt {}, color png {}, kinect conf {}",
                 depth_png_name,
                 depth_txt_name,
                 color_png_name,
                 conf_name);
        gOutputImageIdx += 1;
    }
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
    mCurDepthImage->ConvertFromKinect(image, mEnableDownsample, mDepthAdjustBias);
    k4a_image_release(image);
    k4a_capture_release(capture);
    return mCurDepthImage;
}

std::vector<cKinectImageResourcePtr> cKinectManagerImGui::GetRenderingResource()
{
    // cTimeUtil::Begin("get_rendering_resource");
    // cTimeUtil::Begin("get_render_res");
    std::vector<cKinectImageResourcePtr> res = {};
    switch (mDisplayMode)
    {
    case eDisplayMode::COLOR_AND_DEPTH:
    {
        mCurColorImage = GetColorImageNew();
        mCurDepthImage = GetDepthImageNew();
        if (mCurColorImage != nullptr)
            res.push_back(mCurColorImage);
        if (mCurDepthImage != nullptr)
            res.push_back(mCurDepthImage);
        break;
    }
    case eDisplayMode::ONLY_COLOR:
    {
        mCurColorImage = GetColorImageNew();
        if (mCurColorImage != nullptr)
            res.push_back(mCurColorImage);
        mCurDepthImage->Reset();
        break;
    }
    case eDisplayMode::ONLY_DEPTH:
    {
        mCurDepthImage = GetDepthImageNew();
        if (mCurDepthImage != nullptr)
            res.push_back(mCurDepthImage);
        mCurColorImage->Reset();
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
    // cTimeUtil::End("get_render_res");
    if (mEnableShowRaycast == true)
    {
        // cTimeUtil::Begin("update_raycast");
        UpdateRaycastResult(mCurDepthImage);
        res.push_back(mRaycastDepthResource);
        // cTimeUtil::End("update_raycast");
        if (mEnableDepthDiff == true)
        {
            if (mCurDepthImage->mRawHeight != 0)
            {
                UpdateDepthDiffResult(mDepthDiffResource);
                res.push_back(mDepthDiffResource);
            }
        }
    }

    if (mDebugDrawChessboard == true && mDebugChessboardResource->mPresentHeight != 0)
    {
        res.push_back(mDebugChessboardResource);
    }

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
        return {};
    }

    // Get a color image
    auto color_image = k4a_capture_get_color_image(capture);
    if (color_image == 0)
    {
        printf("Failed to get color image from capture\n");
        return {};
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
        return {};
    }

    k4a_image_t point_cloud_image = NULL;
    if (K4A_RESULT_SUCCEEDED !=
        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, color_image_width_pixels,
                         color_image_height_pixels,
                         color_image_width_pixels * 3 * (int)sizeof(int16_t),
                         &point_cloud_image))
    {
        printf("Failed to create point cloud image\n");
        return {};
    }

    // cTimeUtil::End("create_trans_resource");
    // cTimeUtil::Begin("do_trans");
    if (K4A_RESULT_SUCCEEDED !=
        k4a_transformation_depth_image_to_color_camera(
            mCalibratinTrans, depth_image, transformed_depth_image))
    {
        printf("Failed to compute transformed depth image\n");
        return {};
    }

    if (K4A_RESULT_SUCCEEDED !=
        k4a_transformation_depth_image_to_point_cloud(
            mCalibratinTrans, transformed_depth_image,
            K4A_CALIBRATION_TYPE_COLOR, point_cloud_image))
    {
        printf("Failed to compute point cloud\n");
        return {};
    }
    // cTimeUtil::End("do_trans");
    // cTimeUtil::Begin("post_depth_alignment");
    mCurDepthImage->ConvertFromKinect(transformed_depth_image, mEnableDownsample, mDepthAdjustBias);
    mCurColorImage->ConvertFromKinect(color_image, mEnableDownsample);
    k4a_image_release(transformed_depth_image);
    k4a_image_release(point_cloud_image);
    k4a_image_release(depth_image);
    k4a_image_release(color_image);
    k4a_capture_release(capture);
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

void cKinectManagerImGui::PoseEstimate()
{
    if (mCurColorImage->mPresentData.size() != 0)
    {
        cv::Mat rgb_mat = cOpencvUtil::ConvertFloatArrayToRGBMat(mCurColorImage->mPresentHeight,
                                                                 mCurColorImage->mPresentWidth, mCurColorImage->mPresentData.data());

        tMatrix3d mtx = mColorIntri.mCamMtx;
        tVectorXd dist_coef = mColorIntri.mDistCoef;
        cv::Mat debug_color_mat;
        tVector cam_pos, cam_focus, cam_up;
        if (mCurColorImage->mEnableDownsampling)
        {
            /*
            https://answers.opencv.org/question/118918/does-the-resolution-of-an-image-affect-the-distortion-co-efficients/
            https://dsp.stackexchange.com/questions/6055/how-does-resizing-an-image-affect-the-intrinsic-camera-matrix

            if downsample, mtx/=2, coef keep the same
            */
            mtx /= 2;
        }

        cPoseEstimator::Estimate(
            rgb_mat, mtx, dist_coef, debug_color_mat, cam_pos, cam_focus, cam_up);

        mEstimatedCamPos = cam_pos.segment(0, 3).cast<float>();
        mEstimatedCamFocus = cam_focus.segment(0, 3).cast<float>();
        mEstimatedCamUp = cam_up.segment(0, 3).cast<float>();

        if (cam_pos.norm() < 1e-6)
        {
            mDebugChessboardResource->Reset();
        }
        else if (mDebugDrawChessboard)
        {
            // convert the debug color mat to rendering resource
            cOpencvUtil::ConvertRGBMatToFloatArray(debug_color_mat, mDebugChessboardResource->mPresentHeight, mDebugChessboardResource->mPresentWidth, mDebugChessboardResource->mPresentData);
        }
        // std::cout << "cam pos = " << cam_pos.transpose() << ", cam focus = " << cam_focus.transpose() << std::endl;
    }
    else
    {
        std::cout << "no color resource\n";
        mDebugChessboardResource->Reset();
    }
}
#include "raycast/Raycast.h"
void cKinectManagerImGui::InitRaycastDepthImage()
{
    mRaycaster = std::make_shared<cDepthImageCaster>();
    mRaycastDepthResource = std::make_shared<cKinectImageResource>();
}

void cKinectManagerImGui::UpdateRaycastResult(cKinectImageResourcePtr depth_result)
{
    if (depth_result->mRawHeight == 0)
    {
        std::cout << "depth invalid, cannot update raycast\n";
        return;
    }

    if (mEstimatedCamPos.norm() < 1e-5)
    {
        // std::cout << "estimated cam pos unavailable, cannot update raycast\n";
        return;
    }
    // cTimeUtil::Begin("init_raycast");
    int height = depth_result->mPresentHeight, width = depth_result->mPresentWidth;

    mRaycastDepthResource = std::make_shared<cKinectImageResource>();

    mRaycastDepthResource->mPresentHeight = height;
    mRaycastDepthResource->mPresentWidth = width;
    mRaycastDepthResource->mChannels = 3;
    mRaycaster->CalcResult(mEstimatedCamPos * 1e-3, mEstimatedCamFocus * 1e-3, mEstimatedCamUp * 1e-3, mRaycastFov, height, width, mRaycastDepthResource->mPresentData);
    // cTimeUtil::End("init_raycast");
}

void cKinectManagerImGui::UpdateDepthDiffResult(cKinectImageResourcePtr diff_res)
{
    diff_res->mPresentHeight = mRaycastDepthResource->mPresentHeight;
    diff_res->mPresentWidth = mRaycastDepthResource->mPresentWidth;
    diff_res->mChannels = mRaycastDepthResource->mChannels;
    int size = diff_res->mPresentHeight * diff_res->mPresentWidth * diff_res->mChannels;
    diff_res->mPresentData.resize(size);

    for (int i = 0; i < size; i++)
        diff_res->mPresentData[i] = std::fabs(mCurDepthImage->mPresentData[i] -
                                              mRaycastDepthResource->mPresentData[i]);
}

double cKinectManagerImGui::GetCurDepthVFOV()
{
    if (this->mDisplayMode == eDisplayMode::DEPTH_TO_COLOR)
    {
        return GetColorVFOV(this->mColorMode);
    }
    else
    {

        return GetDepthVFOV(this->mDepthMode);
    }
    return -1;
}

void cKinectManagerImGui::SetColorAndDepthMode(k4a_depth_mode_t new_depth_mode, k4a_color_resolution_t new_color_mode)
{
    if (new_depth_mode != mDepthMode || new_color_mode != mColorMode)
    {
        cKinectManager::SetColorAndDepthMode(new_depth_mode, new_color_mode);
        mRaycastFov = GetCurDepthVFOV();
    }
}