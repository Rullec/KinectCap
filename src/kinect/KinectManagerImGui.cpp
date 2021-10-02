#include "KinectManagerImGui.h"
#include "utils/OpenCVUtil.h"
#include "utils/FileUtil.h"
#include "utils/LogUtil.h"
#include "utils/TimeUtil.hpp"
#include "utils/PoseEstimator.h"
#include "kinect/KinectResource.h"
#include "render/ColorDepthFixer.h"
#include <iostream>
const std::vector<const char *> gDisplayModeStr =
    {"only_color",
     "only_depth",
     "color_and_depth",
     "depth_to_color"};

const std::vector<const char *> gCamPoseEstimationMode = {
    "calib_from_color",
    "calib_from_ir"};
cKinectManagerImGui::cKinectManagerImGui(std::string display_mode)
    : cKinectManager("720p_16_9", "nfov_unbinned")
// : cKinectManager("720p_16_9", "passive_ir")
{
    mDisplayMode = cKinectManagerImGui::BuildDisplayModeFromStr(
        display_mode);
    mCurDepthImage = std::make_shared<cKinectImageResource>();
    mCurColorImage = std::make_shared<cKinectImageResource>();
    mDebugChessboardResource = std::make_shared<cKinectImageResource>();
    mDepthDiffResource = std::make_shared<cKinectImageResource>();

    // for undistortion and comparision
    mUndistortDepthRes = std::make_shared<cKinectImageResource>();
    mUndistortColorRes = std::make_shared<cKinectImageResource>();
    mUndistortDepthDiff = std::make_shared<cKinectImageResource>();
    mUndistortColorDiff = std::make_shared<cKinectImageResource>();
    mDepthFixed = std::make_shared<cKinectImageResource>();
    mDepthFixedRemoval = std::make_shared<cKinectImageResource>();

    mEnableDownsample = true;
    mEnablePoseEstimate = false;
    mEnableShowRaycast = false;
    mDebugDrawChessboard = false;
    mLockEstimation = false;
    mEnableDepthDiff = false;
    mPoseEstimateMode = 0;
    mRaycastFov = 60;
    mDepthAdjustBias = 0;
    mEnableCompareDistort = false;
    mEnableDepthFix = true;
    mEnableWindowed = true;
    mEnableDepthRemoval = true;
    mDepthRemovalValue_mm = 1000;
    mWindowSt[0] = 90 * 2;
    mWindowSt[1] = 120 * 2;
    mWindowSize[0] = 240 * 2;
    mWindowSize[1] = 360 * 2;
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
std::vector<std::string> SplitString(std::string raw_str, std::string deli)
{
    size_t pos = 0;
    std::string token;
    std::vector<std::string> strs{};
    while ((pos = raw_str.find(deli)) != std::string::npos)
    {
        token = raw_str.substr(0, pos);
        strs.push_back(token);

        raw_str.erase(0, pos + deli.length());
    }
    return strs;
}
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

    ImGui::SameLine();
    ImGui::Checkbox("show_comp undistorted", &mEnableCompareDistort);
    ImGui::NewLine();
    ImGui::Checkbox("enable windowed", &mEnableWindowed);

    if (mEnableWindowed)
    {
        ImGui::SameLine();
        mEnableDepthFix = ImGui::Button("refresh depth fix");
        ImGui::SameLine();
        ImGui::Checkbox("enable depth removal", &mEnableDepthRemoval);
        if (mEnableDepthRemoval)
        {
            ImGui::DragInt("depth removal", &mDepthRemovalValue_mm, 1, 300, 1000);
        }
        ImGui::DragInt2("window start", mWindowSt.data());
    }
    if (mEnableShowRaycast)
    {
        ImGui::DragFloat("raycast fov", &mRaycastFov, 1e-1, 50, 120);
    }
    ImGui::DragFloat("depth adjust", &mDepthAdjustBias, 1, -50, 50);
    bool capture = ImGui::Button("capture!");
    ImGui::SameLine();
    ImGui::Text("recommend depth fov %.1f", GetCurDepthVFOV());
    // cTimeUtil::Begin("pose_est");
    if (mEnablePoseEstimate)
    {
        ImGui::Combo("estimate_mode", &mPoseEstimateMode, gCamPoseEstimationMode.data(), gCamPoseEstimationMode.size());
        if (mLockEstimation == false)
        {
            if (mPoseEstimateMode == 0)
            {

                PoseEstimateFromColor();
            }
            else if (mPoseEstimateMode == 1)
            {
                PoseEstimateFromIR();
            }
            else
            {
                SIM_ERROR("unsupported pose estimation mode {}", mPoseEstimateMode);
            }
        }
    }
    else
    {
        mEstimatedCamPos.setZero();
        mEstimatedCamFocus.setZero();
        mEstimatedCamUp.setZero();
    }
    ImGui::Text("cam_pos = %.1f, %.1f, %.1f, ", mEstimatedCamPos[0], mEstimatedCamPos[1], mEstimatedCamPos[2]);
    ImGui::Text("cam_focus = %.1f, %.1f, %.1f", mEstimatedCamFocus[0], mEstimatedCamFocus[1], mEstimatedCamFocus[2]);
    ImGui::Text("cam_up = %.2f, %.2f, %.2f", mEstimatedCamUp[0], mEstimatedCamUp[1], mEstimatedCamUp[2]);
    ShowCamResolution();
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
    // if (
    //     mDisplayMode == eDisplayMode::ONLY_COLOR ||
    //     mDisplayMode == eDisplayMode::ONLY_DEPTH)
    // {
    //     SIM_INFO("only_color/only_depth is enabled at this moment, please adjust");
    //     confirm_output = false;
    // }
    if (confirm_output)
    {
        if (cFileUtil::ExistsDir(gOutputImagedir) == false)
            cFileUtil::CreateDir(gOutputImagedir.c_str());

        std::string depth_png_name = gOutputImagedir + "/depth" + std::to_string(gOutputImageIdx) + ".png";
        std::string depth_txt_name = gOutputImagedir + "/depth" + std::to_string(gOutputImageIdx) + ".txt";
        std::string color_png_name = gOutputImagedir + "/color" + std::to_string(gOutputImageIdx) + ".png";
        std::string conf_name = gOutputImagedir + "/conf" + std::to_string(gOutputImageIdx) + ".txt";

        if (mCurDepthImage->mRawHeight != 0)
        {
            ExportDepthToPng(mCurDepthImage->mRawData.data(), mCurDepthImage->mRawHeight, mCurDepthImage->mRawWidth, mCurDepthImage->mChannels, depth_png_name);
            ExportDepthToTxt(mCurDepthImage->mRawData.data(),
                             mCurDepthImage->mRawHeight, mCurDepthImage->mRawWidth, mCurDepthImage->mChannels, depth_txt_name);
        }
        if (mCurColorImage->mRawHeight != 0)
            ExportRGBColorToPng(mCurColorImage->mRawData.data(),
                                mCurColorImage->mRawHeight, mCurColorImage->mRawWidth, mCurColorImage->mChannels, color_png_name);
        if (mEnableWindowed == true)
        {
            std::string window_color_png_name = gOutputImagedir + "/window_color" + std::to_string(gOutputImageIdx) + ".png";
            std::string window_depth_txt_name = gOutputImagedir + "/window_depth" + std::to_string(gOutputImageIdx) + ".txt";
            std::string window_depth_png_name = gOutputImagedir + "/window_depth" + std::to_string(gOutputImageIdx) + ".png";
            std::string window_depth_fixed_txt_name = gOutputImagedir + "/window_depth_fixed" + std::to_string(gOutputImageIdx) + ".txt";
            std::string window_depth_fixed_png_name = gOutputImagedir + "/window_depth_fixed" + std::to_string(gOutputImageIdx) + ".png";
            std::string window_depth_fixed_removal_txt_name = gOutputImagedir + "/window_depth_fixed_removal" + std::to_string(gOutputImageIdx) + ".txt";
            std::string window_depth_fixed_removal_png_name = gOutputImagedir + "/window_depth_fixed_removal" + std::to_string(gOutputImageIdx) + ".png";

            std::string window_conf_name = gOutputImagedir + "/window_conf" + std::to_string(gOutputImageIdx) + ".txt";
            // 1. get windowed color data, export
            cv::Mat window_color = mCurColorImage->ConvertToOpencvPresented(mWindowSt, mWindowSize);
            cKinectImageResourcePtr tmp_res = std::make_shared<cKinectImageResource>();
            tmp_res->ConvertFromOpencv(window_color, false);
            ExportRGBColorToPng(
                tmp_res->mRawData.data(),
                tmp_res->mRawHeight, tmp_res->mRawWidth, tmp_res->mChannels, window_color_png_name);
            printf("export window rgb png to %s\n", window_color_png_name.c_str());
            // 2. get windowed depth data, export
            cv::Mat window_depth = mCurDepthImage->ConvertToOpencvPresented(mWindowSt, mWindowSize);
            tmp_res->ConvertFromOpencv(window_depth, false);
            ExportDepthToTxt(
                tmp_res->mRawData.data(),
                tmp_res->mRawHeight, tmp_res->mRawWidth, tmp_res->mChannels, window_depth_txt_name);
            ExportDepthToPng(
                tmp_res->mRawData.data(),
                tmp_res->mRawHeight, tmp_res->mRawWidth, tmp_res->mChannels, window_depth_png_name);
            printf("export window depth txt/png to %s\n", window_depth_txt_name.c_str());
            // cv::imshow("color", window_color);
            // cv::imshow("depth", window_depth);
            // cv::waitKey(0);
            // exit(1);
            // 2. get the depth image
            // cv::Mat window_depth = mCurDepthImage->ConvertToOpencvPresented(mWindowSt, mWindowSize);
            // Y = 0.299 R + 0.587 G + 0.114 B
            cv::cvtColor(window_depth, window_depth, CV_RGB2GRAY);
            // 3. do the fix
            // cTimeUtil::Begin("fix");
            cv::Mat fixed_depth = cColorDepthFixer::FixColorDepth(window_color, window_depth);
            cv::cvtColor(fixed_depth, fixed_depth, CV_GRAY2RGB);
            fixed_depth.convertTo(fixed_depth, CV_8UC3);

            mDepthFixed->ConvertFromOpencv(fixed_depth, false);
            ExportDepthToTxt(
                mDepthFixed->mRawData.data(),
                mDepthFixed->mRawHeight, mDepthFixed->mRawWidth, mDepthFixed->mChannels, window_depth_fixed_txt_name);
            ExportDepthToPng(
                mDepthFixed->mRawData.data(),
                mDepthFixed->mRawHeight, mDepthFixed->mRawWidth, mDepthFixed->mChannels, window_depth_fixed_png_name);
            printf("export fixed window depth txt/png to %s\n", window_depth_fixed_txt_name.c_str());

            // export removal depth
            ExportDepthToPng(
                mDepthFixedRemoval->mRawData.data(),
                mDepthFixedRemoval->mRawHeight, mDepthFixedRemoval->mRawWidth, mDepthFixedRemoval->mChannels, window_depth_fixed_removal_png_name);
            ExportDepthToPng(
                mDepthFixedRemoval->mRawData.data(),
                mDepthFixedRemoval->mRawHeight, mDepthFixedRemoval->mRawWidth, mDepthFixedRemoval->mChannels, window_depth_fixed_removal_txt_name);
            printf("export fixed removal window depth txt/png to %s\n", window_depth_fixed_removal_txt_name.c_str());
            ExportWindowConfiguration(window_conf_name);
            printf("export window info to %s\n", window_conf_name.c_str());
        }
        ExportKinectConfiguration(conf_name);
        SIM_INFO("depth png {}, depth txt {}, color png {}, kinect conf {}",
                 depth_png_name,
                 depth_txt_name,
                 color_png_name,
                 conf_name);
        gOutputImageIdx += 1;
    }
}

void cKinectManagerImGui::ShowCamResolution()
{
    cKinectImageResourcePtr res = nullptr;
    if (mCurColorImage->mRawHeight != 0)
        res = mCurColorImage;
    else
        res = mCurDepthImage;
    ImGui::Text("Cam Reso %d %d", res->mRawHeight, res->mRawWidth);
}
void cKinectManagerImGui::ExportKinectConfiguration(std::string output_name)
{
    std::ofstream fout(output_name, 'w');
    fout << "display mode = " << gDisplayModeStr[mDisplayMode] << std::endl;
    fout << "color mode = " << gColorModeStr[mColorMode] << std::endl;
    fout << "depth mode = " << gDepthModeStr[mDepthMode] << std::endl;
    fout << "estimate cam pos = " << mEstimatedCamPos.transpose() << std::endl;
    fout << "estimate cam focus = " << mEstimatedCamFocus.transpose() << std::endl;
    fout << "estimate cam up = " << mEstimatedCamUp.transpose() << std::endl;
    fout << "vfov = " << this->mRaycastFov << std::endl;
    fout << "recommend vfov = " << this->GetCurDepthVFOV() << std::endl;
    fout << "captured at " << cTimeUtil::GetSystemTime() << std::endl;
    fout.close();
}
void cKinectManagerImGui::ExportWindowConfiguration(std::string outputname)
{
    std::ofstream fout(outputname, 'w');
    cKinectImageResourcePtr output = nullptr;
    if (mCurDepthImage->mRawHeight != 0)
    {
        output = mCurDepthImage;
    }
    else if (mCurColorImage->mRawHeight != 0)
    {
        output = mCurColorImage;
    }
    else
    {
        std::cout << "ExportWindowConfiguration failed, cuz there are no info available\n";
        return;
    }
    fout << "image size = " << output->mPresentHeight << " " << output->mPresentWidth << " enabled downsample = " << mEnableDownsample << std::endl;
    fout << "window start = " << this->mWindowSt.transpose() << std::endl;
    fout << "window size = " << mWindowSize.transpose() << std::endl;
    fout.close();
}
cKinectImageResourcePtr cKinectManagerImGui::GetDepthImageNew()
{
    if (mDepthMode == K4A_DEPTH_MODE_OFF)
    {
        return nullptr;
    }
    else
    {
        auto capture = GetCapture();
        if (mDepthMode == K4A_DEPTH_MODE_PASSIVE_IR)
        {
            k4a_image_t image = k4a_capture_get_ir_image(capture);
            mCurDepthImage->ConvertFromKinect(image, mEnableDownsample, 0);
            k4a_image_release(image);
        }
        else
        {
            k4a_image_t image = k4a_capture_get_depth_image(capture);
            mCurDepthImage->ConvertFromKinect(image, mEnableDownsample, mDepthAdjustBias);
            k4a_image_release(image);
        }
        k4a_capture_release(capture);
    }

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
    if (mEnableWindowed)
    {
        for (auto &x : res)
        {
            x->DimmedByWindow(
                this->mWindowSt,
                mWindowSize);
        }

        if (res.size() == 2 && mDisplayMode == eDisplayMode::DEPTH_TO_COLOR && mEnableDepthFix)
        {
            // mCurDepthImage->ConvertToOpencvPresented
            // get the color and depth image, and fix
            std::cout << "ready to fix\n";
            // 1. get the color image

            cv::Mat window_color = mCurColorImage->ConvertToOpencvPresented(mWindowSt, mWindowSize);
            // std::cout << "get window color succ, size = " << window_color.rows << " " << window_color.cols << std::endl;
            // std::cout << "get window depth succ, size = " << window_depth.rows << " " << window_depth.cols << std::endl;

            // cv::imshow("color", window_color);
            // cv::imshow("depth", window_depth);
            // cv::waitKey(0);
            // exit(1);
            // 2. get the depth image
            cv::Mat window_depth = mCurDepthImage->ConvertToOpencvPresented(mWindowSt, mWindowSize);
            // Y = 0.299 R + 0.587 G + 0.114 B
            cv::cvtColor(window_depth, window_depth, CV_RGB2GRAY);
            // 3. do the fix
            // cTimeUtil::Begin("fix");
            cv::Mat fixed_depth = cColorDepthFixer::FixColorDepth(window_color, window_depth);
            cv::cvtColor(fixed_depth, fixed_depth, CV_GRAY2RGB);
            fixed_depth.convertTo(fixed_depth, CV_8UC3);

            mDepthFixed->ConvertFromOpencv(fixed_depth, false);
            // cTimeUtil::End("fix");
            // cv::imshow("color", window_color);
            // cv::imshow("depth", window_depth);
            // cv::imshow("fixed depth", fixed_depth);
            // cv::waitKey(0);

            // 4. restore back the depth buffer
        }

        if (mDepthFixed->mPresentHeight != 0 && mEnableDepthRemoval == true)
        {
            // if depth removal is enabled, and depth fixed is true
            // mDepthRemoval = mDepthFixed
            // then apply the removal mask
            mDepthFixedRemoval->ConvertFromAnotherResource(mDepthFixed);
            mDepthFixedRemoval->ApplyValueMask(this->mDepthRemovalValue_mm / 1000.0);
        }
    }
    if (mDepthFixed->mPresentHeight != 0)
    {
        res.push_back(mDepthFixed);
        if (mEnableDepthRemoval)
        {
            res.push_back(mDepthFixedRemoval);
        }
    }
    if (mDebugDrawChessboard == true && mDebugChessboardResource->mPresentHeight != 0)
    {
        res.push_back(mDebugChessboardResource);
    }

    if (mEnableCompareDistort == true)
    {
        // if color is availiable, undistort and compare
        if (mCurColorImage->mRawHeight != 0)
        {
            cv::Mat raw_mat = cOpencvUtil::ConvertFloatArrayToRGBMat(
                mCurColorImage->mRawHeight,
                mCurColorImage->mRawWidth,
                mCurColorImage->mRawData.data());
            // cv::imshow("raw color", raw_mat);
            // cv::waitKey(0);

            auto opencv_cammat = cOpencvUtil::ConvertEigenMatToOpencv(mColorIntri.mCamMtx);
            // std::cout << "opencv cammat = " << opencv_cammat << std::endl;
            auto opencv_camdist = cOpencvUtil::ConvertEigenVecToOpencv(mColorIntri.mDistCoef);
            // std::cout << "opencv camdist = " << opencv_camdist << std::endl;
            cv::Mat undistort_mat = cOpencvUtil::Undistort(raw_mat, opencv_cammat, opencv_camdist);
            // cv::imshow("undistort color", undistort_mat);
            // cv::waitKey(0);

            // std::cout << cOpencvUtil::type2str(undistort_mat.type()) << std::endl;
            cv::Mat diff = raw_mat - undistort_mat;
            mUndistortColorRes->ConvertFromOpencv(undistort_mat, mEnableDownsample);
            mUndistortColorDiff->ConvertFromOpencv(diff, mEnableDownsample);

            res.push_back(mUndistortColorRes);
            res.push_back(mUndistortColorDiff);
        }

        // if depth is availiable, undistort and compare
        if (mCurDepthImage->mRawHeight != 0)
        {
            cv::Mat raw_mat = cOpencvUtil::ConvertFloatArrayToRGBMat(
                mCurDepthImage->mRawHeight,
                mCurDepthImage->mRawWidth,
                mCurDepthImage->mRawData.data());
            // cv::imshow("raw color", raw_mat);
            // cv::waitKey(0);

            auto opencv_cammat = cOpencvUtil::ConvertEigenMatToOpencv(mDepthIntri.mCamMtx);
            // std::cout << "opencv cammat = " << opencv_cammat << std::endl;
            auto opencv_camdist = cOpencvUtil::ConvertEigenVecToOpencv(mDepthIntri.mDistCoef);
            // std::cout << "opencv camdist = " << opencv_camdist << std::endl;
            cv::Mat undistort_mat = cOpencvUtil::Undistort(raw_mat, opencv_cammat, opencv_camdist);
            // cv::imshow("undistort color", undistort_mat);
            // cv::waitKey(0);

            // std::cout << cOpencvUtil::type2str(undistort_mat.type()) << std::endl;
            cv::Mat diff = raw_mat - undistort_mat;
            mUndistortDepthRes->ConvertFromOpencv(undistort_mat, mEnableDownsample);
            mUndistortDepthDiff->ConvertFromOpencv(diff, mEnableDownsample);

            res.push_back(mUndistortDepthRes);
            res.push_back(mUndistortDepthDiff);
        }
    }
    return res;
}
#include "utils/TimeUtil.hpp"
std::vector<cKinectImageResourcePtr> cKinectManagerImGui::GetDepthToColorImageNew()
{
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

void cKinectManagerImGui::PoseEstimateFromColor()
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
        // std::cout << "no color resource, calibration ret\n";
        mDebugChessboardResource->Reset();
    }
}
cv::Mat LoadMatFromTextFile(std::string ir_path)
{
    //  = "output/depth0.txt";
    std::string cont = cFileUtil::ReadTextFile(ir_path);
    std::vector<std::string> lines = SplitString(cont, "\n");
    std::string first_line = lines[0];
    lines.erase(lines.begin());
    std::
        vector<std::string>
            first_line_splited = SplitString(first_line, " ");
    int width = std::stoi(first_line_splited[1]);
    int height = std::stoi(first_line_splited[3]);

    cv::Mat ir_image(cv::Size(width, height), CV_8UC3);
    for (int row = 0; row < height; row++)
    {
        std::vector<std::string> row_cont = SplitString(lines[row], " ");
        for (int col = 0; col < width; col++)
        {
            float res = std::stof(row_cont[col]);
            ;
            uint8_t val = int(res * 255) > 255 ? 255 : uint8_t(res * 255);
            // std::cout << res << "->" << int(val) << std::endl;
            ir_image.at<cv::Vec3b>(row, col) = cv::Vec3b(val, val, val);
        }
    }
    return ir_image;
}
void cKinectManagerImGui::PoseEstimateFromIR()
{
    std::cout << "begin pose ir est\n";
    if (this->mDepthMode == K4A_DEPTH_MODE_PASSIVE_IR && mCurDepthImage->mRawData.size() != 0)
    {
        // valid ir image
        // 1. convert it to cv::mat
        // begin to calculate cam pose from IR image
        {
            int height = mCurDepthImage->mRawHeight,
                width = mCurDepthImage->mRawWidth;
            cv::Mat debug_image(cv::Size(width, height), CV_8UC3);
            tVector cam_pos = tVector::Zero(), cam_focus = tVector::Zero(), cam_up = tVector::Zero();
            cv::Mat ir_image = cOpencvUtil::ConvertFloatArrayToRGBMat(height, width, mCurDepthImage->mRawData.data());
            cPoseEstimator::Estimate(
                ir_image,
                mDepthIntri.mCamMtx, mDepthIntri.mDistCoef, debug_image, cam_pos, cam_focus, cam_up);

            std::cout << "cam_pos = " << cam_pos.transpose() << std::endl;
            std::cout << "cam_up = " << cam_up.transpose() << std::endl;
            std::cout << "cam_focus = " << cam_focus.transpose() << std::endl;
            mEstimatedCamPos = cam_pos.segment(0, 3).cast<float>();
            mEstimatedCamFocus = cam_focus.segment(0, 3).cast<float>();
            mEstimatedCamUp = cam_up.segment(0, 3).cast<float>();
            // cv::imshow("ir", ir_image);
            // cv::imshow("debug", debug_image);
            // cv::waitKey(0);
            // exit(1);
        }
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
    if (new_depth_mode == K4A_DEPTH_MODE_PASSIVE_IR && mDisplayMode == eDisplayMode::DEPTH_TO_COLOR)
    {
        SIM_WARN("depth mode is passive IR, the mode depth_to_color is converted");
        mDisplayMode = eDisplayMode::COLOR_AND_DEPTH;
    }
    if (new_depth_mode != mDepthMode || new_color_mode != mColorMode)
    {

        cKinectManager::SetColorAndDepthMode(new_depth_mode, new_color_mode);
        mRaycastFov = GetCurDepthVFOV();
    }
}
