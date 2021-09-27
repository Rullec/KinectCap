#pragma once
#include "utils/MathUtil.h"
#include "utils/DefUtil.h"
#include <memory>
#include <k4a/k4a.h>

class cKinectManager
{
public:
    cKinectManager(std::string color_mode_str, std::string depth_mode_str);
    virtual void Init();
    virtual ~cKinectManager();
    virtual tMatrixXi GetDepthImage();
    virtual double GetDepthUnit_mm();
    virtual tMatrixXi GetIrImage();

    // virtual tMatrix3d GetDepthIntrinsicMtx() const;
    // virtual tVectorXd GetDepthIntrinsicDistCoef() const;
    // virtual tMatrix3d GetColorIntrinsicMtx() const;
    // virtual tVectorXd GetColorIntrinsicDistCoef() const;

    virtual std::vector<tMatrixXi> GetColorImage() const;
    virtual tMatrixXi GetDepthToColorImage() const;

    // mode configuration
    virtual std::string GetDepthModeStr() const;
    virtual void SetDepthModeFromStr(std::string mode);
    virtual void SetDepthMode(k4a_depth_mode_t mode);
    virtual void SetColorMode(k4a_color_resolution_t mode);
    virtual void SetColorAndDepthMode(k4a_depth_mode_t new_depth_mode, k4a_color_resolution_t new_color_mode);

protected:
    virtual void CloseDevice();
    k4a_device_t mDevice;
    k4a_device_configuration_t mConfig;
    k4a_depth_mode_t mDepthMode;
    k4a_color_resolution_t mColorMode;
    k4a_fps_t mFPS;
    virtual k4a_capture_t GetCapture() const;
    virtual k4a_calibration_camera_t GetDepthCalibration() const;
    virtual k4a_calibration_camera_t GetColorCalibration() const;
    struct tIntrinsics
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        tIntrinsics()
        {
            mCamMtx.setZero();
            mDistCoef.resize(0);
        }
        tMatrix3d mCamMtx;
        tVectorXd mDistCoef;
    };
    tIntrinsics mDepthIntri, mColorIntri;
    // openni::VideoFrameRef m_depthFrame;
    // openni::Device m_device;
    // openni::VideoStream m_depthStream;
    // int m_width, m_height;
    // tMatrixXi depth_mat, ir_mat;
    // openni::VideoFrameRef m_irFrame;
    // openni::VideoStream m_irStream;
    void UpdateDepthIntrins();
    void UpdateColorIntrins();
    void UpdateIntrins(k4a_calibration_camera_t calib, tIntrinsics & intri) const;
    virtual k4a_fps_t GetFPS(k4a_depth_mode_t depth ,k4a_color_resolution_t color);
};
SIM_DECLARE_PTR(cKinectManager);