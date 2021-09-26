#pragma once
#include "utils/MathUtil.h"
#include "utils/DefUtil.h"
#include <memory>
#include <k4a/k4a.h>

class cKinectManager
{
public:
    cKinectManager(std::string color_mode_str, std::string depth_mode_str);
    virtual ~cKinectManager();
    virtual tMatrixXi GetDepthImage();
    virtual double GetDepthUnit_mm();
    virtual tMatrixXi GetIrImage();

    virtual tMatrix3d GetDepthIntrinsicMtx() const;
    virtual tVectorXd GetDepthIntrinsicDistCoef() const;
    virtual tMatrix3d GetColorIntrinsicMtx() const;
    virtual tVectorXd GetColorIntrinsicDistCoef() const;

    virtual std::vector<tMatrixXi> GetColorImage() const;
    virtual tMatrixXi GetDepthToColorImage() const;

    // mode configuration
    virtual std::string GetDepthModeStr() const;
    virtual void SetDepthModeFromStr(std::string mode);
    virtual void SetDepthMode(k4a_depth_mode_t mode);

protected:
    virtual void Init();
    virtual void CloseDevice();
    k4a_device_t mDevice;
    k4a_device_configuration_t mConfig;
    k4a_depth_mode_t mDepthMode;
    k4a_color_resolution_t mColorMode;
    virtual k4a_capture_t GetCapture() const;
    virtual k4a_calibration_camera_t GetDepthCalibration() const;
    virtual k4a_calibration_camera_t GetColorCalibration() const;

    // openni::VideoFrameRef m_depthFrame;
    // openni::Device m_device;
    // openni::VideoStream m_depthStream;
    // int m_width, m_height;
    // tMatrixXi depth_mat, ir_mat;
    // openni::VideoFrameRef m_irFrame;
    // openni::VideoStream m_irStream;
};
SIM_DECLARE_PTR(cKinectManager);