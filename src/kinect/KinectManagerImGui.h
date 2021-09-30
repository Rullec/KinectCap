#include "KinectManager.h"
#include "utils/DefUtil.h"
enum eDisplayMode
{
    ONLY_COLOR = 0,
    ONLY_DEPTH,
    COLOR_AND_DEPTH,
    DEPTH_TO_COLOR,
    NUM_OF_DISPLAY_MODE
};

SIM_DECLARE_CLASS_AND_PTR(cKinectImageResource);
SIM_DECLARE_CLASS_AND_PTR(cDepthImageCaster);

class cKinectManagerImGui : public cKinectManager
{
public:
    explicit cKinectManagerImGui(std::string display_mode);
    virtual void Init() override final;
    static std::string BuildStrFromDisplayMode(eDisplayMode mode);
    static eDisplayMode BuildDisplayModeFromStr(std::string str);
    void UpdateGui();

    std::vector<cKinectImageResourcePtr> GetRenderingResource();
    virtual void SetColorAndDepthMode(k4a_depth_mode_t new_depth_mode, k4a_color_resolution_t new_color_mode) override;

protected:
    eDisplayMode mDisplayMode;
    std::string mDisplayModeStr, mDepthModeStr, mColorModeStr;

    k4a_calibration_t mCalibration;
    k4a_transformation_t mCalibratinTrans;
    cKinectImageResourcePtr mCurDepthImage, mCurColorImage;
    bool mEnableCompareDistort;
    bool mEnablePoseEstimate;
    int mPoseEstimateMode;
    bool mEnableShowRaycast;
    bool mEnableDownsample;
    bool mDebugDrawChessboard;
    bool mEnableDepthDiff;
    bool mLockEstimation;
    bool mEnableDepthFix;
    bool mEnableWindowed;
    tVector2i mWindowSt, mWindowSize;
    cDepthImageCasterPtr mRaycaster;
    cKinectImageResourcePtr mRaycastDepthResource;
    cKinectImageResourcePtr mDebugChessboardResource;
    cKinectImageResourcePtr mDepthDiffResource;
    cKinectImageResourcePtr mUndistortDepthRes, mUndistortColorRes;
    cKinectImageResourcePtr mUndistortDepthDiff, mUndistortColorDiff;
    cKinectImageResourcePtr mDepthFixed, mColorFixed;
    float mRaycastFov;
    float mDepthAdjustBias;
    tVector3f mEstimatedCamPos, mEstimatedCamFocus, mEstimatedCamUp;

    cKinectImageResourcePtr GetDepthImageNew();
    cKinectImageResourcePtr GetColorImageNew();
    std::vector<cKinectImageResourcePtr> GetDepthToColorImageNew();
    void ExportCapture();
    void ShowCamResolution();
    void PoseEstimateFromColor();
    void PoseEstimateFromIR();
    void ExportKinectConfiguration(std::string outputname);
    void InitRaycastDepthImage();
    void UpdateRaycastResult(cKinectImageResourcePtr depth_result);
    void UpdateDepthDiffResult(cKinectImageResourcePtr diff_depth);
    double GetCurDepthVFOV();
};

SIM_DECLARE_PTR(cKinectManagerImGui);