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

class cKinectManagerImGui : public cKinectManager
{
public:
    explicit cKinectManagerImGui(std::string display_mode);
    virtual void Init() override final;
    static std::string BuildStrFromDisplayMode(eDisplayMode mode);
    static eDisplayMode BuildDisplayModeFromStr(std::string str);
    void UpdateGui();

    std::vector<cKinectImageResourcePtr> GetRenderingResource();

protected:
    eDisplayMode mDisplayMode;
    std::string mDisplayModeStr, mDepthModeStr, mColorModeStr;

    k4a_calibration_t mCalibration;
    k4a_transformation_t mCalibratinTrans;
    cKinectImageResourcePtr mCurDepthImage, mCurColorImage;
    bool mEnableDownsample;
    cKinectImageResourcePtr GetDepthImageNew();
    cKinectImageResourcePtr GetColorImageNew();
    std::vector<cKinectImageResourcePtr> GetDepthToColorImageNew();
};

SIM_DECLARE_PTR(cKinectManagerImGui);