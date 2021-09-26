#include "KinectManager.h"
#include "utils/DefUtil.h"
enum eDisplayMode
{
    ONLY_COLOR = 0,
    ONLY_DEPTH,
    COLOR_AND_DEPTH,
    NUM_OF_DISPLAY_MODE
};

class cKinectImageResource
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    cKinectImageResource();
    void ConvertFromKinect(k4a_image_t image);
    int mHeight;
    int mWidth;
    int mChannels;
    std::vector<float> mData;

protected:
    void ConvertFromDepthImage(k4a_image_t);
    void ConvertFromColorImage(k4a_image_t);
};
SIM_DECLARE_PTR(cKinectImageResource);

class cKinectManagerImGui : public cKinectManager
{
public:
    explicit cKinectManagerImGui(std::string display_mode);
    static std::string BuildStrFromDisplayMode(eDisplayMode mode);
    static eDisplayMode BuildDisplayModeFromStr(std::string str);
    void UpdateGui();

    cKinectImageResourcePtr GetDepthImageNew();
    cKinectImageResourcePtr GetColorImageNew();
    std::vector<cKinectImageResourcePtr> GetRenderingResource();

protected:
    eDisplayMode mDisplayMode;
    std::string mDisplayModeStr, mDepthModeStr, mColorModeStr;
    virtual void Init() override final;

    cKinectImageResourcePtr mCurDepthImage, mCurColorImage;
};

SIM_DECLARE_PTR(cKinectManagerImGui);