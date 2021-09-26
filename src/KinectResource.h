#pragma
#include <vector>
#include <k4a/k4a.h>
#include "utils/DefUtil.h"
class cKinectImageResource
{
public:
    cKinectImageResource();
    void ConvertFromKinect(k4a_image_t image, bool enable_downsampling = true);
    int mHeight;
    int mWidth;
    int mChannels;
    bool mEnableDownsampling;
    std::vector<float> mData;

protected:
    void ConvertFromDepthImage(k4a_image_t);
    void ConvertFromColorImage(k4a_image_t);
};
SIM_DECLARE_PTR(cKinectImageResource);