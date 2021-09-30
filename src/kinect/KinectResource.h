#pragma
#include <vector>
#include <k4a/k4a.h>
#include <string>
#include "utils/DefUtil.h"
#include "utils/MathUtil.h"
namespace cv
{
    class Mat;
};
class cKinectImageResource
{
public:
    cKinectImageResource();
    void ConvertFromKinect(k4a_image_t image, bool enable_downsampling = true, float value_adjust = 0);
    void ConvertFromOpencv(const cv::Mat &image, bool enable_downsampling = true);
    virtual void DimmedByWindow(const tVector2i &st_pos, const tVector2i &window_size);
    virtual cv::Mat ConvertToOpencvPresented(const tVector2i & window_st,
    const tVector2i & window_size);
    void Reset();
    int mPresentHeight, mPresentWidth;
    int mRawHeight, mRawWidth;
    int mChannels;
    bool mEnableDownsampling;
    float mValueAdjust;
    std::vector<float> mPresentData;
    std::vector<float> mRawData;

protected:
    void ConvertFromDepthImage(k4a_image_t);
    void ConvertFromDepthImageRaw(k4a_image_t);
    void ConvertFromColorImage(k4a_image_t);
    void ConvertFromColorImageRaw(k4a_image_t);
};
SIM_DECLARE_PTR(cKinectImageResource);

void ExportDepthToPng(float *buf, int height, int width, int buf_channels, std::string output_name);
void ExportDepthToTxt(float *raw_buf, int height, int width, int buf_channels, std::string output_name);
void ExportRGBColorToPng(float *buf, int height, int width, int buf_channels, std::string output_name);