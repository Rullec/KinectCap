#include "KinectMode.h"
#include "utils/LogUtil.h"
#include <vector>
const std::vector<const char *> gDepthModeStr = {
    "off",
    "nfov_2x2binned",
    "nfov_unbinned",
    "wfov_2x2binned",
    "wfov_unbinned",
    "passive_ir"};

const std::vector<const char *> gColorModeStr = {
    "off",
    "720p_16_9",
    "1080p_16_9",
    "1440p_16_9",
    "1536p_4_3",
    "2160_16_9",
    "3072_4_3"};

std::string cKinectMode::BuildStringFromDepthMode(k4a_depth_mode_t mode)
{
    return gDepthModeStr[mode];
}
k4a_depth_mode_t cKinectMode::BuildDepthModeFromString(std::string mode_str)
{
    for (int i = 0; i < gDepthModeStr.size(); i++)
    {
        if (mode_str == gDepthModeStr[i])
        {
            return static_cast<k4a_depth_mode_t>(i);
        }
    }
    SIM_ERROR("unrecognized depth mode {}", mode_str);
    exit(1);
    return k4a_depth_mode_t::K4A_DEPTH_MODE_OFF;
}
std::string cKinectMode::BuildStringFromColorMode(k4a_color_resolution_t mode)
{
    return gColorModeStr[mode];
}
k4a_color_resolution_t cKinectMode::BuildColorModeFromString(std::string mode_str)
{
    for (int i = 0; i < gColorModeStr.size(); i++)
    {
        if (mode_str == gColorModeStr[i])
        {
            return static_cast<k4a_color_resolution_t>(i);
        }
    }
    SIM_ERROR("unrecognized depth mode {}", mode_str);
    exit(1);
    return k4a_color_resolution_t::K4A_COLOR_RESOLUTION_OFF;
}