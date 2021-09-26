#pragma once
#include <k4a/k4a.h>
#include <string>
#include <vector>
class cKinectMode
{
public:
    static std::string BuildStringFromDepthMode(k4a_depth_mode_t mode);
    static k4a_depth_mode_t BuildDepthModeFromString(std::string);
    static std::string BuildStringFromColorMode(k4a_color_resolution_t mode);
    static k4a_color_resolution_t BuildColorModeFromString(std::string);
};
extern const std::vector<const char *> gColorModeStr;
extern const std::vector<const char *> gDepthModeStr;