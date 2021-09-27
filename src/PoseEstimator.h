#pragma once
#include "utils/OpenCVUtil.h"
#include "utils/MathUtil.h"

class cPoseEstimator
{
public:
    // given an image, return the camera pos now
    static void Estimate(
        const cv::Mat &mat,
        const tMatrix3d &cam_mat,
        const tVectorXd &coef, cv::Mat &new_debug_color_mat, tVector &cam_pos, tVector &cam_focus);

protected:
    static void Test();
    static tVector2i GetImageShape(const cv::Mat &mat);
    static std::vector<cv::Point3f> GetObjPoints();
    static std::vector<cv::Point2f>
    FindChessboardCorner(const cv::Mat &gray_mat, cv::Mat &color_mat);
    static void Downsample(cv::Mat &mat);
    static void CalcCamPos(
        const std::vector<cv::Point2f> &img_pts,
        const std::vector<cv::Point3f> &obj_pts,
        const tMatrix3d &cam_mat,
        const tVectorXd &coef, tVector &cam_pos, tVector &cam_focus);
};