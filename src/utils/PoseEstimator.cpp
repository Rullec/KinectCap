#include "PoseEstimator.h"
#include "utils/MathUtil.h"
#include "utils/LogUtil.h"
#include "utils/TimeUtil.hpp"
cv::Size chessboard_size = cv::Size(6, 9);
float chessboard_gap = 50; // cube edge is 50 mm

void cPoseEstimator::Estimate(const cv::Mat &color_mat,
                              const tMatrix3d &mtx,
                              const tVectorXd &dist_coef, cv::Mat &draw_color_mat, tVector &cam_pos, tVector &cam_focus, tVector &cam_up)
{
    // cPoseEstimator::Test();
    // exit(1);
    draw_color_mat = color_mat;
    // std::cout << "cam mtx = \n"
    //           << mtx << std::endl;
    // std::cout << "cam dist coef = " << dist_coef.transpose() << std::endl;
    // 1. show the image
    // cv::imshow("color", color_mat);
    // cv::waitKey(0);
    // 2. convert it to the grayscale
    cv::Mat gray_mat(color_mat.rows, color_mat.cols, CV_8UC1);
    cv::cvtColor(color_mat, gray_mat, CV_BGR2GRAY);
    // cv::imshow("depth", gray_mat);
    // cv::waitKey(0);

    // 3. get obj points
    auto obj_pts = cPoseEstimator::GetObjPoints();

    // 4. get image point (find chessboard)
    auto img_pts = cPoseEstimator::FindChessboardCorner(gray_mat, draw_color_mat);
    // for (int i = 0; i < obj_pts.size(); i++)
    // {
    //     std::cout << "image point = " << img_pts[i] << " obj point = " << obj_pts[i] << std::endl;
    // }
    // cv::imshow("find corners", draw_color_mat);
    // cv::waitKey(0);
    if (obj_pts.size() == img_pts.size())
    {
        cPoseEstimator::CalcCamPos(img_pts, obj_pts, mtx, dist_coef, cam_pos, cam_focus, cam_up);
    }
    else
    {
        // SIM_WARN("Cannot find the calibrated chessboard, fail to estimate cam pose");
        cam_pos.setZero();
        cam_focus.setZero();
    }
    // 4. solvePnp
    // std::cout << "calc cam pos done\n";
    // exit(1);
}

tVector2i cPoseEstimator::GetImageShape(const cv::Mat &mat)
{

    return tVector2i(mat.rows, mat.cols);
}

std::vector<cv::Point3f> cPoseEstimator::GetObjPoints()
{
    /*
    (5, 0)                        (0, 0)  right up




    (5, 8)                        (0, 8) right down
    | Y+
    |
    |--------------------------------X+
    */
    float height_from_board_bottom_to_thelowest_chessboard = 84;
    float height_rotating_table = 60;
    std::vector<cv::Point3f> img_points(0);
    cv::Point3f right_up_origin(50 * (chessboard_size.height - 1) / 2, 50 * chessboard_size.width + height_from_board_bottom_to_thelowest_chessboard + height_rotating_table, 0);
    // SIM_WARN("set height of rotating table {}", height_rotating_table);
    // std::cout << "right up origin = " << right_up_origin << std::endl;
    for (int col_id_from_right = 0; col_id_from_right < chessboard_size.height; col_id_from_right++)
    {
        for (int row_id_from_up = 0; row_id_from_up < chessboard_size.width; row_id_from_up++)
        {
            float x = right_up_origin.x - (col_id_from_right)*chessboard_gap;
            float y = right_up_origin.y - (row_id_from_up)*chessboard_gap;
            cv::Point3f new_p = cv::Point3f(x, y, 0);
            // printf("for point (%d, %d),", col_id_from_right, row_id_from_up);
            // std::cout << new_p << std::endl;
            img_points.push_back(new_p);
        }
    }
    return img_points;
}

std::vector<cv::Point2f> cPoseEstimator::FindChessboardCorner(const cv::Mat &gray_mat, cv::Mat &color_mat)
{
    std::vector<cv::Point2f> corners{};
    /*
    InputArray image, Size patternSize, OutputArray corners,
                                         int flags = CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE 
    */
    auto obj_pts = cPoseEstimator::GetObjPoints();
    bool found = cv::findChessboardCorners(gray_mat, chessboard_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
    if (found)
    {
        cornerSubPix(gray_mat, corners, chessboard_size, cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1));
        drawChessboardCorners(color_mat, chessboard_size, corners, found);

        int min_x = 1e4, max_x = -1, min_y = 1e4, max_y = -1;
        for (auto &data : corners)
        {
            if (data.x < min_x)
                min_x = data.x;
            if (data.y < min_y)
                min_y = data.y;
            if (data.x > max_x)
                max_x = data.x;
            if (data.y > max_y)
                max_y = data.y;
        }
        min_x = int(min_x * 0.9);
        min_y = int(min_y * 0.9);
        max_x = int(max_x * 1.1);
        max_y = int(max_y * 1.1);
        max_x = std::min(max_x, color_mat.cols);
        max_y = std::min(max_y, color_mat.rows);

        cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);

        for (size_t r = 0; r < chessboard_size.height; r++)
        {
            for (size_t c = 0; c < chessboard_size.width; c++)
            {
                std::ostringstream oss;
                auto cur_corner = corners[r * chessboard_size.width + c];
                auto cur_obj_pt = obj_pts[r * chessboard_size.width + c];
                oss << "(" << r << "," << c << ")";
                cv::putText(color_mat, oss.str(), cur_corner, cv::FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 255, 0), 1);
                // std::ostringstream new_oss;
                // new_oss << "(" << cur_obj_pt.x << "," << cur_obj_pt.y << ")";
                // if (c % 2 == (r % 2))
                //     cur_corner.y += 10;
                // else
                //     cur_corner.y -= 10;
                // cur_corner.x -= 30;
                // cv::putText(color_mat, new_oss.str(), cur_corner, cv::FONT_HERSHEY_PLAIN, 0.7, CV_RGB(255, 0, 0), 1);
            }
        }
        color_mat = color_mat(rect);
        auto new_size = cv::Size(color_mat.cols * 2, color_mat.rows * 2);
        cv::resize(color_mat, color_mat, new_size, CV_INTER_AREA);
        // clip the image and resize
    }
    return corners;
}

void cPoseEstimator::Downsample(cv::Mat &mat)
{
    cv::Size new_size(mat.cols / 2, mat.rows / 2);
    cv::resize(mat, mat, new_size, CV_INTER_LINEAR);
};

void cPoseEstimator::CalcCamPos(
    const std::vector<cv::Point2f> &img_pts,
    const std::vector<cv::Point3f> &obj_pts,
    const tMatrix3d &cam_mat,
    const tVectorXd &coef, tVector &cam_pos, tVector &cam_focus, tVector &cam_up)
{
    // auto cameraMatrix_Front = cam_mat;
    cv::Mat cameraMatrix_Front(3, 3, CV_32FC1);
    cv::Mat dist_coef(1, coef.size(), CV_32FC1);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            cameraMatrix_Front.at<float>(i, j) = cam_mat(i, j);
        }
    for (int i = 0; i < coef.size(); i++)
    {
        dist_coef.at<float>(0, i) = coef[i];
    }
    // std::cout << "input cam mat = " << cameraMatrix_Front << std::endl;
    // std::cout << "input dist coef = " << dist_coef << std::endl;
    cv::Mat world_pts_to_cam_coords_rvec, world_pts_to_cam_coords_tvec;
    solvePnP(obj_pts, img_pts, cameraMatrix_Front, dist_coef, world_pts_to_cam_coords_rvec, world_pts_to_cam_coords_tvec);

    // std::cout << "rotation vector = " << world_pts_to_cam_coords_rvec << std::endl;
    // std::cout << "translation vec = " << world_pts_to_cam_coords_tvec << std::endl;
    // std::cout << world_pts_to_cam_coords_rvec.rows << world_pts_to_cam_coords_rvec.cols << std::endl;
    // std::cout << world_pts_to_cam_coords_rvec.type() << std::endl;

    tVector axis_angle = tVector(world_pts_to_cam_coords_rvec.at<double>(0, 0), world_pts_to_cam_coords_rvec.at<double>(1, 0), world_pts_to_cam_coords_rvec.at<double>(2, 0), 0);
    tVector pos = tVector(world_pts_to_cam_coords_tvec.at<double>(0, 0), world_pts_to_cam_coords_tvec.at<double>(1, 0), world_pts_to_cam_coords_tvec.at<double>(2, 0), 1);
    // std::cout << "axis angle = " << axis_angle.transpose() << std::endl;
    // std::cout << "pos = " << pos.transpose() << std::endl;

    tMatrix cam_pts_to_world_coords = cMathUtil::AxisAngleToRotmat(axis_angle);
    cam_pts_to_world_coords.block(0, 3, 4, 1) = pos;
    // std::cout << "cam pts to world coords cam_pts_to_world_coords = \n"
    //           << cam_pts_to_world_coords << std::endl;
    cam_pts_to_world_coords = cam_pts_to_world_coords.inverse().eval();

    cam_pos = cam_pts_to_world_coords * tVector(0, 0, 0, 1);

    // the obj coordinates: Y axis from up to down (screen)
    cam_up = cam_pts_to_world_coords * tVector(0, -1, 0, 0);
    tVector cam_focus_dir = cam_pts_to_world_coords * tVector(0, 0, 1, 0);
    double t = -cam_pos[2] / cam_focus_dir[2];
    cam_focus = cam_pos + cam_focus_dir * t;
    // std::cout << "cam pos = " << cam_pos.transpose() << std::endl;
    // std::cout << "cam focus dir = " << cam_focus_dir.transpose() << std::endl;
    // std::cout << "cam focus = " << cam_focus.transpose() << std::endl;
}