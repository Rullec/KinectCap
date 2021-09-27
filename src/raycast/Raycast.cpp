#include <iostream>
#include "utils/JsonUtil.h"
#include "raycast/Raycast.h"
#define USE_OPTIX
#include "geometries/OptixRaycaster.h"
#include "sim/KinematicBody.h"
#include "cameras/ArcBallCamera.h"
using namespace std;

cDepthImageCaster::cDepthImageCaster()
{
    // std::cout << "hello moto\n"
    //           << std::endl;
    Init();
    // std::cout << "init done\n";
}

void cDepthImageCaster::Init()
{
    Json::Value conf;
    conf[cOptixRaycaster::ENABLE_ONLY_EXPORTING_CUTTED_WINDOW_KEY] = false;
    cOptixRaycaster::Init(conf);
    InitObstacles();

    // int height = 500, width = 500;

    // tVector3f cam_pos = tVector3f(0, 0.5, 0.5);
    // tVector3f cam_center = tVector3f(0, 0, 0);

    // float fov = 60;

    // // std::string output = "go.png";
    // // CalcDepthMap(range, height, width, cam, output);
    // std::vector<float> data;
    // CalcResult(cam_pos, cam_center, fov, height, width, data);
}

void cDepthImageCaster::InitObstacles()
{
    // 1. parse the number of obstacles
    Json::Value conf;
    cJsonUtil::LoadJson("obstacle.json", conf);

    int num_of_obstacles = conf.size();
    SIM_ASSERT(num_of_obstacles == conf.size());
    for (int i = 0; i < num_of_obstacles; i++)
    {
        cKinematicBodyPtr ptr = std::make_shared<cKinematicBody>(i);
        ptr->Init(conf[i]);
        // std::cout << ptr->GetNumOfTriangles() << std::endl;
        AddResources(ptr);
    }
}

void cDepthImageCaster::CalcResult(
    const tVector3f &cam_pos,
    const tVector3f &cam_center,
    const tVector3f &cam_up,
    float fov,
    int height,
    int width,
    std::vector<float> &data)
{
    float near_plane = 1e-2;
    float far_plane = 1e3;
    auto cam = std::make_shared<cArcBallCamera>(cam_pos, cam_center,
                                                cam_up, fov, near_plane, far_plane);
    tMatrix2i range = tMatrix2i::Zero();
    range(0, 1) = width;
    range(1, 1) = height;
    tVectorXf data_eigen(width * height);
    CalcDepthMap(range, height, width, cam, data_eigen);
    data.resize(3 * height * width);
    for (int row = 0; row < height; row++)
        for (int col = 0; col < width; col++)
        {
            int raw_idx = (height - 1 - row) * width + col;
            int output_idx = row * width + col;
            float val = data_eigen[raw_idx];
            data[3 * output_idx + 0] = val;
            data[3 * output_idx + 1] = val;
            data[3 * output_idx + 2] = val;
        }
}
