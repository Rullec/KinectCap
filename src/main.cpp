#include <iostream>
#include "utils/GLUtil.h"
#include "utils/MathUtil.h"
// #include "KinectManager.h"
#include "kinect/KinectManagerImGui.h"
unsigned int gWindowWidth = 0;
unsigned int gWindowHeight = 0;
int gStartX = 100;
int gStartY = 100;
std::string gWindowName = "";

#include "utils/TimeUtil.hpp"
#include "render/Render.h"
#include "raycast/Raycast.h"
SIM_DECLARE_PTR(cRender);
cRenderPtr render = nullptr;
#include "render/ColorDepthFixer.h"
#include "utils/OpenCVUtil.h"

void Downsample(cv::Mat &mat, uint8_t type)
{
    cv::Size new_size(mat.cols / 2, mat.rows / 2);
    cv::Mat buf_mat(new_size, type);

    cv::pyrDown(mat, buf_mat, new_size);
    mat = buf_mat;
}
void Test()
{
    std::string color_path = "E:\\KinectCap\\output\\color0.png";
    std::string depth_path = "E:\\KinectCap\\output\\depth0.png";
    cv::Mat color = cv::imread(color_path, cv::IMREAD_COLOR);
    cv::Mat depth = cv::imread(depth_path, cv::IMREAD_GRAYSCALE);

    Downsample(color, CV_8UC3);
    Downsample(color, CV_8UC3);
    
    Downsample(depth, CV_8UC3);
    Downsample(depth, CV_8UC3);
    
    cColorDepthFixer::FixColorDepth(color, depth);
    exit(1);
    cv::imshow("color", color);
    cv::waitKey(0);
    cv::imshow("depth", depth);
    cv::waitKey(0);
}

int main()
{
    // Test();
    // exit(1);
    // cKinectManagerImGuiPtr manager = std::make_shared<cKinectManagerImGui>("depth_to_color");
    cKinectManagerImGuiPtr manager = std::make_shared<cKinectManagerImGui>("only_depth");
    manager->Init();
    tMatrixXi depth_image = manager->GetDepthImage();
    printf("get depth image size %d %d\n", depth_image.rows(), depth_image.cols());
    gWindowHeight = depth_image.rows();
    gWindowWidth = depth_image.cols();

    render = std::make_shared<cRender>(gWindowHeight, gWindowWidth);
    render->SetKinectManager(manager);
    render->Init();
    GLFWwindow *gWindow = render->GetWindow();

    while (!glfwWindowShouldClose(gWindow))
    {
        glfwPollEvents();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        render->UseProgram();
        // render->UpdateTextureFromRenderResource(manager->GetDepthImageNew());
        // render->UpdateTextureFromRenderResource(manager->GetColorImageNew());
        render->UpdateTextureFromRenderResourceVec(manager->GetRenderingResource());
        render->PostUpdate();
    }
    glfwTerminate();
}

// float GetPosValue(int row, int col)
// {
//     int bias = ((gWindowHeight - 1 - row) * gWindowWidth + col) * 3;
//     return global_data_ptr[bias];
// }