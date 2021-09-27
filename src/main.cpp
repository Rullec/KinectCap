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

int main()
{
    cKinectManagerImGuiPtr manager = std::make_shared<cKinectManagerImGui>("color_and_depth");
    manager->Init();
    tMatrixXi depth_image = manager->GetDepthImage();
    printf("get depth image size %d %d\n", depth_image.rows(), depth_image.cols());
    gWindowHeight = depth_image.rows();
    gWindowWidth = depth_image.cols();

    cRenderPtr render = std::make_shared<cRender>(gWindowHeight, gWindowWidth);
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