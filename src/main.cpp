#include <iostream>
#include "GLUtil.h"
#include "utils/MathUtil.h"
#include "KinectManager.h"
unsigned int gWindowWidth = 0;
unsigned int gWindowHeight = 0;
int gStartX = 100;
int gStartY = 100;
std::string gWindowName = "";

#include "utils/TimeUtil.hpp"
#include "Render.h"

SIM_DECLARE_PTR(cRender);
int main()
{
    cKinectManagerPtr manager = std::make_shared<cKinectManager>("nfov_unbinned");
    tMatrixXi depth_image = manager->GetDepthImage();
    printf("get depth image size %d %d\n", depth_image.rows(), depth_image.cols());
    gWindowHeight = depth_image.rows();
    gWindowWidth = depth_image.cols();

    cRenderPtr render = std::make_shared<cRender>(gWindowHeight, gWindowWidth);
    render->Init();
    GLFWwindow *gWindow = render->GetWindow();

    while (!glfwWindowShouldClose(gWindow))
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        render->UseProgram();
        const tMatrixXi &depth_image = manager->GetDepthImage();
        render->UpdateTextureFromDepthImage(depth_image);
        render->PostUpdate();
    }
    glfwTerminate();
}

// float GetPosValue(int row, int col)
// {
//     int bias = ((gWindowHeight - 1 - row) * gWindowWidth + col) * 3;
//     return global_data_ptr[bias];
// }