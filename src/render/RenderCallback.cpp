#include "RenderCallback.h"
#include "render/Render.h"
extern cRenderPtr render;
void MouseMoveEventCallback(GLFWwindow *window, double xpos, double ypos)
{
    // std::cout << "[log] mouse move to " << xpos << " " << ypos << std::endl;
    if (
        (xpos >= 0) && (xpos < gWindowWidth) && (ypos >= 0) && (ypos < gWindowHeight))
        render->MouseMoveCallback(xpos, ypos);

    // {

    // }
    // {
    //     float depth_m = GetPosValue(ypos, xpos) * 1e3;
    //     std::cout << "cur depth = " << depth_m << " mm\n";
    // }
    // gScene->MouseMoveEvent(xpos, ypos);
}

void MouseButtonEventCallback(GLFWwindow *window, int button, int action, int mods)
{
    // gScene->MouseButtonEvent(button, action, mods);
}

void ErrorCallback(int error, const char *description)
{
    std::cout << "[error] GLFW error callback: " << error << " " << description << std::endl;
    exit(1);
}

void KeyEventCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);

    // gScene->KeyEvent(key, scancode, action, mods);
}

void ResizeCallback(GLFWwindow *window, int w, int h)
{
    // gScene->Update();
    glfwSwapBuffers(window);
    std::cout << "need to call renderer mouse event, to get depth value\n";
}

void ScrollCallback(GLFWwindow *window, double xoff, double yoff)
{
    std::cout << "scroll: x y = " << xoff << " " << yoff << std::endl;
}

const char *vertexShaderSource = "#version 330 core\n"
                                 "layout (location = 0) in vec3 aPos;\n"
                                 "void main()\n"
                                 "{\n"
                                 "   gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
                                 "}\0";
const char *fragmentShaderSource = "#version 330 core\n"
                                   "out vec4 FragColor;\n"
                                   "void main()\n"
                                   "{\n"
                                   "   FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);\n"
                                   "}\n\0";
