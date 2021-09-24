#include <iostream>
#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "utils/MathUtil.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "kinect_manager.h"
cKinectManager *manager = nullptr;

// const unsigned int gWindowWidth = 308;
// const unsigned int gWindowHeight = 215;
unsigned int gWindowWidth = 0;
unsigned int gWindowHeight = 0;
int gStartX = 100;
int gStartY = 100;
std::string gWindowName = "";
GLFWwindow *gWindow = nullptr;

void framebuffer_size_callback(GLFWwindow *window, int width, int height);
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

void MouseButtonEventCallback(GLFWwindow *window, int button, int action, int mods);
void MouseMoveEventCallback(GLFWwindow *window, double xpos, double ypos);
void ErrorCallback(int error, const char *description);
void KeyEventCallback(GLFWwindow *window, int key, int scancode, int action, int mods);
void ResizeCallback(GLFWwindow *window, int w, int h);
void ScrollCallback(GLFWwindow *window, double xoff, double yoff);
unsigned int InitShader();

void InitGLFW()
{
    // init glfw
    if (!glfwInit())
    {
        std::cout << "[error] InitGLFW:: glfw inti failed" << std::endl;
        glfwTerminate();
    }
    glfwSetErrorCallback(ErrorCallback);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE); // fixed size

    gWindow = glfwCreateWindow(gWindowWidth, gWindowHeight, gWindowName.c_str(), NULL, NULL);
    if (gWindow == NULL)
    {
        std::cout << "[error] Failed to create GLFW window" << std::endl;
        glfwTerminate();
    }
    glfwSetWindowPos(gWindow, gStartX, gStartY);
    glfwMakeContextCurrent(gWindow);

    glfwSetKeyCallback(gWindow, KeyEventCallback);
    glfwSetCursorPosCallback(gWindow, MouseMoveEventCallback);
    glfwSetMouseButtonCallback(gWindow, MouseButtonEventCallback);
    glfwSetInputMode(gWindow, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    glfwSetFramebufferSizeCallback(gWindow, ResizeCallback);
    glfwSetScrollCallback(gWindow, ScrollCallback);
}

void InitGL()
{
    if (GLEW_OK != glewInit())
    {
        std::cout << "[errpr] glew init failed " << std::endl;
        exit(1);
    }

    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);
    // glClearColor(0.2, 0.3, 0.4, 1);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

unsigned int BindVAO();

void ConvertImageToTexutre(GLuint &texture)
{
    texture = 0;
    cv::Mat image = cv::imread("now.png");
    if (image.empty())
    {
        std::cout << "image empty" << std::endl;
    }
    else
    {
        cv::flip(image, image, 0);
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        // Set texture clamping method
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

        glTexImage2D(GL_TEXTURE_2D,    // Type of texture
                     0,                // Pyramid level (for mip-mapping) - 0 is the top level
                     GL_RGBA,          // Internal colour format to convert to
                     image.cols,       // Image width  i.e. 640 for Kinect in standard mode
                     image.rows,       // Image height i.e. 480 for Kinect in standard mode
                     0,                // Border width in pixels (can either be 1 or 0)
                     GL_BGR,           // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                     GL_UNSIGNED_BYTE, // Image data type
                     image.ptr());     // The actual image data itself

        glGenerateMipmap(GL_TEXTURE_2D);
    }
}

float *ConvertNoiseToTexture(GLuint &texture)
{
    texture = 0;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);

    float *data = new float[gWindowHeight * gWindowWidth * 3];
    memset(data, 0, sizeof(data));

    for (int row = 0; row < gWindowHeight; row++)
    {
        for (int col = 0; col < gWindowWidth; col++)
        {
            int bias = (row * gWindowWidth + col) * 3;
            data[bias + 0] = float(row) / float(gWindowHeight);
            data[bias + 1] = 0;
            data[bias + 2] = 0;
        }
    }
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Set texture clamping method
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    glTexImage2D(GL_TEXTURE_2D, // Type of texture
                 0,             // Pyramid level (for mip-mapping) - 0 is the top level
                 GL_RGB,        // Internal colour format to convert to
                 gWindowWidth,  // Image width  i.e. 640 for Kinect in standard mode
                 gWindowHeight, // Image height i.e. 480 for Kinect in standard mode
                 0,             // Border width in pixels (can either be 1 or 0)
                 GL_RGB,        // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                 GL_FLOAT,      // Image data type
                 data);         // The actual image data itself

    glGenerateMipmap(GL_TEXTURE_2D);
    return data;
}
void UpdateValue(float *data, int comp = 3)
{
    for (int row = 0; row < gWindowHeight; row++)
    {
        for (int col = 0; col < gWindowWidth; col++)
        {
            int bias = (row * gWindowWidth + col) * 3;
            // data[bias + 0] = 0;
            data[bias + 1] += (cMathUtil::RandDouble() - 0.5) / 10;
            data[bias + 2] += (cMathUtil::RandDouble() - 0.5) / 10;
        }
    }
}

void ConvertDepthImageToRGB(const tMatrixXi &depth_image,
                            tMatrixXf &R, tMatrixXf &G, tMatrixXf &B)
{
    R = depth_image.cast<float>();
    G.noalias() = G;
    B.noalias() = B;
    float max_amp = R.cwiseAbs().maxCoeff();
    R /= max_amp;
    G /= max_amp;
    B /= max_amp;
}

void ConvertDepthImageToGLRGBTextureBuffer(const tMatrixXi &depth_image, float *buf)
{
    float max = depth_image.cwiseAbs().maxCoeff();
    // int buf_size = gWindowHeight * gWindowWidth * 3;
    // if (buf.size() != buf_size)
    //     buf.resize(buf_size, 0.0f);
    for (int row = 0; row < gWindowHeight; row++)
    {
        for (int col = 0; col < gWindowWidth; col++)
        {
            int bias = (row * gWindowWidth + col) * 3;
            float value = float(depth_image(row, col)) / max;
            buf[bias + 0] = value;
            buf[bias + 1] = value;
            buf[bias + 2] = value;
        }
    }
}

void InitKinect()
{
    manager = new cKinectManager("nfov_unbinned");
    tMatrixXi depth_image = manager->GetDepthImage();
    printf("get depth image size %d %d\n", depth_image.rows(), depth_image.cols());
    gWindowHeight = depth_image.rows();
    gWindowWidth = depth_image.cols();
}

int main()
{
    InitKinect();
    InitGLFW();
    InitGL();
    auto shaderProgram = InitShader();
    auto VAO = BindVAO();

    GLuint texture;
    // ConvertImageToTexutre(texture);
    float *data = ConvertNoiseToTexture(texture);

    // One time during setup.
    GLuint readFboId = 0;
    glGenFramebuffers(1, &readFboId);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, readFboId);
    glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, texture, 0);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);

    while (!glfwWindowShouldClose(gWindow))
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUseProgram(shaderProgram);
        {
            ConvertDepthImageToGLRGBTextureBuffer(manager->GetDepthImage(), data);
            UpdateValue(data, 3);
            glBindTexture(GL_TEXTURE_2D, texture);

            glTexSubImage2D(
                GL_TEXTURE_2D, 0, 0, 0, gWindowWidth, gWindowHeight, GL_RGB, GL_FLOAT, data);
        }
        // Every time you want to copy the texture to the default framebuffer.
        glBindFramebuffer(GL_READ_FRAMEBUFFER, readFboId);
        glBlitFramebuffer(0, 0, gWindowWidth, gWindowHeight,
                          0, 0, gWindowWidth, gWindowHeight,
                          GL_COLOR_BUFFER_BIT, GL_LINEAR);

        glfwSwapBuffers(gWindow);
        glfwPollEvents();
    }
    glfwTerminate();
}

unsigned int InitShader()
{
    // build and compile our shader program
    // ------------------------------------
    // vertex shader
    unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);
    // check for shader compile errors
    int success;
    char infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n"
                  << infoLog << std::endl;
    }
    // fragment shader
    unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);
    // check for shader compile errors
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n"
                  << infoLog << std::endl;
    }
    // link shaders
    unsigned int shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    // check for linking errors
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success)
    {
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n"
                  << infoLog << std::endl;
    }
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    return shaderProgram;
}

unsigned int BindVAO()
{
    // set up vertex data (and buffer(s)) and configure vertex attributes
    // ------------------------------------------------------------------
    float vertices[] = {
        0.5f, 0.5f, 0.0f,   // top right
        0.5f, -0.5f, 0.0f,  // bottom right
        -0.5f, -0.5f, 0.0f, // bottom left
        -0.5f, 0.5f, 0.0f   // top left
    };
    unsigned int indices[] = {
        // note that we start from 0!
        0, 1, 3, // first Triangle
        1, 2, 3  // second Triangle
    };
    unsigned int VBO, VAO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    GLuint position_attrib_idx_in_shader = 0;
    glVertexAttribPointer(position_attrib_idx_in_shader, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(position_attrib_idx_in_shader);

    // note that this is allowed, the call to glVertexAttribPointer registered VBO as the vertex attribute's bound vertex buffer object so afterwards we can safely unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // remember: do NOT unbind the EBO while a VAO is active as the bound element buffer object IS stored in the VAO; keep the EBO bound.
    //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    // You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but this rarely happens. Modifying other
    // VAOs requires a call to glBindVertexArray anyways so we generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
    glBindVertexArray(0);
    return VAO;
}
void MouseMoveEventCallback(GLFWwindow *window, double xpos, double ypos)
{
    std::cout << "[log] mouse move to " << xpos << " " << ypos << std::endl;
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
    glfwSwapBuffers(gWindow);
}

void ScrollCallback(GLFWwindow *window, double xoff, double yoff)
{
    std::cout << "scroll: x y = " << xoff << " " << yoff << std::endl;
    // gScene->ScrollEvent(yoff);
}

// using namespace cv;

// int width = I.cols;
// int height = I.rows
//                  GLubyte *
//              pixels;

// void get_img()
// {
//     // Mat I = imread("now.png");
//     // cv::imshow("hello", I);
//     // cv::waitKey(0);
//     cv::Mat image = cv::imread("now.png");
//     //cv::Mat flipped;
//     //cv::flip(image, flipped, 0);
//     //image = flipped;
//     if (image.empty())
//     {
//         std::cout << "image empty" << std::endl;
//     }
//     else
//     {
//         GLuint textureTrash = 0;
//         cv::flip(image, image, 0);
//         glGenTextures(1, &textureTrash);
//         glBindTexture(GL_TEXTURE_2D, textureTrash);

//         glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
//         glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

//         // Set texture clamping method
//         glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
//         glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

//         glTexImage2D(GL_TEXTURE_2D,    // Type of texture
//                      0,                // Pyramid level (for mip-mapping) - 0 is the top level
//                      GL_RGB,           // Internal colour format to convert to
//                      image.cols,       // Image width  i.e. 640 for Kinect in standard mode
//                      image.rows,       // Image height i.e. 480 for Kinect in standard mode
//                      0,                // Border width in pixels (can either be 1 or 0)
//                      GL_BGR,           // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
//                      GL_UNSIGNED_BYTE, // Image data type
//                      image.ptr());     // The actual image data itself

//         glGenerateMipmap(GL_TEXTURE_2D);
//     }
// }

// void main(int argc, char **argv)
// {
//     get_img();
//     // glutInit(&argc, argv);
//     // glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
//     // glutInitWindowSize(width, height);
//     // glutInitWindowPosition(100, 100);
//     // glutCreateWindow("OpenGL");
//     // glutDisplayFunc(display);

//     // glutMainLoop();
//     // free(pixels);
// }