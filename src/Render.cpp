#include "Render.h"
#include "RenderCallback.h"
#include <iostream>
#include "utils/DefUtil.h"
#include "imgui.h"
#include "backends\imgui_impl_glfw.h"
#include "backends\imgui_impl_opengl3.h"
#include "KinectManagerImGui.h"
#include "KinectResource.h"
#include "GLFW/glfw3.h"
#include "GLFW/glfw3native.h"

const int gui_height = 200;
cRender::cRender(int height, int width) : mHeight(height), mWidth(width)
{
    mNeedToUpdateImGuiWindowPos = true;
}
cRender::~cRender()
{
    // ImGui_ImplOpenGL3_Shutdown();
    // ImGui_ImplGlfw_Shutdokwn();
    // ImGui::DestroyContext();
}
void cRender::InitGL()
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

    mWindow = glfwCreateWindow(mWidth, mHeight, mWindowName.c_str(), NULL, NULL);
    if (mWindow == NULL)
    {
        std::cout << "[error] Failed to create GLFW window" << std::endl;
        glfwTerminate();
    }
    glfwSetWindowPos(mWindow, mStartX, mStartY);
    glfwMakeContextCurrent(mWindow);

    glfwSwapInterval(1); // enable vsync from ImGUI

    glfwSetKeyCallback(mWindow, KeyEventCallback);
    glfwSetCursorPosCallback(mWindow, MouseMoveEventCallback);
    glfwSetMouseButtonCallback(mWindow, MouseButtonEventCallback);
    glfwSetInputMode(mWindow, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    glfwSetFramebufferSizeCallback(mWindow, ResizeCallback);
    glfwSetScrollCallback(mWindow, ScrollCallback);
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

    // ------------------ add imgui code -------------------
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.IniFilename = "imgui.ini";
    io.IniSavingRate = 1.0f;
    // (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(mWindow, true);
    const char *glsl_version = "#version 130";
    ImGui_ImplOpenGL3_Init(glsl_version);
}

void cRender::Init()
{
    InitGL();
    mShaderProgram = InitShader();
    InitTextureAndFBO();
}
void cRender::UseProgram()
{
    glUseProgram(mShaderProgram);
}
unsigned int cRender::InitShader()
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

unsigned int cRender::BindVAO()
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

GLFWwindow *cRender::GetWindow()
{
    return mWindow;
}

void cRender::CreateTexture(GLuint &texture, std::vector<float> &texture_data, int width, int height) const
{
    texture = 0;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);

    texture_data.resize(height * width * 3);
    float *data = texture_data.data();
    memset(data, 0, sizeof(data));

    for (int row = 0; row < height; row++)
    {
        for (int col = 0; col < width; col++)
        {
            int bias = (row * width + col) * 3;
            // data[bias + 0] = float(row) / float(height);
            // data[bias + 1] = 0;
            // data[bias + 2] = 0;
            data[bias + 0] = 0.2;
            data[bias + 1] = 0.3;
            data[bias + 2] = 0.4;
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
                 width,         // Image width  i.e. 640 for Kinect in standard mode
                 height,        // Image height i.e. 480 for Kinect in standard mode
                 0,             // Border width in pixels (can either be 1 or 0)
                 GL_RGB,        // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                 GL_FLOAT,      // Image data type
                 data);         // The actual image data itself

    glGenerateMipmap(GL_TEXTURE_2D);
}

void cRender::CreateFBOFromTexture(GLuint &fbo, GLuint texture)
{
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo);
    glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, texture, 0);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
}

// void cRender::UpdateTextureData(GLuint texture, float *data)
#include "utils/LogUtil.h"
void cRender::UpdateTextureData(GLuint texture, std::vector<float *> data_array, const tEigenArr<tVector2i> &shape_array, const tEigenArr<tVector2i> &st_array)
{
    glBindTexture(GL_TEXTURE_2D, texture);

    for (int i = 0; i < shape_array.size(); i++)
    {
        int height = shape_array[i][0];
        int width = shape_array[i][1];
        int st_height = st_array[i][0];
        int st_width = st_array[i][1];
        float *data = data_array[i];
        printf("[debug] update texutre data at %d %d for shape %d %d\n",
               st_height, st_width,
               height, width);
        glTexSubImage2D(GL_TEXTURE_2D, 0, st_width, st_height, width, height, GL_RGB, GL_FLOAT, data);
        // glTexSubImage2D(
        //     GL_TEXTURE_2D, 0, 0, 0, mWidth, mHeight, GL_RGB, GL_FLOAT, data);
    }
}

void cRender::UpdateFBO(GLuint fbo)
{

    glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo);
    glBlitFramebuffer(0, 0, gWindowWidth, gWindowHeight,
                      0, 0, gWindowWidth, gWindowHeight,
                      GL_COLOR_BUFFER_BIT, GL_LINEAR);
}

void cRender::InitTextureAndFBO()
{
    CreateTexture(mTextureId, mTextureData, mWidth, mHeight);
    // One time during setup.
    CreateFBOFromTexture(mFBO, mTextureId);
}

// float *cRender::GetTextureData()
// {
//     return mTextureData.data();
// }

tEigenArr<tVector2i> image_st_array;
tEigenArr<tVector2i> image_shape_array;
std::vector<float *> rendering_resouce_array;
#include "utils/TimeUtil.hpp"
cTimePoint st = cTimeUtil::GetCurrentTime_chrono();
void cRender::PostUpdate()
{

    // UpdateValue(data, 3);
    UpdateTextureData(mTextureId, rendering_resouce_array, image_shape_array, image_st_array);

    // Every time you want to copy the texture to the default framebuffer.
    UpdateFBO(mFBO);

    // rendering imgui
    {

        // bool show_demo_window = true;
        // bool show_another_window = false;
        // ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
        {

            ImVec2 init_window_size = ImVec2(250, 180);
            ImGui::SetNextWindowSize(init_window_size, ImGuiCond_FirstUseEver);
            if (mNeedToUpdateImGuiWindowPos == true)
            {
                ImGui::SetNextWindowPos(ImVec2(float(gWindowWidth) - init_window_size.x, 0),
                                        ImGuiCond_Always);
                mNeedToUpdateImGuiWindowPos = false;
            }

            ImGuiWindowFlags window_flags = 0;
            // window_flags |= ImGuiWindowFlags_NoMove;
            // window_flags |= ImGuiWindowFlags_NoResize;
            bool open = false;
            bool *p_open = &open;

            ImGui::Begin("kinect setting", p_open, window_flags);
            if (mKinectManager != nullptr)
                mKinectManager->UpdateGui();

            // show fps

            {
                cTimePoint cur = cTimeUtil::GetCurrentTime_chrono();
                int fps = int(1.0 / (cTimeUtil::CalcTimeElaspedms(st, cur) * 1e-3));
                ImGui::NewLine();
                ImGui::Text("FPS %d", fps);
                st = cur;
            }
            // ImGui::ShowDemoWindow();
            ImGui::End();
        }

        // Rendering
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }
    glfwSwapBuffers(mWindow);
}

void cRender::UpdateTextureFromDepthImage(const tMatrixXi &depth_image)
{
    float max = 1000;
    int input_height = depth_image.rows();
    int input_width = depth_image.cols();

    // check the shape
    {
        if (input_height != mHeight || input_width != mWidth)
        {
            printf("cur window size %d %d, input depth size %d %d, need to resize\n", input_height, input_width, mHeight, mWidth);
            Resize(input_height, input_width);
        }
    }
    for (int row = 0; row < input_height; row++)
    {
        for (int col = 0; col < input_width; col++)
        {
            int bias = (row * input_width + col) * 3;
            float value = float(depth_image(input_height - 1 - row, col)) / max;
            mTextureData[bias + 0] = value;
            mTextureData[bias + 1] = value;
            mTextureData[bias + 2] = value;
        }
    }
}

void cRender::SetKinectManager(cKinectManagerImGuiPtr mana)
{
    mKinectManager = mana;
}

void cRender::Resize(int height, int width)
{
    glfwSetWindowSize(mWindow, width, height);
    glDeleteTextures(1, &mTextureId);
    glDeleteFramebuffers(1, &mFBO);
    mHeight = height;
    mWidth = width;
    gWindowHeight = mHeight;
    gWindowWidth = mWidth;
    mNeedToUpdateImGuiWindowPos = true;
    InitTextureAndFBO();
    // CreateTexture(mTextureId, mTextureData, mWidth, mHeight);
    printf("resize to %d %d\n", height, width);
}

void cRender::UpdateTextureFromRenderResource(cKinectImageResourcePtr resource)
{
    int new_height = resource->mHeight,
        new_width = resource->mWidth;
    if (new_height != mHeight || new_width != mWidth)
    {
        std::cout << "resize before update tex\n";
        Resize(new_height, new_width);
    }
    memcpy(mTextureData.data(), resource->mData.data(), sizeof(float) * (new_height * new_width * resource->mChannels));
    std::cout << "update texture from rendering resource!\n";
}

void cRender::UpdateTextureFromRenderResourceVec(std::vector<cKinectImageResourcePtr> resource)
{
    // 1. calculate the final size
    int tex_height = 0, tex_width = 0;
    for (int i = 0; i < resource.size(); i++)
    {
        auto res = resource[i];
        tex_height = std::max(res->mHeight + gui_height, tex_height);
        tex_width += res->mWidth;
    }
    if (tex_height != mHeight || tex_width != mWidth)
    {
        printf("[debug] combined height %d width %d, resize\n");
        Resize(tex_height, tex_width);
    }

    {
        image_st_array.clear();
        image_shape_array.clear();
        rendering_resouce_array.clear();

        int cur_width_st = 0;
        for (int i = 0; i < resource.size(); i++)
        {
            auto cur = resource[i];

            image_st_array.push_back(
                tVector2i(0, cur_width_st));
            // std::cout << "resouce " << i << " st = " << cur_width_st << std::endl;
            cur_width_st += cur->mWidth;
            image_shape_array.push_back(
                tVector2i(cur->mHeight, cur->mWidth));
            rendering_resouce_array.push_back(cur->mData.data());
        }
    }
}