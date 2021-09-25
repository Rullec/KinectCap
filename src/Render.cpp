#include "Render.h"
#include "RenderCallback.h"
#include <iostream>

cRender::cRender(int height, int width) : mHeight(height), mWidth(width)
{
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
            data[bias + 0] = float(row) / float(height);
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

void cRender::UpdateTextureData(GLuint texture, float *data)
{
    glBindTexture(GL_TEXTURE_2D, texture);

    glTexSubImage2D(
        GL_TEXTURE_2D, 0, 0, 0, mWidth, mHeight, GL_RGB, GL_FLOAT, data);
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

void cRender::PostUpdate()
{

    // UpdateValue(data, 3);
    UpdateTextureData(mTextureId, mTextureData.data());

    // Every time you want to copy the texture to the default framebuffer.
    UpdateFBO(mFBO);

    glfwSwapBuffers(mWindow);
    glfwPollEvents();
}

void cRender::UpdateTextureFromDepthImage(const tMatrixXi &depth_image)
{
    float max = 1000;
    int height = depth_image.rows();
    int width = depth_image.cols();

    for (int row = 0; row < height; row++)
    {
        for (int col = 0; col < width; col++)
        {
            int bias = (row * width + col) * 3;
            float value = float(depth_image(height - 1 - row, col)) / max;
            mTextureData[bias + 0] = value;
            mTextureData[bias + 1] = value;
            mTextureData[bias + 2] = value;
        }
    }

}