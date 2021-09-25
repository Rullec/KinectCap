#pragma once
#include "GLUtil.h"
#include <memory>
#include <string>
#include <vector>
#include "utils/MathUtil.h"

class cRender
{
public:
    cRender(int window_height, int window_width);
    void Init();
    unsigned int BindVAO();
    GLFWwindow *GetWindow();
    void UseProgram();
    void UpdateTextureFromDepthImage(const tMatrixXi &depth_image);
    void PostUpdate();

protected:
    unsigned int InitShader();
    int mHeight, mWidth;
    unsigned int mShaderProgram;
    GLFWwindow *mWindow;
    const int mStartX = 100, mStartY = 100;
    GLuint mTextureId, mFBO;
    std::vector<float> mTextureData;
    const std::string mWindowName = "KinCap";
    void InitGL();
    void InitTextureAndFBO();
    void CreateTexture(GLuint &texture, std::vector<float> &texture_data, int width, int height) const;
    void CreateFBOFromTexture(GLuint &fbo, GLuint texture);

    void UpdateTextureData(GLuint texture, float *data);
    void UpdateFBO(GLuint fbo);
};
