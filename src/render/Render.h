#pragma once
#include <memory>
#include <string>
#include <vector>
#include "utils/MathUtil.h"
#include "utils/DefUtil.h"
#include "utils/GLUtil.h"
SIM_DECLARE_CLASS_AND_PTR(cKinectManagerImGui);
SIM_DECLARE_CLASS_AND_PTR(cKinectImageResource);
class cRender
{
public:
    cRender(int window_height, int window_width);
    virtual ~cRender();
    void Init();
    unsigned int BindVAO();
    GLFWwindow *GetWindow();
    void MouseMoveCallback(double xpos, double ypos);
    void UseProgram();
    void UpdateTextureFromRenderResourceVec(std::vector<cKinectImageResourcePtr> resource);
    void PostUpdate();
    void SetKinectManager(cKinectManagerImGuiPtr mana);

protected:
    unsigned int InitShader();
    int mHeight, mWidth;
    unsigned int mShaderProgram;
    GLFWwindow *mWindow;
    const int mStartX = 100, mStartY = 100;
    GLuint mTextureId, mFBO;
    std::vector<float> mTextureData;
    const std::string mWindowName = "KinCap";
    cKinectManagerImGuiPtr mKinectManager;
    std::vector<cKinectImageResourcePtr> mCurRenderingResource;
    bool mNeedToUpdateImGuiWindowPos;
    void InitGL();
    void InitTextureAndFBO();
    void CreateTexture(GLuint &texture, std::vector<float> &texture_data, int width, int height) const;
    void CreateFBOFromTexture(GLuint &fbo, GLuint texture);

    void UpdateTextureFromDepthImage(const tMatrixXi &depth_image);
    void UpdateTextureFromRenderResource(cKinectImageResourcePtr resource);
    void UpdateTextureData(GLuint texture, std::vector<float *> data_array, const tEigenArr<tVector2i> &shape_array, const tEigenArr<tVector2i> &st_array);
    void UpdateFBO(GLuint fbo);

    void Resize(int height, int width);
};
SIM_DECLARE_PTR(cRender);