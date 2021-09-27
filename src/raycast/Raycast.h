#pragma once
#include "utils/DefUtil.h"
#include "geometries\\OptixRaycaster.h"
namespace Json
{
    class Value;
};
SIM_DECLARE_CLASS_AND_PTR(cKinematicBody);
class cDepthImageCaster : protected cOptixRaycaster
{
public:
    explicit cDepthImageCaster();
    virtual void CalcResult(
        const tVector3f &cam_pos,
        const tVector3f &cam_center,
        const tVector3f &cam_up,
        float fov,
        int height,
        int width,
        std::vector<float> &data);

protected:
    virtual void Init();
    virtual void InitObstacles();
};
SIM_DECLARE_PTR(cDepthImageCaster);