#ifdef CAMERA_H
#define CAMERA_H

#include "ownslam/common_include.h"

namespace ownslam
{
    class camera
    {
    private:
        /* data */
    public:
        typedef std::shared_ptr<Camera> Ptr;
        float fx_, fy_, cx_, cy_, depth_scale_;

        Camera();
        Camera(float fx, float fy, float cx, float cy, float depth_scale=0):
        fx_( fx ), fy_( fy ),cx_(cx),cy_(cy),depth_scale_(depth_scale)
        {}

        ve


        camera(/* args */);
        ~camera();
    };
    
}