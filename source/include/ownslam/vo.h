#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/map.h"

#include <opencv2/features2d/features2d.hpp>

namespace ownslam
{
    class VisualOdometry
    {
        public:
            typedef shared_ptr<VisualOdometry> ptr;
            enum VOState{
                INITIALIZING=1,
                OK=0,
                LOST
            };
            VOState state_;
            Map::Ptr map_;

            Frame::Ptr ref_;
            Frame::Ptr curr_;

            cv::Ptr<cv::ORB> orb_;
            vector<cv::KeyPoint> keypoints_curr_;
            Mat                  descriptors_curr_;
    }
}