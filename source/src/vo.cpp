#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "ownslam/config.h"
#include "ownslam/vo.h"
#include "ownslam/g2o_types.h"
#include "ownslam/camera.h"

namespace ownslam
{
    VisualOdometry::VisualOdometry():
    state_(INITIALING), ref_(nullptr), curr_(nullptr), map_(new Map), num_lost_(0), num_inliers_(0),matcher_flann_(new cv::flann::LshIndexParams(5,10,2))
    {
        num_of_features_ = Config::get<int>("number_of_features");
        scale_factor_ = Config::get<double>("scale_factor");
        level_pyramid_      = Config::get<int> ( "level_pyramid" );
        match_ratio_        = Config::get<float> ( "match_ratio" );
        max_num_lost_       = Config::get<float> ( "max_num_lost" );
        min_inliers_        = Config::get<int> ( "min_inliers" );
        key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
        key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
        map_point_erase_ratio_ = Config::get<double> ( "map_point_erase_ratio" );
        orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
    }
    VisualOdometry::~VisualOdometry()
    {

    }
    bool VisualOdometry::addFrame(Frame::Ptr frame)
    {
        switch(state_)
        {
            case INITIAZING:
            {
                state_ = OK;
                curr_ = ref_ = frame;
                extractKeyPoints();
                computerDescriptors();
                addKeyFrame();
                break;
            }
            case OK:
        {
            curr_= frame;
            curr_->T_c_w_= ref_->T_c_w_;
            extractKeyPoints();
            computeDescriptors();
            featureMatching();
            poseEstimationPnP();

        }
        }
    }
}

void VisualOdometry::extractKeyPoints()
{
    boost::timer timer;
    orb_->detect(curr_->color_,keypoints_curr_);
    cout<<"extract keypoints cost time:"<<timer.elapsed()<<endl;
}

void VisualOdometry::computeDescriptors()
{
    boost::timer timer;
    orb_->computer(curr_->color_,keypoints_curr,descriptors_curr_);

}
void VisualOdometry::featureMatching()
{
    vector<cv::DMatch> matches;

    Map desp_map;
    vector<MapPoints::Ptr> candidate;
    //for(type x:varible) range-based for
    for(auto& allpoints: map_->map_points_)
    {
        //data was saved as hash bucket .first return its key index .second return its pointer
        MapPoint::Ptr& p = allpoints.second;

        if( curr_->isInFrame(p_->pos_))
        {
            //the times of present point being seeing increase one
            p_->visible_times_++;
            candiate.push_back(p);
            //push back for a vector, store in it
            candidate.push_back(p);
            desp_map.push_back(p->descriptor_);

        }
    }
    FlannBasedMatcher.match(desp_map,desciptors_curr_,matches);
    //calculate the miner distance use min_element iter as range and lambeda as compere function
    float min_ds = std::min_element(
                matches.begin(),matches.end(),[](const cv::DMatch& m1, const cv::DMatch& m2)
    {
        return m1.distance < m2.distance;
    })->distance;

    match_3dpts_.clear();
    match_2dkp_index_.clear();
    for(cv::DMatch& m : mathches)
    {
        if(m.distance<max<float>(min_dis*match_ratio_,30.0))
        {
            match_3dpts_.push_back(candiate[m.queryIdx]);
            match_2dkp_index_.push_back(m.trainIdx);
        }
    }

    void VisualOdometry::poseEstimationPnP()
    {

        vector<cv::Point3f> pts3d;
        vector<cv::Point2f> pts2d;

        for(int index:match_)
    }
}
void VisualOdometry::poseEstimationPnP()
{
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;

    for(int index:match_2dkp_index_)
    {
        pts2d.push_back(keypoints_curr_[index].pt);
    }
    for(MapPoint::Ptr pt:match_3dpts_)
    {
        pts3d.push_back(pt->getPositionCV());
    }

    Mat K = (cv::Mat_<double> (3,3)<<
             ref_->camera_->fx_,0,ref_camera_->cx_,
             0, ref_->camera_->fy_,ref_->camera_->cy,
             0,0,1
             );
    Mat rvec, tvec, inliers;
    cv::solvePnPRansc(pts3d,pts2d,K,Mat(),rvec,tvec,false,100,4.0,0.99,inliers);

}
bool VisualOdometry::checkEstimatedPose()
{
    if(num_inliers_<min_inliers)
    {
        cout<<"reject because inlier is too small:"<<num_inliers_<<endl;
        return false;
    }

    SE T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();
    if(d.norm())
}
bool VisualOdometry::checkKeyFrame()
{
    SE3 T_r_c = ref_->T_c_w_*T_c_w_estimated_inverse();
}
void VisualOdometry::addKeyFrame()
{
    if(map_->keyframes_.emtry())
    {
        for(size_t i = 0;i<keypoints_curr_.size();i++)
        {
            double d = curr_->findDepth(keypoints_curr_[i]);
            if(d<0)
                continue;
            Vector3d n = p_world-ref_->getCamCenter();

        }
    }
}
