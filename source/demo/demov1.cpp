#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ownslam/config.h"
#include "ownslam/vo.h"

int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }

    ownslam::Config::setParameterFile ( argv[1] );
    ownslam::VisualOdometry::Ptr vo ( new ownslam::VisualOdometry );

    string dataset_dir = ownslam::Config::get<string> ( "dataset_dir" );
    cout<<"dataset: "<<dataset_dir<<endl;
    ifstream fin ( dataset_dir+"/associate.txt" );
    if ( !fin )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }

    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
    while ( !fin.eof() )
    {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
        rgb_times.push_back ( atof ( rgb_time.c_str() ) );
        depth_times.push_back ( atof ( depth_time.c_str() ) );
        rgb_files.push_back ( dataset_dir+"/"+rgb_file );
        depth_files.push_back ( dataset_dir+"/"+depth_file );

        if ( fin.good() == false )
            break;
    }

    ownslam::Camera::Ptr camera ( new ownslam::Camera );

//    for ( int i=0; i<rgb_files.size(); i++ )
//    {
//        cout<<"****** loop "<<i<<" ******"<<endl;
//        Mat color = cv::imread ( rgb_files[i] );
//        Mat depth = cv::imread ( depth_files[i], -1 );
//        if ( color.data==nullptr || depth.data==nullptr )
//            break;
//        ownslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
//        pFrame->camera_ = camera;
//        pFrame->color_ = color;
//        pFrame->depth_ = depth;
//        pFrame->time_stamp_ = rgb_times[i];

//        boost::timer timer;
//        vo->addFrame ( pFrame );
//        cout<<"VO costs time: "<<timer.elapsed() <<endl;

//        if ( vo->state_ == myslam::VisualOdometry::LOST )
//            break;
//        SE3 Twc = pFrame->T_c_w_.inverse();

//        // show the map and the camera pose
//        cv::Affine3d M (
//            cv::Affine3d::Mat3 (
//                Twc.rotation_matrix() ( 0,0 ), Twc.rotation_matrix() ( 0,1 ), Twc.rotation_matrix() ( 0,2 ),
//                Twc.rotation_matrix() ( 1,0 ), Twc.rotation_matrix() ( 1,1 ), Twc.rotation_matrix() ( 1,2 ),
//                Twc.rotation_matrix() ( 2,0 ), Twc.rotation_matrix() ( 2,1 ), Twc.rotation_matrix() ( 2,2 )
//            ),
//            cv::Affine3d::Vec3 (
//                Twc.translation() ( 0,0 ), Twc.translation() ( 1,0 ), Twc.translation() ( 2,0 )
//            )
//        );

//        Mat img_show = color.clone();
//        for ( auto& pt:vo->map_->map_points_ )
//        {
//            myslam::MapPoint::Ptr p = pt.second;
//            Vector2d pixel = pFrame->camera_->world2pixel ( p->pos_, pFrame->T_c_w_ );
//            cv::circle ( img_show, cv::Point2f ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,0 ), 2 );
//        }

//        cv::imshow ( "image", img_show );
//        cv::waitKey ( 1 );
//        cout<<endl;
//    }

    return 0;
}
