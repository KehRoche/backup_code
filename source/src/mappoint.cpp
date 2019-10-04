#include "ownslam/common_include.h"
#include "ownslam/mappoint.h"

namespace ownslam
{
   MapPoint::MapPoint()
   :id_(-1), pos_(Vector3d(0,0,0),norm_(Vector3d(0,0,0),good_(true),visable_times_(0)),matched_timer_(0))
   {

   }
   MapPoint::MapPoint()
   : id_(id), pos_(position), norm_(norm), good_(true), visible_times_(1), matched_times_(1), descriptor_(descriptor)
   {
        observed_frames_.push_back(frame);
   }
   MapPoint::Ptr MapPoint::createMapPoint()
   {
       return MapPoint::Ptr(
                   new MatPoint(factory_id_++, Vector3d(0,0,0),Vector3d(0,0,0))
                   );
   }
   MapPoint::Ptr MapPoint::createMapPoint(
           const Vector3d& pos_world,
           const Vector3d& norm,
           const Mat& descriptor,
           Frame* frame)
   {
       return MapPoint::Ptr(
                   new MapPoint(factory_id_++, pos_world, norm, frame, descriptor)
                   );
   }
   unsigned long MapPoint::factory_id_ = 0;
}