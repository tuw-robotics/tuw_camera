#include <nodelet/nodelet.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <memory>

#include <ros/ros.h>
namespace tuw_opencv_cam
{

class OpencvCamNodelet : public nodelet::Nodelet
{
    ros::Subscriber sub_odometry_;  /// Subscriber to the odometry measurements
    std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster;
public:
    virtual void onInit();
    void callbackOdometry ( const nav_msgs::OdometryPtr &odom );
};

}
