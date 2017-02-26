// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>
#include <tuw_opencv_cam_nodelet.h>
#include <tf/transform_datatypes.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS ( tuw_opencv_cam::OpencvCamNodelet, nodelet::Nodelet )

namespace tuw_opencv_cam {
void OpencvCamNodelet::onInit() {
    NODELET_DEBUG ( "Initializing nodelet..." );
    ros::NodeHandle &n         = getNodeHandle();
    ros::NodeHandle &private_nh = getPrivateNodeHandle();
    /// subscribes to  odometry values
    tf_broadcaster = std::make_shared<tf::TransformBroadcaster>();
    sub_odometry_ = n.subscribe ( "odom", 1, &OpencvCamNodelet::callbackOdometry, this );
}


/**
 * copies incoming odemetry messages to the base class
 * @param odom
 **/
void OpencvCamNodelet::callbackOdometry ( const nav_msgs::OdometryPtr &odom ) {
    NODELET_DEBUG ( "callbackOdometry..." );

    tf::Transform transform;
    transform.setOrigin ( tf::Vector3 ( 1, 0, 0 ) );
    tf::Quaternion q ( 0, 0, 0, 1 );
    transform.setRotation ( q );
    tf_broadcaster->sendTransform ( tf::StampedTransform ( transform, ros::Time::now(), "map", "odom" ) );
}


}
