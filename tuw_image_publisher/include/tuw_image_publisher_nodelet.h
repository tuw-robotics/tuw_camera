#ifndef TUW_IMAGE_PUBLISHER_NOTELET_H
#define TUW_IMAGE_PUBLISHER_NOTELET_H

#include <nodelet/nodelet.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <tuw_image_publisher/ImagePublisherConfig.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/core/core.hpp>
#include <memory>

#include <ros/ros.h>
namespace tuw_image_publisher
{

class ImagePublisherNodelet : public nodelet::Nodelet
{
    ros::Subscriber sub_odometry_;  /// Subscriber to the odometry measurements
public:
    virtual void onInit();
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::CameraPublisher pub_camera_;
    sensor_msgs::CameraInfo camera_info_;
    std::vector<std::string> image_files_;
    std::string image_last_;
    std::size_t image_idx_;
    cv::Mat image_;
    ros::Timer timer_;
    ros::NodeHandle private_nh_;
      
    tuw_image_publisher::ImagePublisherConfig config_;
    dynamic_reconfigure::Server<tuw_image_publisher::ImagePublisherConfig> reconfigureServer_; /// parameter server 
    dynamic_reconfigure::Server<tuw_image_publisher::ImagePublisherConfig>::CallbackType reconfigureFnc_; /// parameter server stuff general use
    void callbackReconfigure ( tuw_image_publisher::ImagePublisherConfig &config, uint32_t level ); /// callback function on incoming parameter changes for general use
    int files_in_folder ( std::vector<std::string> &files, std::string folder,  const std::string regx = "(.*)$");
    void publish (const ros::TimerEvent& event);
};

}


#endif