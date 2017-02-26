#include <ros/ros.h>
#include <nodelet/loader.h>

int main ( int argc, char **argv )
{
    ros::init ( argc, argv, "ekf" );


    nodelet::Loader manager ( false ); // Don't bring up the manager ROS API
    nodelet::M_string remappings;
    nodelet::V_string my_argv;

    manager.load ( "tuw_opencv_cam", "tuw_opencv_cam/tuw_opencv_cam_nodelet", remappings, my_argv );


    ros::spin();
    return 0;
}
