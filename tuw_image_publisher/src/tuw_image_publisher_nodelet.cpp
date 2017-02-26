// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>
#include <tuw_image_publisher_nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/filesystem.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <wordexp.h>
#include <string>
#include <regex>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS ( tuw_image_publisher::ImagePublisherNodelet, nodelet::Nodelet )


// Update the input string.
void autoExpandEnvironmentVariables ( std::string & text ) {
    static std::regex env ( "\\$\\{([^}]+)\\}" );
    std::smatch match;
    while ( std::regex_search ( text, match, env ) ) {
        const char * s = getenv ( match[1].str().c_str() );
        const std::string var ( s == NULL ? "" : s );
        text.replace ( match[0].first, match[0].second, var );
    }
}

namespace tuw_image_publisher {
void ImagePublisherNodelet::onInit() {
    NODELET_DEBUG ( "Initializing nodelet..." );
    private_nh_ = getPrivateNodeHandle();
    /// subscribes to  odometry values
    image_transport_ = std::shared_ptr<image_transport::ImageTransport> ( new image_transport::ImageTransport ( private_nh_ ) );
    pub_camera_ = image_transport::ImageTransport ( private_nh_ ).advertiseCamera ( "image_raw", 1 );

    reconfigureFnc_ = boost::bind ( &ImagePublisherNodelet::callbackReconfigure, this,  _1, _2 );
    reconfigureServer_.setCallback ( reconfigureFnc_ );
}

int ImagePublisherNodelet::files_in_folder ( std::vector<std::string> &files, std::string folder, std::string regx ) {
    using namespace boost::filesystem;
    autoExpandEnvironmentVariables ( folder );
    if ( ( exists ( folder ) && is_directory ( folder ) ) ) {
        NODELET_INFO ( "Folder  %s exits!", folder.c_str() );
    } else {
        NODELET_ERROR ( "Folder  %s does not exit!", folder.c_str() );
    }

    static const std::regex expression ( regx );
    directory_iterator end;
    for ( directory_iterator it ( folder ); it != end; ++it ) {
	const path &p = it->path();
	
        std::string filename = it->path( ).filename( ).string( );
        if ( !is_directory ( *it ) && std::regex_match ( filename, expression ) ) {
            std::string filename_full = it->path( ).c_str();
            files.push_back ( filename_full );
            NODELET_INFO ( "file %s", filename_full.c_str() );
        }
    }
    std::sort ( files.begin(), files.end() );
    return files.size();
}

void ImagePublisherNodelet::callbackReconfigure ( tuw_image_publisher::ImagePublisherConfig &config, uint32_t level ) {
    NODELET_DEBUG ( "callbackReconfigure!" );
    config_ = config;
    image_idx_ = files_in_folder ( image_files_,config_.folder,  config_.file_regx ) - 1;
    timer_ = private_nh_.createTimer(ros::Duration(1.0/config_.publish_rate), &ImagePublisherNodelet::publish, this);
}


void ImagePublisherNodelet::publish (const ros::TimerEvent& event) {
    NODELET_DEBUG ( "publish!" );
    if ( ( image_idx_ ) < 0 ) {
        NODELET_ERROR ( "no image files: %s / %s!", config_.folder.c_str(), config_.file_regx.c_str() );
    }
    try {
	if(image_last_.compare(image_files_[image_idx_]) != 0){
	  NODELET_INFO ( "load image %s", image_files_[image_idx_].c_str() );
	  image_ = cv::imread ( image_files_[image_idx_], CV_LOAD_IMAGE_COLOR );
	  image_last_ = image_files_[image_idx_++];
	  image_idx_ = image_idx_%image_files_.size();
	} 
    } catch ( cv::Exception &e ) {
        NODELET_ERROR ( "Failed to load image %s: %s %s %i", image_files_[image_idx_].c_str(), e.func.c_str(), e.file.c_str(), e.line );
    }
    try {
        sensor_msgs::ImagePtr out_img = cv_bridge::CvImage ( std_msgs::Header(), "bgr8", image_ ).toImageMsg();
        out_img->header.frame_id = config_.frame_id;
        out_img->header.stamp = ros::Time::now();
        camera_info_.header.frame_id = out_img->header.frame_id;
        camera_info_.header.stamp = out_img->header.stamp;

        pub_camera_.publish ( *out_img, camera_info_ );
    } catch ( cv::Exception &e ) {
        NODELET_ERROR ( "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line );
    }
}
}

