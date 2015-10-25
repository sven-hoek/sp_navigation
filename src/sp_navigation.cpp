#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <image_geometry/stereo_camera_model.h>

namespace sp_navigation
{

// Class for receiving and synchronizing stereo images
class ImageSubscriber
{
	private:
	image_transport::ImageTransport it_;
	image_transport::SubscriberFilter limage_sub_; /**< Left image msg. */
	image_transport::SubscriberFilter rimage_sub_; /**< right image msg. */
	message_filters::Subscriber<sensor_msgs::CameraInfo> lcinfo_sub_; /**< Left camera info msg. */
	message_filters::Subscriber<sensor_msgs::CameraInfo> rcinfo_sub_; /**< Right camera info msg. */
	//sensor_msgs::CvBridge lbridge_; /**< ROS->OpenCV bridge for the left image. */
	//sensor_msgs::CvBridge rbridge_; /**< ROS->OpenCV bridge for the right image. */
	message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> sync_; /**< Stereo topic synchronizer. */
	
	public:
	//image_geometry::StereoCameraModel cam_model_;
	cv_bridge::CvImageConstPtr limage_cptr_;
	cv_bridge::CvImageConstPtr rimage_cptr_;

	/* Constructor
	 *
	 * */
	ImageSubscriber(ros::NodeHandle& nh) :
		it_(nh),
		sync_(4)
	{
	    std::string stereo_namespace;
	    stereo_namespace = nh.resolveName("stereo");

	    // Subscribe to stereo cameras
	    limage_sub_.subscribe(it_, ros::names::clean(stereo_namespace + "/left/image_rect"), 3);
	    rimage_sub_.subscribe(it_, ros::names::clean(stereo_namespace + "/right/image_rect"), 3);
	    lcinfo_sub_.subscribe(nh, ros::names::clean(stereo_namespace + "/left/camera_info"), 3);
	    rcinfo_sub_.subscribe(nh, ros::names::clean(stereo_namespace + "/right/camera_info"), 3);    
	    sync_.connectInput(limage_sub_, rimage_sub_, lcinfo_sub_, rcinfo_sub_),
	    sync_.registerCallback(boost::bind(&ImageSubscriber::imageCallback, this, _1, _2, _3, _4));
	}

	/* callback method for synchronized stereo images
	 *
	 * */
	void imageCallback(const sensor_msgs::Image::ConstPtr& limage, const sensor_msgs::Image::ConstPtr& rimage, const sensor_msgs::CameraInfo::ConstPtr& lcinfo, const sensor_msgs::CameraInfo::ConstPtr& rcinfo)
	{
	  ROS_INFO_STREAM_NAMED("Viewer Test", "got callback.");
	  try
	  {
		//cam_model_.fromCameraInfo(lcinfo, rcinfo);
		limage_cptr_ = cv_bridge::toCvShare(limage, "mono16");
		rimage_cptr_ = cv_bridge::toCvShare(rimage, "mono16");
	  }
	  catch (...)
	  {
	    ROS_ERROR("Error in imageCallback!");
	  }
	}

};

} //End of namespace


int main(int argc, char** argv)
{
	ros::init(argc, argv, "sp_navigation");
	ros::NodeHandle nh;
	sp_navigation::ImageSubscriber image_sub(nh);
	
	ros::spin();
}
