#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <image_geometry/stereo_camera_model.h>

namespace sp_navigation
{
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> SyncPolicy;

// Class for receiving and synchronizing stereo images
class StereoSubscriber
{
	private:
	image_transport::ImageTransport it_;
	image_transport::SubscriberFilter limage_sub_; /**< Left image msg. */
	image_transport::SubscriberFilter rimage_sub_; /**< right image msg. */
	message_filters::Subscriber<sensor_msgs::CameraInfo> lcinfo_sub_; /**< Left camera info msg. */
	message_filters::Subscriber<sensor_msgs::CameraInfo> rcinfo_sub_; /**< Right camera info msg. */
	message_filters::Synchronizer<SyncPolicy> sync_; /**< Stereo topic synchronizer. */
	image_transport::Publisher limage_pub_; /**< Left image publisher. */
	image_transport::Publisher rimage_pub_; /**< Right image publisher. */
	
	public:
	image_geometry::StereoCameraModel cam_model_;
	cv_bridge::CvImageConstPtr limage_cptr_;
	cv_bridge::CvImageConstPtr rimage_cptr_;


	/* Constructor
	 *
	 * */
	StereoSubscriber(ros::NodeHandle& nh, const std::string nameToResolve) :
		it_(nh),
		sync_(3)
	{
	    std::string stereo_namespace;
	    stereo_namespace = nh.resolveName(nameToResolve);

	    // Subscribe to stereo cameras
	    limage_sub_.subscribe(it_, ros::names::clean(stereo_namespace + "/left/image_rect"), 3);
	    rimage_sub_.subscribe(it_, ros::names::clean(stereo_namespace + "/right/image_rect"), 3);
	    lcinfo_sub_.subscribe(nh, ros::names::clean(stereo_namespace + "/left/camera_info"), 3);
	    rcinfo_sub_.subscribe(nh, ros::names::clean(stereo_namespace + "/right/camera_info"), 3);
	    sync_.connectInput(limage_sub_, rimage_sub_, lcinfo_sub_, rcinfo_sub_),
	    sync_.registerCallback(boost::bind(&StereoSubscriber::syncCallback, this, _1, _2, _3, _4));
	    
	    // Initialize publishers
	    limage_pub_ = it_.advertise("sp_navigation/left/image_mono", 1);
	    rimage_pub_ = it_.advertise("sp_navigation/right/image_mono", 1);
	}

	/* Callback method for synchronized stereo images
	 *
	 * */
	void syncCallback(const sensor_msgs::Image::ConstPtr& limage, const sensor_msgs::Image::ConstPtr& rimage, const sensor_msgs::CameraInfo::ConstPtr& lcinfo, const sensor_msgs::CameraInfo::ConstPtr& rcinfo)
	{
	  ROS_INFO_STREAM_NAMED("Subscriber Test", "got callback.");
	  try
	  {
		cam_model_.fromCameraInfo(lcinfo, rcinfo);
		limage_cptr_ = cv_bridge::toCvShare(limage, "mono8");
		rimage_cptr_ = cv_bridge::toCvShare(rimage, "mono8");
	  }
	  catch (...)
	  {
	    ROS_ERROR("Error in syncCallback!");
	  }
	}
	
	/* Simple method to publish cv::Mat images
	 *
	 * */
	void publishCVImages(const cv::Mat& limage, const cv::Mat& rimage)
	{
		cv_bridge::CvImage cvImLeft(std_msgs::Header(), "mono8", limage);
		cv_bridge::CvImage cvImRight(std_msgs::Header(), "mono8", rimage);
		limage_pub_.publish(cvImLeft.toImageMsg());
		rimage_pub_.publish(cvImRight.toImageMsg());
	}
};

} //End of namespace


int main(int argc, char** argv)
{
	ros::init(argc, argv, "sp_navigation");
	ros::NodeHandle nh;
	
	sp_navigation::StereoSubscriber stereo_sub(nh, "stereo");

	cv::Mat left, right;
	
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		if (stereo_sub.limage_cptr_ != NULL && stereo_sub.rimage_cptr_ != NULL)
		{
			stereo_sub.limage_cptr_->image.copyTo(left);
			stereo_sub.rimage_cptr_->image.copyTo(right);
			

			
			stereo_sub.publishCVImages(left, right);
		}
		
		
		loop_rate.sleep();
	}
	
	ros::spin();
}
