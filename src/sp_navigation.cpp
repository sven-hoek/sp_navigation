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
	image_transport::SubscriberFilter imgSub_l_; /**< Left image msg. */
	image_transport::SubscriberFilter imgSub_r_; /**< right image msg. */
	message_filters::Subscriber<sensor_msgs::CameraInfo> camInfoSub_l_; /**< Left camera info msg. */
	message_filters::Subscriber<sensor_msgs::CameraInfo> camInfoSub_r_; /**< Right camera info msg. */
	message_filters::Synchronizer<SyncPolicy> sync_; /**< Stereo topic synchronizer. */
	image_transport::Publisher imgPub_l_; /**< Left image publisher. */
	image_transport::Publisher imgPub_r_; /**< Right image publisher. */
	
	public:
	image_geometry::StereoCameraModel stCamModel_;
	cv_bridge::CvImageConstPtr imgConstPtr_l_;
	cv_bridge::CvImageConstPtr imgConstPtr_r_;


	/* Constructor
	 *
	 * */
	StereoSubscriber(ros::NodeHandle& nh, const std::string nameToResolve) :
		it_(nh),
		sync_(3)
	{
		std::string stereoNamespace;
		stereoNamespace = nh.resolveName(nameToResolve);

		// Subscribe to stereo cameras
		imgSub_l_.subscribe(it_, ros::names::clean(stereoNamespace + "/left/image_rect"), 3);
		imgSub_r_.subscribe(it_, ros::names::clean(stereoNamespace + "/right/image_rect"), 3);
		camInfoSub_l_.subscribe(nh, ros::names::clean(stereoNamespace + "/left/camera_info"), 3);
		camInfoSub_r_.subscribe(nh, ros::names::clean(stereoNamespace + "/right/camera_info"), 3);
		sync_.connectInput(imgSub_l_, imgSub_r_, camInfoSub_l_, camInfoSub_r_),
		sync_.registerCallback(boost::bind(&StereoSubscriber::syncCallback, this, _1, _2, _3, _4));
		
		// Initialize publishers
		imgPub_l_ = it_.advertise("sp_navigation/left/image_mono", 1);
		imgPub_r_ = it_.advertise("sp_navigation/right/image_mono", 1);
	}

	/* Callback method for synchronized stereo images
	 *
	 * */
	void syncCallback(const sensor_msgs::Image::ConstPtr& lImage, const sensor_msgs::Image::ConstPtr& rImage, const sensor_msgs::CameraInfo::ConstPtr& lCamInfo, const sensor_msgs::CameraInfo::ConstPtr& rCamInfo)
	{
		ROS_INFO_STREAM_NAMED("Subscriber Test", "got callback.");
		try
		{
			stCamModel_.fromCameraInfo(lCamInfo, rCamInfo);
			imgConstPtr_l_ = cv_bridge::toCvShare(lImage, "bgr8");
			imgConstPtr_r_ = cv_bridge::toCvShare(rImage, "bgr8");
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		catch (...)
		{
			ROS_ERROR("Error in syncCallback!");
			return;
		}
	}

	
	/* Simple method to publish cv::Mat images
	 *
	 * */
	void publishCVImages(const cv::Mat& lImage, const cv::Mat& rImage)
	{
		cv_bridge::CvImage cvImLeft(std_msgs::Header(), "bgr8", lImage);
		cv_bridge::CvImage cvImRight(std_msgs::Header(), "bgr8", rImage);
		imgPub_l_.publish(cvImLeft.toImageMsg());
		imgPub_r_.publish(cvImRight.toImageMsg());
	}
};


class VisualOdometry
{
	private:
		
	
	public:
		void getImagePair(const StereoSubscriber& stereoSub)
		{
			
		}
};

} //End of namespace


int main(int argc, char** argv)
{
	ros::init(argc, argv, "sp_navigation");
	ros::NodeHandle nh;
	sp_navigation::StereoSubscriber stereoSub(nh, "stereo");

	cv::Mat left, right;
	cv::ORB oFDDE;	// ORB feature detector and descriptor extractor
	cv::BFMatcher bfMatcher(cv::NORM_HAMMING, true); // Bruteforce matcher with implemented cross-checking if second arg is 'true'
	
	ros::Rate loopRate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		if (stereoSub.imgConstPtr_l_ != NULL && stereoSub.imgConstPtr_r_ != NULL)
		{
			stereoSub.imgConstPtr_l_->image.copyTo(left);
			stereoSub.imgConstPtr_r_->image.copyTo(right);
			
			cv::circle(left, cv::Point(100, 100), 10, CV_RGB(255,0,0));
			cv::circle(right, cv::Point(100, 100), 10, CV_RGB(0,255,0));
			
			
			stereoSub.publishCVImages(left, right);
		}
		
		
		loopRate.sleep();
	}
	
	ros::spin();
}
