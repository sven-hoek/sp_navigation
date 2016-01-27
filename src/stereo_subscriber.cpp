#include "sp_navigation/stereo_subscriber.hpp"

namespace sp_navigation
{
/*
 * Constructor.
 * @param[in] nh The nodehandle to use.
 * @param[in] stereoNamespace Name of the stereo namespace to use.
 * */
StereoSubscriber::StereoSubscriber(ros::NodeHandle& nh, const std::string stereoNamespace) :
	it_(nh),
	sync_(3),
	imgIdx(0)
	{
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


 /*
 * Callback method for synchronized stereo images
 * Converts ROS image messages to openCV images and stores them in member variables.
 * @param[in] lImage Pointer to left image.
 * @param[in] rImage Pointer to right image.
 * @param[in] lCamInfo Pointer to left camera info.
 * @param[in] rCamInfo Pointer to right camera info.
 * */
void StereoSubscriber::syncCallback(const sensor_msgs::Image::ConstPtr& lImage, const sensor_msgs::Image::ConstPtr& rImage, const sensor_msgs::CameraInfo::ConstPtr& lCamInfo, const sensor_msgs::CameraInfo::ConstPtr& rCamInfo)
	{
	ROS_INFO_STREAM_NAMED("Subscriber Test", "got callback.");
	try
		{
		stCamModel_.fromCameraInfo(lCamInfo, rCamInfo);
		imgConstPtr_l_ = cv_bridge::toCvShare(lImage, "bgr8");
		imgConstPtr_r_ = cv_bridge::toCvShare(rImage, "bgr8");
		imgIdx++;
		}
	catch(cv_bridge::Exception& e)
		{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
		}
	catch(...)
		{
		ROS_ERROR("Error in StereoSubscriber::syncCallback!");
		return;
		}
	}


/*
 * Simple method to publish two openCV images in two ROS topics.
 * @param[in] lImage Left image to publish.
 * @param[in] rImage Right image to publish.
 * */
void StereoSubscriber::publishCVImages(const cv::Mat& lImage, const cv::Mat& rImage)
	{
	try
		{
		cv_bridge::CvImage cvImLeft(std_msgs::Header(), "bgr8", lImage);
		cv_bridge::CvImage cvImRight(std_msgs::Header(), "bgr8", rImage);
		imgPub_l_.publish(cvImLeft.toImageMsg());
		imgPub_r_.publish(cvImRight.toImageMsg());
		}
	catch(cv::Exception &e)
		{
		std::cout << e.what() << std::endl;
		return;
		}
	catch(...)
		{
		ROS_ERROR("Error in StereoSubscriber::publishCVImages!");
		}
	}
}
