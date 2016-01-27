#ifndef STEREO_SUBSCRIBER_HPP
#define STEREO_SUBSCRIBER_HPP

#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/stereo_camera_model.h>
#include <cv_bridge/cv_bridge.h>

namespace sp_navigation
{
/**
 * Class for synchronized image subscribing
 * This class subscribes to each two camera image and info topics and stores the data until new data was received.
 */
class StereoSubscriber
	{
	private:
		image_transport::ImageTransport it_; /**< Image transport handle. */
		image_transport::SubscriberFilter imgSub_l_; /**< Left image msg. */
		image_transport::SubscriberFilter imgSub_r_; /**< Right image msg. */
		message_filters::Subscriber<sensor_msgs::CameraInfo> camInfoSub_l_; /**< Left camera info msg. */
		message_filters::Subscriber<sensor_msgs::CameraInfo> camInfoSub_r_; /**< Right camera info msg. */
		message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> > sync_; /**< Stereo topic synchronizer. */
		image_transport::Publisher imgPub_l_; /**< Left image publisher. */
		image_transport::Publisher imgPub_r_; /**< Right image publisher. */

	public:
		image_geometry::StereoCameraModel stCamModel_; /**< StereoCameraModel with newest data */
		cv_bridge::CvImageConstPtr imgConstPtr_l_; /**< Pointer to current left image data */
		cv_bridge::CvImageConstPtr imgConstPtr_r_; /**< Pointer to current right image data */
		unsigned int imgIdx;
		StereoSubscriber(ros::NodeHandle& nh, const std::string stereoNamespace);
		void syncCallback(const sensor_msgs::Image::ConstPtr& lImage, const sensor_msgs::Image::ConstPtr& rImage, const sensor_msgs::CameraInfo::ConstPtr& lCamInfo, const sensor_msgs::CameraInfo::ConstPtr& rCamInfo);
		void publishCVImages(const cv::Mat& lImage, const cv::Mat& rImage);
	};
}
#endif
