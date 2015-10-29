#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
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

// Class for receiving and synchronizing stereo images
class StereoSubscriber
	{
	private:
		image_transport::ImageTransport it_;
		image_transport::SubscriberFilter imgSub_l_; /**< Left image msg. */
		image_transport::SubscriberFilter imgSub_r_; /**< right image msg. */
		message_filters::Subscriber<sensor_msgs::CameraInfo> camInfoSub_l_; /**< Left camera info msg. */
		message_filters::Subscriber<sensor_msgs::CameraInfo> camInfoSub_r_; /**< Right camera info msg. */
		message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> > sync_; /**< Stereo topic synchronizer. */
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
	cv::ORB oFDDE(3000);	// ORB feature detector and descriptor extractor
	cv::BFMatcher bfMatcher(cv::NORM_HAMMING, true); // Bruteforce matcher with implemented cross-checking if second arg is 'true'
	
	ros::Rate loopRate(10);
	while (ros::ok())
		{
		// Let ROS spin, callback methods are called
		ros::spinOnce();
		if (stereoSub.imgConstPtr_l_ != NULL && stereoSub.imgConstPtr_r_ != NULL)
			{
			// Copy new pictures
			stereoSub.imgConstPtr_l_->image.copyTo(left);
			stereoSub.imgConstPtr_r_->image.copyTo(right);
			
			// Detect features and compute descriptors
			std::vector<cv::KeyPoint> keyPoints_l, keyPoints_r;
			cv::Mat descriptors_l, descriptors_r;
			oFDDE(left, cv::noArray(), keyPoints_l, descriptors_l);
			oFDDE(right, cv::noArray(), keyPoints_r, descriptors_r);
			std::cout << "KeyPoints left:		" << keyPoints_l.size() << std::endl;
			std::cout << "KeyPoints right:	" << keyPoints_r.size() << std::endl;
			cv::drawKeypoints(left, keyPoints_l, left, cv::Scalar(255,0,0));
			
			// Match features with a descriptor(!)-distance below a threshold
			std::vector<std::vector<cv::DMatch> > matches;
			bfMatcher.radiusMatch(descriptors_l, descriptors_r, matches, 35);
			
			// Only use matches that fulfill the epipolar constraint, thus lying on one horizontal line
			unsigned int numberOfMatches = 0;
			std::vector<cv::DMatch> refinedMatches;
			for(unsigned int i = 0; i < matches.size(); ++i)
				{
				numberOfMatches += matches[i].size() > 0 ? 1 : 0;
				if (matches[i].size() > 0 && fabs(keyPoints_l[matches[i][0].queryIdx].pt.y - keyPoints_r[matches[i][0].trainIdx].pt.y) <= 2)
					{
					refinedMatches.push_back(matches[i][0]);
					cv::line(left, cv::Point(keyPoints_l[matches[i][0].queryIdx].pt.x, keyPoints_l[matches[i][0].queryIdx].pt.y), cv::Point(keyPoints_r[matches[i][0].trainIdx].pt.x, keyPoints_r[matches[i][0].trainIdx].pt.y), cv::Scalar(0,0,200));
					}
				}
			std::cout << "Matches:		" << numberOfMatches << std::endl;
			std::cout << "refined Matches:	" << refinedMatches.size() << std::endl;
			
			// Create Vectors containing only Points from refined matches
			std::vector<cv::KeyPoint> refinedKeyPoints_l, refinedKeyPoints_r;
			std::vector<cv::Point2f> refinedPoints_l, refinedPoints_r;
			for (unsigned int i = 0; i < refinedMatches.size(); ++i)
				{
				refinedKeyPoints_l.push_back(keyPoints_l[refinedMatches[i].queryIdx]);
				refinedKeyPoints_r.push_back(keyPoints_r[refinedMatches[i].trainIdx]);
				
				refinedPoints_l.push_back(keyPoints_l[refinedMatches[i].queryIdx].pt);
				refinedPoints_r.push_back(keyPoints_r[refinedMatches[i].trainIdx].pt);
				}
			cv::drawKeypoints(left, refinedKeyPoints_l, left, cv::Scalar(0, 255, 0));
			
			// Remove outliers by RANSAC
			cv::Mat inlierStatus;
			cv::findFundamentalMat(refinedPoints_l, refinedPoints_r, CV_FM_RANSAC, 3., 0.99, inlierStatus);
			
			std::vector<cv::KeyPoint> ransacProofKPs_l, ransacProofKPs_r;
			for(unsigned int i = 0; i < inlierStatus.rows; ++i)
				{
				if(inlierStatus.ptr<bool>(i)[0])
					{
					ransacProofKPs_l.push_back(refinedKeyPoints_l[i]);
					ransacProofKPs_r.push_back(refinedKeyPoints_r[i]);
					}
				}
			
			std::cout << "RANSAC proof KPs: 	" << ransacProofKPs_l.size() << std::endl << std::endl;
			
			// Draw Matches
			cv::Mat imgMatches;
			cv::drawMatches(left, keyPoints_l, right, keyPoints_r, refinedMatches, imgMatches);
			// Draw RANSAC proof KeyPoints in red
			cv::drawKeypoints(left, ransacProofKPs_l, left, cv::Scalar(0, 0, 200));
			
			stereoSub.publishCVImages(left, imgMatches);
			}
		
		
		loopRate.sleep();
		}
	
	ros::spin();
	}
