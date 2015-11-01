#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
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

	cv::Mat left, right, new_l, new_r;
	cv::ORB oFDDE(3000);	// ORB feature detector and descriptor extractor
	cv::BFMatcher bfMatcher(cv::NORM_HAMMING, true); // Bruteforce matcher with implemented cross-checking if second arg is 'true'
	
	tf::TransformBroadcaster tfBr;
	tf::Transform tfVehiOdom;
	tfVehiOdom.setIdentity();
	
	ros::Rate loopRate(12);
	while (ros::ok())
		{
		// Take time of each loop
		double tickTime = (double)cv::getTickCount();
		// Let ROS spin, callback methods are called
		ros::spinOnce();
		if (stereoSub.imgConstPtr_l_ != NULL && stereoSub.imgConstPtr_r_ != NULL)
			{
			// Copy new pictures
			if (new_l.empty() || new_r.empty())
				{
				stereoSub.imgConstPtr_l_->image.copyTo(new_l);
				stereoSub.imgConstPtr_r_->image.copyTo(new_r);
				continue;
				}
			new_l.copyTo(left);
			new_r.copyTo(right);
			stereoSub.imgConstPtr_l_->image.copyTo(new_l);
			stereoSub.imgConstPtr_r_->image.copyTo(new_r);
			
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
			std::vector<unsigned char> inlierStatus;
			cv::findFundamentalMat(refinedPoints_l, refinedPoints_r, CV_FM_RANSAC, 3., 0.99, inlierStatus);
			// Create Vectors containing only Points that have been proven consistent by RANSAC
			std::vector<cv::KeyPoint> ransacProofKPs_l, ransacProofKPs_r;
			std::vector<cv::Point2f> ransacProofPoints_l, ransacProofPoints_r;
			for(unsigned int i = 0; i < inlierStatus.size(); ++i)
				{
				if(inlierStatus[i])
					{
					ransacProofKPs_l.push_back(refinedKeyPoints_l[i]);
					ransacProofKPs_r.push_back(refinedKeyPoints_r[i]);
					
					ransacProofPoints_l.push_back(refinedPoints_l[i]);
					ransacProofPoints_r.push_back(refinedPoints_r[i]);
					}
				}
			std::cout << "RANSAC proof KPs: 	" << ransacProofKPs_l.size() << std::endl;
			
			// Track features in next set of stereo image
			std::vector<cv::Point2f> matchedPointsNewAll_l;
			std::vector<unsigned char> opticalFlowStatus;
			std::vector<float> opticalFlowError;
			try
				{
				cv::calcOpticalFlowPyrLK(left, new_l, ransacProofPoints_l, matchedPointsNewAll_l, opticalFlowStatus, opticalFlowError);
				}
			catch(cv::Exception& e)
				{
				std::cout << e.what() << std::endl;
				}
			std::vector<cv::Point2f> twiceMatchedPoints_l, twiceMatchedPoints_r, matchedPointsNew_l;
			std::vector<cv::KeyPoint> twiceMatchedKPs_l, twiceMatchedKPs_r;
			for(unsigned int i = 0; i < opticalFlowStatus.size(); i++)
				{
				if(opticalFlowStatus[i])
					{
					twiceMatchedPoints_l.push_back(ransacProofPoints_l[i]);
					twiceMatchedPoints_r.push_back(ransacProofPoints_r[i]);
					
					twiceMatchedKPs_l.push_back(ransacProofKPs_l[i]);
					twiceMatchedKPs_r.push_back(ransacProofKPs_r[i]);
					
					matchedPointsNew_l.push_back(matchedPointsNewAll_l[i]);
					}
				}
			std::cout << "Twice matched KPs:	" << twiceMatchedKPs_l.size() << std::endl;
			std::cout << "Points in new img:	" << matchedPointsNew_l.size() << std::endl;
			
			// Here comes the Odometry part
			// First try to triangulate the points, then get transform between camera poses and publish it
			cv::Mat worldPoints, rVec, tVec;
			const cv::Mat projectionMatrix(stereoSub.stCamModel_.left().projectionMatrix());
			try
				{
				cv::triangulatePoints(projectionMatrix, stereoSub.stCamModel_.right().projectionMatrix(), twiceMatchedPoints_l, twiceMatchedPoints_r, worldPoints);
					
				cv::solvePnPRansac(worldPoints.rowRange(0, 3).t(), matchedPointsNew_l, projectionMatrix.colRange(0, 3), cv::noArray(), rVec, tVec, false, 500, 2.0);
				
				cv::Mat R;
				cv::Rodrigues(rVec, R);
				tf::Matrix3x3 rRos(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
									R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
									R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
				tf::Transform tfNewOld(rRos, tf::Vector3(tVec.at<double>(0), tVec.at<double>(1), tVec.at<double>(2)));
				tfVehiOdom *= tfNewOld;
				}
			catch(cv::Exception& e)
				{
				std::cout << e.what() << std::endl;
				}
			catch(...)
				{
				std::cout << "Something went wrong in the odomodong" << std::endl;
				}
			
			
			// Draw Matches
			cv::Mat imgMatches;
			cv::drawMatches(left, keyPoints_l, right, keyPoints_r, refinedMatches, imgMatches);
			// Draw RANSAC proof KeyPoints in red
			cv::drawKeypoints(left, ransacProofKPs_l, left, cv::Scalar(0, 0, 200));
			
			geometry_msgs::TransformStamped tfMessage;
			tfMessage.header.stamp = ros::Time::now();
			tfMessage.header.frame_id = "odom";
			tfMessage.child_frame_id = "base_link";
			tfMessage.transform.translation.x = tfVehiOdom.getOrigin().getX()/1000;
			tfMessage.transform.translation.y = tfVehiOdom.getOrigin().getY()/1000;
			tfMessage.transform.translation.z = tfVehiOdom.getOrigin().getZ()/1000;
			tfMessage.transform.rotation.x = tfVehiOdom.getRotation().getX();
			tfMessage.transform.rotation.y = tfVehiOdom.getRotation().getY();
			tfMessage.transform.rotation.z = tfVehiOdom.getRotation().getZ();
			tfMessage.transform.rotation.w = tfVehiOdom.getRotation().getW();
			tfBr.sendTransform(tfMessage);
			stereoSub.publishCVImages(left, imgMatches);
			}
		
		tickTime = ((double)cv::getTickCount() - tickTime) / cv::getTickFrequency();
		std::cout << "Time used:		" << tickTime*1000 << "ms" << std::endl << std::endl;
		
		loopRate.sleep();
		}
	
	ros::spin();
	}
