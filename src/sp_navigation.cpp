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
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/LinearMath/Quaternion.h>
#include <image_geometry/stereo_camera_model.h>
#include <stdexcept>

namespace sp_navigation
{

// Reconstructs a 3D point by Linear LS triangulation
cv::Point3f triangulatePoint(cv::Point2f u, cv::Matx34d P, cv::Point2f u1, cv::Matx34d P1) 
	{
	cv::Matx43d A(u.x*P(2,0)-P(0,0),	u.x*P(2,1)-P(0,1),		u.x*P(2,2)-P(0,2),		
			  u.y*P(2,0)-P(1,0),	u.y*P(2,1)-P(1,1),		u.y*P(2,2)-P(1,2),		
			  u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),	u1.x*P1(2,2)-P1(0,2),	
			  u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),	u1.y*P1(2,2)-P1(1,2)
			  );
	cv::Matx41d B(-(u.x*P(2,3)	-P(0,3)),
			  -(u.y*P(2,3)	-P(1,3)),
			  -(u1.x*P1(2,3)	-P1(0,3)),
			  -(u1.y*P1(2,3)	-P1(1,3)));
	
	cv::Mat_<double> X;
	cv::solve(A, B, X, cv::DECOMP_SVD);

	cv::Point3f x(X.ptr<double>(0)[0], X.ptr<double>(1)[0], X.ptr<double>(2)[0]);
	
	return x;
	}

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
		unsigned int imgIdx;


		/* Constructor
		 *
		 * */
		StereoSubscriber(ros::NodeHandle& nh, const std::string nameToResolve) :
			it_(nh),
			sync_(3),
			imgIdx(0)
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


		/* Simple method to publish cv::Mat images
		 *
		 * */
		void publishCVImages(const cv::Mat& lImage, const cv::Mat& rImage)
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
	};

// Class for computing odometry from stereo images
class VisualOdometer
	{
	private:
		cv::ORB oFDDE_;	// ORB feature detector and descriptor extractor
		cv::BFMatcher bfMatcher_; // Bruteforce matcher with cross-checking of matches
		StereoSubscriber* stereoSub_ = NULL;
		std::string parentFrame_, childFrame_;
		unsigned int lastImgIdx_;
		tf::TransformListener tfListener_;
	
	public:
		cv::Mat imgPrev_l_, imgPrev_r_;
		cv::Mat imgCurr_l_, imgCurr_r_;
		// Keypoints/points in 'previous' images
		std::vector<cv::KeyPoint> keyPointsPrev_l_, keyPointsPrev_r_;
		std::vector<cv::Point2f> pointsPrev_l_, pointsPrev_r_;
		// Points in 'current' images
		std::vector<cv::Point2f> pointsCurr_l_;
		
		std::vector<cv::Point3f> worldPoints;
		tf::Transform tfBLOdom;
		tf::Transform tfCamPreCamCur;
		
		tf::TransformBroadcaster tfBr_	;
		
		/* Constructor
		 *
		 * */
		VisualOdometer(StereoSubscriber& stereoSub, std::string parentFrame, std::string childFrame) :
			parentFrame_(parentFrame),
			childFrame_(childFrame),
			lastImgIdx_(0),
			oFDDE_(3000),
			bfMatcher_(cv::NORM_HAMMING, true)
			{
			stereoSub_ = &stereoSub;
			tfBLOdom.setIdentity();
			tfCamPreCamCur.setIdentity();
			}
		
		/* Fetches an stereo image pair from a sp_navigation::StereoSubscriber
		 * returns true if a new pair of pictures was received and successfully updated
		 * */
		bool getImagePair()
			{
			if (!stereoSub_) throw std::runtime_error("Error in VisualOdometer::getImagePair: stereoSub points to no object!");
			if (stereoSub_->imgConstPtr_l_ != NULL && stereoSub_->imgConstPtr_r_ != NULL)
				{
				if (imgCurr_l_.empty() || imgCurr_r_.empty())
					{
					stereoSub_->imgConstPtr_l_->image.copyTo(imgCurr_l_);
					stereoSub_->imgConstPtr_r_->image.copyTo(imgCurr_r_);
					lastImgIdx_ = stereoSub_->imgIdx;
					return false;
					}
				else if (stereoSub_->imgIdx != lastImgIdx_)
					{
					imgCurr_l_.copyTo(imgPrev_l_);
					imgCurr_r_.copyTo(imgPrev_r_);
					stereoSub_->imgConstPtr_l_->image.copyTo(imgCurr_l_);
					stereoSub_->imgConstPtr_r_->image.copyTo(imgCurr_r_);
					lastImgIdx_ = stereoSub_->imgIdx;
					return true;
					}
				}
			else std::cout << "No images received!" << std::endl;
			return false;
			}
		
		/* Finds matches in the previous pair of stereo images
		 *
		 * */
		bool findMatches()
			{
			// Check if there is a pair of images to find matches
			if (imgPrev_l_.empty() || imgPrev_r_.empty() || imgCurr_l_.empty()) throw std::runtime_error("Error in VisualOdometer::updateStereoMatches(): No images to match!");
			
			// Clear previous found features
			keyPointsPrev_l_.clear();
			keyPointsPrev_r_.clear();
			pointsPrev_l_.clear();
			pointsPrev_r_.clear();
			pointsCurr_l_.clear();
			
			// Detect features in left and right image of the previous pair
			std::vector<cv::KeyPoint> keyPoints_l, keyPoints_r;
			cv::Mat descriptors_l, descriptors_r;
			oFDDE_(imgPrev_l_, cv::noArray(), keyPoints_l, descriptors_l);
			oFDDE_(imgPrev_r_, cv::noArray(), keyPoints_r, descriptors_r);
			
			// Match features with a descriptor(!)-distance below a threshold
			std::vector<std::vector<cv::DMatch> > matches;
			bfMatcher_.radiusMatch(descriptors_l, descriptors_r, matches, 35);
			
			// Only use matches that fulfill the epipolar constraint, thus lying on a horizontal line
			std::vector<cv::KeyPoint> refinedKPs_l, refinedKPs_r;
			std::vector<cv::Point2f> refinedPoints_l, refinedPoints_r;
			for(unsigned int i = 0; i < matches.size(); ++i)
				{
				if (matches[i].size() > 0 && fabs(keyPoints_l[matches[i][0].queryIdx].pt.y - keyPoints_r[matches[i][0].trainIdx].pt.y) <= 2)
					{
					refinedKPs_l.push_back(keyPoints_l[matches[i][0].queryIdx]);
					refinedKPs_r.push_back(keyPoints_r[matches[i][0].trainIdx]);
				
					refinedPoints_l.push_back(keyPoints_l[matches[i][0].queryIdx].pt);
					refinedPoints_r.push_back(keyPoints_r[matches[i][0].trainIdx].pt);
					}
				}
					
			// Remove outliers by RANSAC
			std::vector<unsigned char> inlierStatus;
			cv::findFundamentalMat(refinedPoints_l, refinedPoints_r, CV_FM_RANSAC, 3., 0.99, inlierStatus);
			std::vector<cv::KeyPoint> stereoKPs_l, stereoKPs_r;
			std::vector<cv::Point2f> stereoPoints_l, stereoPoints_r;
			for(unsigned int i = 0; i < inlierStatus.size(); ++i)
				{
				if(inlierStatus[i])
					{
					stereoKPs_l.push_back(refinedKPs_l[i]);
					stereoKPs_r.push_back(refinedKPs_r[i]);
					
					stereoPoints_l.push_back(refinedPoints_l[i]);
					stereoPoints_r.push_back(refinedPoints_r[i]);
					}
				}
			
			// If there are no matches left, return false
			if (stereoPoints_l.empty()) return false;
			
			// Try to find found features in the next image
			std::vector<cv::Point2f> matchedPointsCurrentAll_l;
			std::vector<unsigned char> opticalFlowStatus;
			std::vector<float> opticalFlowError;
			cv::calcOpticalFlowPyrLK(imgPrev_l_, imgCurr_l_, stereoPoints_l, matchedPointsCurrentAll_l, opticalFlowStatus, opticalFlowError);
			
			for(unsigned int i = 0; i < opticalFlowStatus.size(); i++)
				{
				if(opticalFlowStatus[i])
					{
					keyPointsPrev_l_.push_back(stereoKPs_l[i]);
					keyPointsPrev_r_.push_back(stereoKPs_r[i]);
					
					pointsPrev_l_.push_back(stereoPoints_l[i]);
					pointsPrev_r_.push_back(stereoPoints_r[i]);
					
					pointsCurr_l_.push_back(matchedPointsCurrentAll_l[i]);
					}
				}
			
			if (pointsCurr_l_.size() > 5) return true;
			else return false;
			}
			
		/* Calculates pose between current and last camera pose after getting new images and matching features
		 *
		 * */
		void updateRT()
			{
			// If images successfully updated and features matched, update tfVehiOdom with new tf, otherwise use last transform
			if (getImagePair() && findMatches())
				{
				worldPoints.clear();
				for (unsigned int i = 0; i < pointsPrev_l_.size(); ++i)
					{
					cv::Point3f worldPoint = triangulatePoint(pointsPrev_l_[i], stereoSub_->stCamModel_.left().projectionMatrix(), pointsPrev_r_[i], stereoSub_->stCamModel_.right().projectionMatrix());
					worldPoints.push_back(worldPoint);
					}
				
				cv::Mat rVec, tVec;
				cv::solvePnPRansac(worldPoints, pointsCurr_l_, stereoSub_->stCamModel_.left().intrinsicMatrix(), cv::noArray(), rVec, tVec, false, 500, 2.0);
				
//				rVec.ptr<double>(0)[0] = 0;
//				rVec.ptr<double>(1)[0] = 0;
//				rVec.ptr<double>(2)[0] = 0;
//				tVec.ptr<double>(0)[0] = 0;
//				tVec.ptr<double>(1)[0] = 0;
//				tVec.ptr<double>(2)[0] = 0;
				std::cout << "Ischts double? 6 = " << tVec.type() << ", " << rVec.type() << std::endl;
				tf::Vector3 rVector(rVec.ptr<double>(0)[0], rVec.ptr<double>(1)[0], rVec.ptr<double>(2)[0]);
				tf::Vector3 tVector(tVec.ptr<double>(0)[0], tVec.ptr<double>(1)[0], tVec.ptr<double>(2)[0]);
				tf::Quaternion q = (rVector.length() == 0) ? tf::Quaternion::getIdentity() : tf::Quaternion(rVector, rVector.length());
				tfCamPreCamCur = tf::Transform(q, tVector);
				}
			tf::StampedTransform tfCamBL;
			tfListener_.lookupTransform("base_link", "VRMAGIC", ros::Time(0), tfCamBL);
			tf::Transform tfBLCurBLPre = tfCamBL * (tfCamPreCamCur.inverse() * tfCamBL.inverse());
			tfBLOdom *= tfBLCurBLPre;
			}
		
		/* Publishes the last found rotation and translation
		 *
		 * */
		void publishTF()
			{
/*			tf::Vector3 rVector(rVecBO.ptr<double>(0)[0], rVecBO.ptr<double>(1)[0], rVecBO.ptr<double>(2)[0]);
			tf::Quaternion q = (rVector.length() == 0) ? tf::Quaternion::getIdentity() : tf::Quaternion(rVector, rVector.length());
			tf::Transform tfBO_c(q, rVector);

			tf::StampedTransform tfVrBl;
			tfListener_.lookupTransform("base_link", "VRMAGIC", ros::Time(0), tfVrBl);
			
			tf::Transform tfBO_b = tfBO_c * tfVrBl;
			
			geometry_msgs::PoseStamped cameraPose;
			cameraPose.header.frame_id = "VRMAGIC";
			cameraPose.pose.position.x = tVecBO.ptr<double>(0)[0];
			cameraPose.pose.position.y = tVecBO.ptr<double>(1)[0];
			cameraPose.pose.position.z = tVecBO.ptr<double>(2)[0];
			cameraPose.pose.orientation.x = q.x();
			cameraPose.pose.orientation.y = q.y();
			cameraPose.pose.orientation.z = q.z();
			cameraPose.pose.orientation.w = q.w();
			
			//geometry_msgs::TransformStamped tfCamBase = tfBuffer_.lookupTransform("VRMAGIC", "base_link	", ros::Time(0));
			tfListener_.transformPose("base_link", cameraPose, cameraPose);
			
			geometry_msgs::TransformStamped tfMessage;
			tfMessage.header.stamp = ros::Time::now();
			tfMessage.header.frame_id = parentFrame_;
			tfMessage.child_frame_id = childFrame_;
			
			tfMessage.transform.translation.x = 0;//cameraPose.pose.position.x;
			tfMessage.transform.translation.y = 0;//cameraPose.pose.position.y;
			tfMessage.transform.translation.z = 0;//cameraPose.pose.position.z;
			
			tfMessage.transform.rotation.x = 1;//cameraPose.pose.orientation.x;
			tfMessage.transform.rotation.y = 1;//cameraPose.pose.orientation.y;
			tfMessage.transform.rotation.z = 1;//cameraPose.pose.orientation.z;
			tfMessage.transform.rotation.w = 1;//cameraPose.pose.orientation.w;
*/			
			tfBr_.sendTransform(tf::StampedTransform(tfBLOdom, ros::Time::now(), "odom", "base_link"));
			}
	};

} //End of namespace


int main(int argc, char** argv)
	{
	ros::init(argc, argv, "sp_navigation");
	ros::NodeHandle nh;
	sp_navigation::StereoSubscriber stereoSub(nh, "stereo");
	sp_navigation::VisualOdometer visualOdo(stereoSub, std::string("odom"), std::string("base_link"));
	
	ros::Publisher posePublisher = nh.advertise<geometry_msgs::PoseStamped>("sp_navigation/Pose", 500);
	
	ros::Rate loopRate(12);
	while (ros::ok())
		{
		// Take time of each loop
		double tickTime = (double)cv::getTickCount();
		ros::spinOnce();
		try
			{
			visualOdo.updateRT();
			std::cout << "P links  : " << visualOdo.pointsPrev_l_.size() << std::endl;
			std::cout << "P rechts : " << visualOdo.pointsPrev_r_.size() << std::endl;
			std::cout << "P curr_l : " << visualOdo.pointsCurr_l_.size() << std::endl;
			visualOdo.publishTF();
			
/*			tf::Vector3 rVector(visualOdo.rVecBO.ptr<double>(0)[0], visualOdo.rVecBO.ptr<double>(1)[0], visualOdo.rVecBO.ptr<double>(2)[0]);
			tf::Quaternion q = (rVector.length() == 0) ? tf::Quaternion::getIdentity() : tf::Quaternion(rVector, rVector.length());
			geometry_msgs::PoseStamped cameraPose;
			cameraPose.header.frame_id = "VRMAGIC";
			cameraPose.header.stamp = ros::Time::now();
			cameraPose.pose.position.x = visualOdo.tVecBO.ptr<double>(0)[0];
			cameraPose.pose.position.y = visualOdo.tVecBO.ptr<double>(1)[0];
			cameraPose.pose.position.z = visualOdo.tVecBO.ptr<double>(2)[0];
			cameraPose.pose.orientation.x = q.x();
			cameraPose.pose.orientation.y = q.y();
			cameraPose.pose.orientation.z = q.z();
			cameraPose.pose.orientation.w = q.w();
			posePublisher.publish(cameraPose);
/*			
			for (unsigned int i = 0; i < visualOdo.stereoPoints_l.size(); ++i)
				{
				cv::circle(visualOdo.imgPrev_l_, visualOdo.stereoPoints_l[i], 5, cv::Scalar(200,0,0));
				cv::circle(visualOdo.imgPrev_r_, visualOdo.stereoPoints_r[i], 5, cv::Scalar(200,0,0));
				}
			
			cv::drawKeypoints(visualOdo.imgPrev_l_, visualOdo.stereoKPs_l, visualOdo.imgPrev_l_, cv::Scalar(0,0,200));
			cv::drawKeypoints(visualOdo.imgPrev_r_, visualOdo.stereoKPs_r, visualOdo.imgPrev_r_, cv::Scalar(0,0,200));
*/			
			cv::Mat pub_l, pub_r;
			visualOdo.imgPrev_l_.copyTo(pub_l);
			visualOdo.imgCurr_l_.copyTo(pub_r);
			for (unsigned int i = 0; i < visualOdo.pointsPrev_l_.size(); ++i)
				{
				cv::circle(pub_l, visualOdo.pointsPrev_l_[i], 5, cv::Scalar(0,200,0));
				cv::circle(pub_r, visualOdo.pointsCurr_l_[i], 5, cv::Scalar(0,200,0));
				}
			
			
			stereoSub.publishCVImages(pub_l, pub_r);
			}
		catch(std::exception& e)
			{
			std::cout << e.what() << std::endl;
			}
		catch(...)
			{
			std::cout << "Something unknown went wrong." << std::endl;
			}
		
		tickTime = ((double)cv::getTickCount() - tickTime) / cv::getTickFrequency();
		std::cout << "Time used:		" << tickTime*1000 << "ms" << std::endl << std::endl;
		
		loopRate.sleep();
		}
	
	ros::spin();
	}
