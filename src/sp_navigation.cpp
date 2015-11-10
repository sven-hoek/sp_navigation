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
		std::string parentFrame_, childFrame_, cameraFrame_;
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
		tf::TransformBroadcaster tfBr_;
		ros::Publisher posePub_;
		
		/* Constructor
		 *
		 * */
		VisualOdometer(StereoSubscriber& stereoSub, std::string parentFrame, std::string childFrame, std::string cameraFrame) :
			parentFrame_(parentFrame),
			childFrame_(childFrame),
			cameraFrame_(cameraFrame),
			lastImgIdx_(0),
			oFDDE_(3000),
			bfMatcher_(cv::NORM_HAMMING, true)
			{
			stereoSub_ = &stereoSub;
			tfBLOdom.setIdentity();
			tfCamPreCamCur.setIdentity();
			ros::NodeHandle nh;
			posePub_ = nh.advertise<geometry_msgs::PoseStamped>("sp_navigation/Pose", 50);
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
				// Triangulate stereo matches into 3D
/*				worldPoints.clear();
				for (unsigned int i = 0; i < pointsPrev_l_.size(); ++i)
					{
					cv::Point3f worldPoint = triangulatePoint(pointsPrev_l_[i], stereoSub_->stCamModel_.left().projectionMatrix(), pointsPrev_r_[i], stereoSub_->stCamModel_.right().projectionMatrix());
					worldPoints.push_back(worldPoint);
					}
*/
				cv::Mat worldPointMat, rVec, tVec;
				cv::Mat projMat_l = cv::Mat(stereoSub_->stCamModel_.left().projectionMatrix());
				cv::Mat projMat_r = cv::Mat(stereoSub_->stCamModel_.right().projectionMatrix());
				cv::triangulatePoints(projMat_l, projMat_r, pointsPrev_l_, pointsPrev_r_, worldPointMat);
				cv::convertPointsFromHomogeneous(worldPointMat.t(), worldPoints);
				
				// Get difference between current and last camera pose
				cv::solvePnPRansac(worldPoints, pointsCurr_l_, projMat_l.colRange(0,3), cv::noArray(), rVec, tVec, false, 500, 2.0);
				
				std::cout << "tVec: " << tVec << std::endl;
				
				// Use only values above/below a threshold and filter movements that are not possible
//				rVec.ptr<double>(0)[0] = 0;
//				rVec.ptr<double>(1)[0] = 0;
//				rVec.ptr<double>(2)[0] = 0;
//				tVec.ptr<double>(0)[0] = 0;
//				tVec.ptr<double>(1)[0] = 0;
//				tVec.ptr<double>(2)[0] = 0;

/*				for (unsigned int i=0; i<3; ++i)
					{
					if(std::fabs(rVec.at<double>(i)) < 0.0035 || std::fabs(rVec.at<double>(i)) > 1.5)
						{
						rVec.at<double>(i)=0;
						}
					if(std::fabs(tVec.at<double>(i)) < 0.005 || fabs(tVec.at<double>(i)) > 0.6)
						{
						tVec.at<double>(i)=0;
						}
					}
*/				
				// Create a tf::Transform object from rVec and tvec
				tf::Vector3 rVector(rVec.ptr<double>(0)[0], rVec.ptr<double>(1)[0], rVec.ptr<double>(2)[0]);
				tf::Vector3 tVector(tVec.ptr<double>(0)[0], tVec.ptr<double>(1)[0], tVec.ptr<double>(2)[0]);
				tf::Quaternion q = (rVector.length() == 0) ? tf::Quaternion::getIdentity() : tf::Quaternion(rVector, rVector.length());
				tfCamPreCamCur = tf::Transform(q, tVector);
				}
			// Receive transform from camera to base_link and update position of base_link
			tf::StampedTransform tfCamBL;
			tfListener_.lookupTransform(childFrame_, cameraFrame_, ros::Time(0), tfCamBL);
			tf::Transform tfBLCurBLPre = tfCamBL * (tfCamPreCamCur.inverse() * tfCamBL.inverse());
			tfBLOdom *= tfBLCurBLPre;
			}
		
		/* Publishes the TF information which is stored in the object at that moment
		 *
		 * */
		void publishTF()
			{
			tfBr_.sendTransform(tf::StampedTransform(tfBLOdom, ros::Time::now(), parentFrame_, childFrame_));
			}
		
		/* Publishes the TF information as pose message
		 *
		 * */
		void publishPose()
			{
			geometry_msgs::PoseStamped poseMSG;
			poseMSG.header.stamp = ros::Time::now();
			poseMSG.header.frame_id = parentFrame_;
			poseMSG.pose.position.x = tfBLOdom.getOrigin().getX();
			poseMSG.pose.position.y = tfBLOdom.getOrigin().getY();
			poseMSG.pose.position.z = tfBLOdom.getOrigin().getZ();
			poseMSG.pose.orientation.w = tfBLOdom.getRotation().getW();
			poseMSG.pose.orientation.x = tfBLOdom.getRotation().getX();
			poseMSG.pose.orientation.y = tfBLOdom.getRotation().getY();
			poseMSG.pose.orientation.z = tfBLOdom.getRotation().getZ();
			
			posePub_.publish(poseMSG);
			}
	};

} //End of namespace


int main(int argc, char** argv)
	{
	ros::init(argc, argv, "sp_navigation");
	ros::NodeHandle nh;
	sp_navigation::StereoSubscriber stereoSub(nh, "stereo");
	sp_navigation::VisualOdometer visualOdo(stereoSub, std::string("odom"), std::string("base_link"), std::string("VRMAGIC"));
	
	ros::Rate loopRate(10);
	while (ros::ok())
		{
		// Take time of each loop
		double tickTime = (double)cv::getTickCount();
		ros::spinOnce();
		try
			{
			visualOdo.updateRT();
			visualOdo.publishTF();
			visualOdo.publishPose();
			tickTime = ((double)cv::getTickCount() - tickTime) / cv::getTickFrequency();
			
			// Visualize matches and publish them
			cv::Mat pub_l, pub_r;
			visualOdo.imgPrev_l_.copyTo(pub_l);
			visualOdo.imgPrev_l_.copyTo(pub_r);
			for (unsigned int i = 0; i < visualOdo.pointsPrev_l_.size(); ++i)
				{
				cv::circle(pub_r, visualOdo.pointsPrev_l_[i], 1, cv::Scalar(0,200,0));
				//cv::circle(pub_r, visualOdo.pointsCurr_l_[i], 5, cv::Scalar(0,200,200));
				cv::line(pub_r, visualOdo.pointsPrev_l_[i], visualOdo.pointsCurr_l_[i], cv::Scalar(0,200,0));
				}

			//draw lines for moving Keypoints that moved more than a threshold
			for(int i=0; i < visualOdo.pointsPrev_l_.size(); i++)
				{
				cv::Point p0(std::ceil(visualOdo.pointsCurr_l_[i].x), std::ceil(visualOdo.pointsCurr_l_[i].y));
				cv::Point p1(std::ceil(visualOdo.pointsPrev_l_[i].x), std::ceil(visualOdo.pointsPrev_l_[i].y));

				double res = cv::norm(p1-p0);
				if (res>5){
					cv::line(pub_l, p0, p1, CV_RGB(0,255,0), 1);
				}

			}
			stereoSub.publishCVImages(pub_l, pub_r);
			std::cout << "Found features  : " << visualOdo.pointsPrev_l_.size() << std::endl;
			}
		catch(std::exception& e)
			{
			std::cout << e.what() << std::endl;
			}
		catch(...)
			{
			std::cout << "Something unknown went wrong." << std::endl;
			}
			
		std::cout << "Time used:		" << tickTime*1000 << "ms" << std::endl << std::endl;
		loopRate.sleep();
		}
	
	ros::spin();
	}
