#include <stdexcept>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>


namespace sp_navigation
{
// Converts openCV rVec and tVec to a ROS tf::Transform. Note that rVec, tVec must be double format.
tf::Transform transfromFromRTVecs(const cv::Mat& rVec, const cv::Mat& tVec)
	{
	tf::Vector3 rVector(rVec.ptr<double>(0)[0], rVec.ptr<double>(1)[0], rVec.ptr<double>(2)[0]);
	tf::Vector3 tVector(tVec.ptr<double>(0)[0], tVec.ptr<double>(1)[0], tVec.ptr<double>(2)[0]);
	tf::Quaternion q = (rVector.length() == 0) ? tf::Quaternion::getIdentity() : tf::Quaternion(rVector, rVector.length());
	return tf::Transform(q, tVector);
	}

// Represents a viewpoint, e.g. a position of a stereo camera
struct Node
	{
	static cv::ORB oFDDE;	/**< ORB feature detector and descriptor extractor.*/
	static cv::BFMatcher bfMatcher; /**< Bruteforce matcher with cross-checking of matches.*/
	static std::vector< std::vector<cv::Point3f> > map; /**< Map consisting of Points, each with 3D information of each viewpoint */

	ros::Time timestamp_; /**< Time of creation of the node. */	
	cv::Mat projMat_l_, projMat_r_; /**< Projection Matrix of the left and right camera. */
	std::vector<cv::KeyPoint> stereoKPs_l_, stereoKPs_r_; /**< Stereo matched Keypoints. */
	std::vector<cv::Point2f> stereoPoints_l_, stereoPoints_r_; /**< Stereo matched Points. */
	cv::Mat desc_l_, desc_r_; /**< Descriptors of stereo keypoints. */
	std::vector<cv::Point3f> nodePoints_; /**< 3D Points in camera/node frame. */
	cv::Mat rVecRel_, tVecRel_; /**< R, T to previous node. */
	cv::Mat rVecAbs_, tVecAbs_; /**< R, T to world frame. */
	std::vector<unsigned int> mapIdxs_; /**< Map Index. Stores at which index the KeyPoint/3D Point was put in the map. */
	
	/* Default Constructor. Only timestamp initialized.
	 *
	 * */
	Node() : timestamp_(ros::Time::now()) {}
	
	/* Constructor that creates an empty node with timestamp and projection matrices.
	 *
	 * */
	Node(cv::Mat& projMat_l, cv::Mat& projMat_r) :
		timestamp_(ros::Time::now()),
		projMat_l_(projMat_l),
		projMat_r_(projMat_r)
		{}
	
	/* Constructor that already adds stereo matched points and triangulated 3D points.
	 *
	 * */
	Node(const cv::Mat& projMat_l, const cv::Mat& projMat_r, const cv::Mat& img_l, const cv::Mat& img_r) :
		timestamp_(ros::Time::now()),
		projMat_l_(projMat_l),
		projMat_r_(projMat_r)
		{
		pointsFromImages(img_l, img_r);
		}
	
	/* Constructor that fills all information by an image pair and a previous node.
	 * The previous node needs to have R, T and 3D points filled.
	 *
	 * */
	Node(const cv::Mat& projMat_l, const cv::Mat& projMat_r, const cv::Mat& img_l, const cv::Mat& img_r, const Node& previousNode, bool mapFromBadPose = false) :
		timestamp_(ros::Time::now()),
		projMat_l_(projMat_l),
		projMat_r_(projMat_r)
		{
		pointsFromImages(img_l, img_r);
		putIntoWorld(previousNode, mapFromBadPose);
		}
	
	/* Generates the first Node from an image pair.
	 *
	 * */
	static Node firstNode(const cv::Mat& projMat_l, const cv::Mat& projMat_r, const cv::Mat& img_l, const cv::Mat& img_r)
		{
		// Create Node that already has all coordinate data filled
		Node n(projMat_l, projMat_r, img_l, img_r);
		// It's the first node, it practically defines the world frame
		n.rVecRel_ = cv::Mat::zeros(3, 1, CV_64F);
		n.tVecRel_ = cv::Mat::zeros(3, 1, CV_64F);
		n.rVecAbs_ = cv::Mat::zeros(3, 1, CV_64F);
		n.tVecAbs_ = cv::Mat::zeros(3, 1, CV_64F);
		// Put all found 3D points into the map and save where they have been put (no need to transform them)
		for (unsigned int i = 0; i < n.nodePoints_.size(); ++i)
			{
			map.push_back(std::vector<cv::Point3f>());
			map[i].push_back(n.nodePoints_[i]);
			n.mapIdxs_.push_back(i);
			}
		
		return n;
		}
	
	/* Matches left and right image, filters bad keypoints, stores the good ones and
	 * calculates 3D Points from them.
	 *
	 * */
	void pointsFromImages(const cv::Mat& img_l, const cv::Mat& img_r)
		{
		// In case this method gets called more than once, clear data first
		stereoKPs_l_.clear();
		stereoKPs_r_.clear();
		stereoPoints_l_.clear();
		stereoPoints_r_.clear();
		desc_l_ = cv::Mat();
		desc_r_ = cv::Mat();
		nodePoints_.clear();
		
		// Detect features in left and right image
		std::vector<cv::KeyPoint> keyPoints_l, keyPoints_r;
		cv::Mat descriptors_l, descriptors_r;
		oFDDE(img_l, cv::noArray(), keyPoints_l, descriptors_l);
		oFDDE(img_r, cv::noArray(), keyPoints_r, descriptors_r);
		
		// Match features with a descriptor(!)-distance below a threshold
		std::vector< std::vector<cv::DMatch> > matches;
		bfMatcher.radiusMatch(descriptors_l, descriptors_r, matches, 35);
		
		// Only use matches that fulfill the epipolar constraint, thus lying on a horizontal line
		std::vector<cv::KeyPoint> refinedKPs_l, refinedKPs_r;
		std::vector<cv::Point2f> refinedPoints_l, refinedPoints_r;
		cv::Mat refinedDesc_l, refinedDesc_r;
		for(unsigned int i = 0; i < matches.size(); ++i)
			{
			if (matches[i].size() > 0 && fabs(keyPoints_l[matches[i][0].queryIdx].pt.y - keyPoints_r[matches[i][0].trainIdx].pt.y) <= 2)
				{
				refinedKPs_l.push_back(keyPoints_l[matches[i][0].queryIdx]);
				refinedKPs_r.push_back(keyPoints_r[matches[i][0].trainIdx]);
			
				refinedPoints_l.push_back(keyPoints_l[matches[i][0].queryIdx].pt);
				refinedPoints_r.push_back(keyPoints_r[matches[i][0].trainIdx].pt);
				
				refinedDesc_l.push_back(descriptors_l.row(matches[i][0].queryIdx));
				refinedDesc_r.push_back(descriptors_r.row(matches[i][0].trainIdx));
				}
			}
				
		// Remove outliers by RANSAC
		std::vector<unsigned char> inlierStatus;
		cv::findFundamentalMat(refinedPoints_l, refinedPoints_r, CV_FM_RANSAC, 3., 0.99, inlierStatus);
		for(unsigned int i = 0; i < inlierStatus.size(); ++i)
			{
			if(inlierStatus[i])
				{
				stereoKPs_l_.push_back(refinedKPs_l[i]);
				stereoKPs_r_.push_back(refinedKPs_r[i]);
				
				stereoPoints_l_.push_back(refinedPoints_l[i]);
				stereoPoints_r_.push_back(refinedPoints_r[i]);
				
				desc_l_.push_back(refinedDesc_l.row(i));
				desc_r_.push_back(refinedDesc_r.row(i));
				}
			}
		
		// Calculate 3D points
		cv::Mat triangulatedPointMat;
		cv::triangulatePoints(projMat_l_, projMat_r_, stereoPoints_l_, stereoPoints_r_, triangulatedPointMat);
		cv::convertPointsFromHomogeneous(triangulatedPointMat.t(), nodePoints_);
		}
		
	/* Calculates the pose of the node and, if successful, transforms 3D points into world frame and adds them
	 * to the map.
	 *
	 * */
	bool putIntoWorld(const Node& previousNode, bool useBadPose = false)
		{
		// Match features with a descriptor(!)-distance below a threshold
		std::vector< std::vector<cv::DMatch> > radiusMatches;
		bfMatcher.radiusMatch(desc_l_, previousNode.desc_l_, radiusMatches, 35);
		
		// Create Vectors containing the matched points
		std::vector<cv::Point2f> matchedPointsCurr, matchedPointsPrev;
		std::vector<cv::Point3f> matchedNodePointsPrev;
		std::vector<cv::DMatch> matches;
		for(unsigned int i = 0; i < radiusMatches.size(); ++i)
			{
			if (radiusMatches[i].size() > 0)
				{
				matchedPointsCurr.push_back(stereoPoints_l_[radiusMatches[i][0].queryIdx]);
				
				matchedPointsPrev.push_back(previousNode.stereoPoints_l_[radiusMatches[i][0].trainIdx]);
				matchedNodePointsPrev.push_back(previousNode.nodePoints_[radiusMatches[i][0].trainIdx]);
				
				matches.push_back(radiusMatches[i][0]);
				}
			}
				
		// Remove outliers by RANSAC
		std::vector<unsigned char> inlierStatus;
		cv::findFundamentalMat(matchedPointsCurr, matchedPointsPrev, CV_FM_RANSAC, 3., 0.99, inlierStatus);
		std::vector<cv::Point2f> filteredMatchedPointsCurr;
		std::vector<cv::Point3f> filteredMatchedNodePointsPrev;
		std::vector<cv::DMatch> filteredMatches;
		for(unsigned int i = 0; i < inlierStatus.size(); ++i)
			{
			if(inlierStatus[i])
				{
				filteredMatchedPointsCurr.push_back(matchedPointsCurr[i]);
				filteredMatchedNodePointsPrev.push_back(matchedNodePointsPrev[i]);
				filteredMatches.push_back(matches[i]);
				}
			}
		
		// If there are less than 5 points left return false
		if (filteredMatchedPointsCurr.size() < 5) return false;
		
		// Get pose between previous and current node (tf previous->current)
		cv::solvePnPRansac(filteredMatchedNodePointsPrev, filteredMatchedPointsCurr, projMat_l_.colRange(0,3), cv::noArray(), rVecRel_, tVecRel_, false, 500, 2.0);
		
		// Calculate absolute pose (tf current->world/first)
		cv::composeRT(-rVecRel_, -tVecRel_, previousNode.rVecAbs_, previousNode.tVecAbs_, rVecAbs_, tVecAbs_);
		
		// Transform 3D points of this node into the world frame and add them to the map
		cv::Mat R, RT = cv::Mat::eye(4, 4, CV_64F);
		cv::Rodrigues(rVecAbs_, R);
		R.copyTo(RT.colRange(0, 3).rowRange(0, 3));
		tVecAbs_.copyTo(RT.rowRange(0, 3).col(3));
		std::vector<cv::Point3f> worldPoints;
		cv::perspectiveTransform(nodePoints_, worldPoints, RT);
		for (unsigned int i = 0; i < worldPoints.size(); ++i)
			{
			// Check if point was already found by the last node
			bool foundMatch = false;
			unsigned int mapIdx;
			for (unsigned int j = 0; j < filteredMatches.size(); ++j)
				{
				if (filteredMatches[j].queryIdx == i)
					{
					mapIdx = previousNode.mapIdxs_[filteredMatches[j].trainIdx];
					foundMatch = true;
					break;
					}
				}
			if (!foundMatch)
				{
				map.push_back(std::vector<cv::Point3f>());
				mapIdx = map.size() - 1;
				}
			map[mapIdx].push_back(worldPoints[i]);
			mapIdxs_.push_back(mapIdx);
			}
		}
	
	};

cv::ORB Node::oFDDE(3000);
cv::BFMatcher Node::bfMatcher(cv::NORM_HAMMING, true); // Only cross-checked matches will be used

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
		std::vector<cv::KeyPoint> keyPointsPrev_l_, keyPointsPrev_r_;
		std::vector<cv::Point2f> pointsPrev_l_, pointsPrev_r_;
		std::vector<cv::Point2f> pointsCurr_l_;
		std::vector<cv::Point3f> triangulatedPoints;
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
			std::vector< std::vector<cv::DMatch> > matches;
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
				cv::Mat triangulatedPointMat, rVec, tVec;
				cv::Mat projMat_l = cv::Mat(stereoSub_->stCamModel_.left().projectionMatrix());
				cv::Mat projMat_r = cv::Mat(stereoSub_->stCamModel_.right().projectionMatrix());
				cv::triangulatePoints(projMat_l, projMat_r, pointsPrev_l_, pointsPrev_r_, triangulatedPointMat);
				cv::convertPointsFromHomogeneous(triangulatedPointMat.t(), triangulatedPoints);
				
				// Get difference between current and last camera pose
				cv::solvePnPRansac(triangulatedPoints, pointsCurr_l_, projMat_l.colRange(0,3), cv::noArray(), rVec, tVec, false, 500, 2.0);
				
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
