#include <stdexcept>
#include <memory>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
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

#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <cvsba/cvsba.h>	// External library for sparse bundle adjustment


namespace sp_navigation
{
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

/**
 * Converts openCV rVec and tVec to a ROS tf::Transform.
 * @param[in] rVec The rotation vector in rodrigues format. Must be of double type.
 * @param[in] tVec The translation vector. Must be of double type.
 * @return The resulting Transform.
 */
tf::Transform tfFromRTVecs(const cv::Mat& rVec, const cv::Mat& tVec)
	{
	tf::Vector3 rVector(rVec.ptr<double>(0)[0], rVec.ptr<double>(1)[0], rVec.ptr<double>(2)[0]);
	tf::Vector3 tVector(tVec.ptr<double>(0)[0], tVec.ptr<double>(1)[0], tVec.ptr<double>(2)[0]);
	tf::Quaternion q = (rVector.length() == 0) ? tf::Quaternion::getIdentity() : tf::Quaternion(rVector, rVector.length());
	return tf::Transform(q, tVector);
	}

/**
 * Converts a ROS StampedTransform into openCV rVec and tVec.
 * @param[in] tf The Transform to be converted.
 * @param[out] rVec The resulting rotation Vector of type double in Rodrigues-format.
 * @param[out] tVec The resulting translation Vector of type double.
 */
void rtVecsFromTF(const tf::StampedTransform tf, cv::Mat& rVec, cv::Mat& tVec)
	{
	rVec = (cv::Mat_<double>(3,1) << tf.getRotation().getAxis().getX(), tf.getRotation().getAxis().getY(), tf.getRotation().getAxis().getZ());
	rVec *= tf.getRotation().getAngle();
	tVec = (cv::Mat_<double>(3,1) << tf.getOrigin().getX(), tf.getOrigin().getY(), tf.getOrigin().getZ());
	}

/**
 * Node class to represent viewpoints.
 * Each node represents a stereo camera at a certain position. It uses the right camera only for triangulating feature points
 * and stores 2D features and their corresponding 3D points and puts them in a map.
 */
struct Node
	{
	static cv::Ptr<cv::FeatureDetector> fDetector; /**< Feature detector interface.*/
	static cv::Ptr<cv::DescriptorExtractor> dExtractor; /**< Descriptor extractor interface.*/
	static cv::Ptr<cv::DescriptorMatcher> dMatcher; /**< Descriptor matcher interface.*/
	
	std::vector<cv::Point3f> worldPoints_; /**< Pointer to the map.*/
	std::vector<cv::Point3d>* map_; /**< Pointer to map.*/
	cv::Mat projMat_l_, projMat_r_; /**< Projection Matrix of the left and right camera. */
	std::vector<cv::Point2f> stereoPoints_l_, stereoPoints_r_; /**< Stereo matched Points. */
	cv::Mat desc_l_, desc_r_; /**< Descriptors of stereo keypoints. */
	std::vector<unsigned int> mapIdxs_; /**< Map Index. Stores at which index the KeyPoint/3D Point was put in the map. */
	std::vector<cv::DMatch> matchesToPrev_; /**< Matches of stereo points to stereo points of previous node. */
	std::vector<cv::Point3f> nodePoints_; /**< 3D Points in camera/node frame (triangulated stereoPoints). */
	cv::Mat rVecRel_, tVecRel_; /**< rVec, tVec to previous node. */
	cv::Mat rVecAbs_, tVecAbs_; /**< rVec, tVec to world frame ("odom"). */
	
	/*
	 * Default Constructor.
	 * */
	Node() {}
	
	/*
	 * Constructor that creates an empty node with given projection matrices.
	 * @param[in] map Map to be filled with all Points.
	 * @param[in] projMat_l Projection matrix for left camera.
	 * @param[in] projmat_r projection matrix for right camera.
	 * */
	Node(std::vector<cv::Point3d>& map,const cv::Matx34d& projMat_l,const cv::Matx34d& projMat_r) :
		projMat_l_(projMat_l),
		projMat_r_(projMat_r)
		{
		map_ = &map;
		}
 
	/*
	 * Constructor that already adds stereo matched points and triangulated 3D points.
	 * @param[in] map Map to be filled with all Points.
	 * @param[in] projMat_l Projection matrix for left camera.
	 * @param[in] projmat_r projection matrix for right camera.
	 * @param[in] img_l Left camera image.
	 * @param[in] img_r Right camera image.
	 * */
	Node(std::vector<cv::Point3d>& map, const cv::Matx34d& projMat_l, const cv::Matx34d& projMat_r, const cv::Mat& img_l, const cv::Mat& img_r) :
		projMat_l_(projMat_l),
		projMat_r_(projMat_r)
		{
		map_ = &map;
		pointsFromImages(img_l, img_r);
		}
	
	/*
	 * Constructor that fills all information by an image pair and a previous node.
	 * Previous node needs to have R, T, 2D and 3D points filled.
	 * @param[in] map Map to be filled with all Points.
	 * @param[in] projMat_l Projection matrix for left camera.
	 * @param[in] projmat_r projection matrix for right camera.
	 * @param[in] img_l Left camera image.
	 * @param[in] img_r Right camera image.
	 * @param[in] previousNode The previous node to relate this node to.
	 * @param[in] mapFromBadPose Set this to 'true' if points still should be put into the map when there was no match to previous node.
	 * */
	Node(std::vector<cv::Point3d>& map, const cv::Matx34d& projMat_l, const cv::Matx34d& projMat_r, const cv::Mat& img_l, const cv::Mat& img_r, const Node& previousNode, bool mapFromBadPose = false) :
		projMat_l_(projMat_l),
		projMat_r_(projMat_r)
		{
		map_ = &map;
		pointsFromImages(img_l, img_r);
		putIntoWorld(previousNode, mapFromBadPose);
		}
	
	/*
	 * Generates a first node at 0.
	 * All 3D points will be put in an own vector of map as they don't have to be matched to those of a previous node.
	 * @param[in] map Map to be filled with all Points.
	 * @param[in] projMat_l Projection matrix for left camera.
	 * @param[in] projmat_r projection matrix for right camera.
	 * @param[in] img_l Left camera image.
	 * @param[in] img_r Right camera image.
	 * @return The created node.
	 * */
	static Node firstNode(std::vector<cv::Point3d>& map, const cv::Matx34d& projMat_l, const cv::Matx34d& projMat_r, const cv::Mat& img_l, const cv::Mat& img_r)
		{
		// Create Node that already has all coordinate data filled
		Node n(map, projMat_l, projMat_r, img_l, img_r);
		// It's the first node, it practically defines the world frame
		n.rVecRel_ = cv::Mat::zeros(3, 1, CV_64F);
		n.tVecRel_ = cv::Mat::zeros(3, 1, CV_64F);
		n.rVecAbs_ = cv::Mat::zeros(3, 1, CV_64F);
		n.tVecAbs_ = cv::Mat::zeros(3, 1, CV_64F);
		// Put all found 3D points into the map and save where they have been put (no need to transform them)
		for (unsigned int i = 0; i < n.nodePoints_.size(); ++i)
			{
			n.worldPoints_.push_back(n.nodePoints_[i]);
			n.map_->push_back(n.nodePoints_[i]);
			n.mapIdxs_.push_back(i);
			}
		return n;
		}
	
	/*
	 * Generates a first node with a given transformation.
	 * This is for cameras mounted on a vehicle or robot. The transformation is from the camera to the world frame so that all
	 * 3D Points and further nodes can be transformed into the world frame.
	 * All 3D points will be put in an own vector of map as they don't have to be matched to those of a previous node.
	 * @param[in] map Map to be filled with all Points.
	 * @param[in] projMat_l Projection matrix for left camera.
	 * @param[in] projmat_r projection matrix for right camera.
	 * @param[in] img_l Left camera image.
	 * @param[in] img_r Right camera image.
	 * @param[in] rVec Rotation vector of the transformation in rodrigues format. Should be of double type.
	 * @param[in] tVec Translation vector of the transformation. Should be of double type.
	 * @return The created node.
	 * */
	static Node firstNode(std::vector<cv::Point3d>& map, const cv::Matx34d& projMat_l, const cv::Matx34d& projMat_r, const cv::Mat& img_l, const cv::Mat& img_r, const cv::Mat& rVec, const cv::Mat& tVec)
		{
		// Create Node that already has all coordinate data filled
		Node n(map, projMat_l, projMat_r, img_l, img_r);
		// It's the first node, it practically defines the world frame
		n.rVecRel_ = -rVec;
		n.tVecRel_ = -tVec;
		n.rVecAbs_ = rVec;
		n.tVecAbs_ = tVec;
		// Put all found 3D points into the map after transforming them into the world frame and save where they have been put
		cv::Mat R, RT = cv::Mat::eye(4, 4, CV_64F);
		cv::Rodrigues(n.rVecAbs_, R);
		R.copyTo(RT.colRange(0, 3).rowRange(0, 3));
		n.tVecAbs_.copyTo(RT.rowRange(0, 3).col(3));
		cv::perspectiveTransform(n.nodePoints_, n.worldPoints_, RT);
		for (unsigned int i = 0; i < n.worldPoints_.size(); ++i)
			{
			n.map_->push_back(n.worldPoints_[i]);
			n.mapIdxs_.push_back(i);
			}
		return n;
		}

	/*
	 * Matches stereo images.
	 * Matches two rectified stereo images, filters bad keypoints, stores good ones and triangulates them into 3D space.
	 * @param[in] img_l Left camera image.
	 * @param[in] img_r Right camera image.
	 * */
	void pointsFromImages(const cv::Mat& img_l, const cv::Mat& img_r)
		{

		// In case this method gets called more than once, clear data first
		stereoPoints_l_.clear();
		stereoPoints_r_.clear();
		desc_l_ = cv::Mat();
		desc_r_ = cv::Mat();
		nodePoints_.clear();

		// Detect features in left and right image
		std::vector<cv::KeyPoint> keyPoints_l, keyPoints_r;
		cv::Mat descriptors_l, descriptors_r;
		fDetector->detect(img_l, keyPoints_l);
		fDetector->detect(img_r, keyPoints_r);
		dExtractor->compute(img_l, keyPoints_l, descriptors_l);
		dExtractor->compute(img_r, keyPoints_r, descriptors_r);
//		std::cout << "Found features left:		" << keyPoints_l.size() << std::endl;
//		std::cout << "Found features right:		" << keyPoints_r.size() << std::endl;
		
		// Match features with a descriptor(!)-distance below a threshold
		std::vector< std::vector<cv::DMatch> > matches;
		dMatcher->radiusMatch(descriptors_l, descriptors_r, matches, 35);
		
		// Only use matches that fulfill the epipolar constraint, thus lying on a horizontal line
		std::vector<cv::Point2f> refinedPoints_l, refinedPoints_r;
		cv::Mat refinedDesc_l, refinedDesc_r;
		for(unsigned int i = 0; i < matches.size(); ++i)
			{
			// Check if points are on epiline
			if (matches[i].size() > 0 && fabs(keyPoints_l[matches[i][0].queryIdx].pt.y - keyPoints_r[matches[i][0].trainIdx].pt.y) <= 1)
				{
				// Cameras are parallel -> points in left camera images have to be further right than in right camera images
				if (keyPoints_l[matches[i][0].queryIdx].pt.x - keyPoints_r[matches[i][0].trainIdx].pt.x > 0 && keyPoints_l[matches[i][0].queryIdx].pt.x - keyPoints_r[matches[i][0].trainIdx].pt.x < 210)
					{
					refinedPoints_l.push_back(keyPoints_l[matches[i][0].queryIdx].pt);
					refinedPoints_r.push_back(keyPoints_r[matches[i][0].trainIdx].pt);
					
					refinedDesc_l.push_back(descriptors_l.row(matches[i][0].queryIdx));
					refinedDesc_r.push_back(descriptors_r.row(matches[i][0].trainIdx));
					}
				}
			}
//		std::cout << "Matches found (on epilines):	" << refinedPoints_l.size() << std::endl;
		
		
		// Remove outliers by RANSAC
		std::vector<unsigned char> inlierStatus;
		cv::findFundamentalMat(refinedPoints_l, refinedPoints_r, CV_FM_RANSAC, 3., 0.99, inlierStatus);
		std::vector<cv::Point2f> ransacProofPoints_l, ransacProofPoints_r;
		cv::Mat ransacProofDesc_l, ransacProofDesc_r;
		for(unsigned int i = 0; i < inlierStatus.size(); ++i)
			{
			if(inlierStatus[i])
				{
				ransacProofPoints_l.push_back(refinedPoints_l[i]);
				ransacProofPoints_r.push_back(refinedPoints_r[i]);
				
				ransacProofDesc_l.push_back(refinedDesc_l.row(i));
				ransacProofDesc_r.push_back(refinedDesc_r.row(i));
				}
			}
		
		if (!ransacProofPoints_l.empty())
			{
			// Calculate 3D points
			cv::Mat triangulatedPointMat;
			std::vector<cv::Point3f> triangulatedPoints;
			cv::triangulatePoints(projMat_l_, projMat_r_, ransacProofPoints_l, ransacProofPoints_r, triangulatedPointMat);
			cv::convertPointsFromHomogeneous(triangulatedPointMat.t(), triangulatedPoints);

			// Filter points that are too far away
			for (unsigned int i = 0; i < triangulatedPoints.size(); ++i)
				{
				if (triangulatedPoints[i].z < 15)
					{
					stereoPoints_l_.push_back(ransacProofPoints_l[i]);
					stereoPoints_r_.push_back(ransacProofPoints_r[i]);
				
					desc_l_.push_back(ransacProofDesc_l.row(i));
					desc_r_.push_back(ransacProofDesc_r.row(i));
					
					nodePoints_.push_back(triangulatedPoints[i]);
					}
				}
			}
		std::cout << "Matches found (after RANSAC and >z after triang):	" << stereoPoints_l_.size() << std::endl;
		}
		
	/*
	 * Calculates the relation of this node to a previous node and the world.
	 * This method compares the stereo matched features/descriptors of this node and a given node. Using the 3D points of the
	 * previous node it uses PnP to get the relative pose from this node to the previous. Using the absolute pose of the previous
	 * node it then calculates the absolute pose of this node.
	 * @param[in] previousNode The node to compare this one to.
	 * @param[in] useBadPose Set this to 'true' if points still should be put into the map when there was no match to previous node.
	 * @return true if nodePoints_ were transformed into world frame and put into map, false if it couldn't be matched successfully.
	 * */
	bool putIntoWorld(const Node& previousNode, bool useBadPose = false)
		{
		// Match features with a descriptor(!)-distance below a threshold
		std::vector< std::vector<cv::DMatch> > radiusMatches;
		dMatcher->radiusMatch(desc_l_, previousNode.desc_l_, radiusMatches, 35);
		
		// Create vectors containing the matched points
		std::vector<cv::Point2f> matchedPointsCurr1, matchedPointsPrev1;
		std::vector<cv::Point3f> matchedNodePointsPrev1;
		std::vector<cv::DMatch> matches1;
		std::vector<double> dist;
		double meanDist = 0;
		for(unsigned int i = 0; i < radiusMatches.size(); ++i)
			{
			if (radiusMatches[i].size() > 0)
				{
				matchedPointsCurr1.push_back(stereoPoints_l_[radiusMatches[i][0].queryIdx]);
				
				matchedPointsPrev1.push_back(previousNode.stereoPoints_l_[radiusMatches[i][0].trainIdx]);
				matchedNodePointsPrev1.push_back(previousNode.nodePoints_[radiusMatches[i][0].trainIdx]);
				
				matches1.push_back(radiusMatches[i][0]);
				
				double xDist = stereoPoints_l_[radiusMatches[i][0].queryIdx].x - previousNode.stereoPoints_l_[radiusMatches[i][0].trainIdx].x;
				double yDist = stereoPoints_l_[radiusMatches[i][0].queryIdx].y - previousNode.stereoPoints_l_[radiusMatches[i][0].trainIdx].y;
				// Add all distances up to calculate the mean
				double d = std::sqrt(xDist*xDist + yDist*yDist);
				dist.push_back(d);
				meanDist += d;
				}
			}
		meanDist = meanDist/matches1.size();
		std::cout << "meanDist prev-curr:		" << meanDist << std::endl;
		std::cout << "Matches1 prev-curr:		" << matches1.size() << std::endl;
		
		// Only use matches that are below 2*meanDist to eliminate wrong matches
		std::vector<cv::Point2f> matchedPointsCurr, matchedPointsPrev;
		std::vector<cv::Point3f> matchedNodePointsPrev;
		std::vector<cv::DMatch> matches;
		for(unsigned int i = 0; i < matches1.size(); ++i)
			{
			if (dist[i] < 2.*meanDist)
				{
				matchedPointsCurr.push_back(matchedPointsCurr1[i]);
				matchedPointsPrev.push_back(matchedPointsPrev1[i]);
				matchedNodePointsPrev.push_back(matchedNodePointsPrev1[i]);
				matches.push_back(matches1[i]);
				}
			}
		std::cout << "Matches prev-curr:		" << matches.size() << std::endl;
		
		// Remove outliers by double RANSAC to yield better results
		std::vector<unsigned char> inlierStatus1;
		if (!useBadPose && matches.size() < 8) return false;
		else if (matches.size() >= 8) cv::findFundamentalMat(matchedPointsCurr, matchedPointsPrev, CV_FM_RANSAC, 1., 0.99, inlierStatus1);
		std::vector<cv::Point2f> filteredMatchedPointsCurr1, filteredMatchedPointsPrev1;
		std::vector<cv::Point3f> filteredMatchedNodePointsPrev1;
		std::vector<cv::DMatch> matchesToPrev;
		for(unsigned int i = 0; i < inlierStatus1.size(); ++i)
			{
			if(inlierStatus1[i])
				{
				filteredMatchedPointsCurr1.push_back(matchedPointsCurr[i]);
				filteredMatchedPointsPrev1.push_back(matchedPointsPrev[i]);
				filteredMatchedNodePointsPrev1.push_back(matchedNodePointsPrev[i]);
				matchesToPrev.push_back(matches[i]);
				}
			}
		std::cout << "Matches prev-curr after RANSAC1:	" << matchesToPrev.size() << std::endl;
		
		
		std::vector<unsigned char> inlierStatus;
		if (!useBadPose && matchesToPrev.size() < 8) return false;
		else if (matchesToPrev.size() >= 8) cv::findFundamentalMat(filteredMatchedPointsCurr1, filteredMatchedPointsPrev1, CV_FM_RANSAC, 1., 0.99, inlierStatus);
		std::vector<cv::Point2f> filteredMatchedPointsCurr;
		std::vector<cv::Point3f> filteredMatchedNodePointsPrev;
		for(unsigned int i = 0; i < inlierStatus.size(); ++i)
			{
			if(inlierStatus[i])
				{
				filteredMatchedPointsCurr.push_back(filteredMatchedPointsCurr1[i]);
				filteredMatchedNodePointsPrev.push_back(filteredMatchedNodePointsPrev1[i]);
				matchesToPrev_.push_back(matchesToPrev[i]);
				}
			}
		std::cout << "Matches prev-curr after RANSAC2:	" << matchesToPrev_.size() << std::endl;
		
		// Try to get pose between previous and current node (tf previous->current)
		if (!useBadPose && filteredMatchedPointsCurr.size() < 8) return false;
		else if (useBadPose && filteredMatchedPointsCurr.size() < 8)
			{
			std::cout << "Found less than 5 matches. useBadPose == true -> using rVec, tVec of previous node." << std::endl;
			rVecRel_ = previousNode.rVecRel_;
			tVecRel_ = previousNode.tVecRel_;
			}
		else cv::solvePnPRansac(filteredMatchedNodePointsPrev, filteredMatchedPointsCurr, projMat_l_.colRange(0,3), cv::noArray(), rVecRel_, tVecRel_, false, 500, 2.0);

		std::cout << "rVecRel_:	" << rVecRel_ << std::endl;
		std::cout << "tVecRel_:	" << tVecRel_ << std::endl;
		
		// Calculate absolute pose (tf current->world/first)
		cv::composeRT(-rVecRel_, -tVecRel_, previousNode.rVecAbs_, previousNode.tVecAbs_, rVecAbs_, tVecAbs_);
		std::cout << "rVecAbs_:	" << rVecAbs_ << std::endl;
		std::cout << "tVecAbs_:	" << tVecAbs_ << std::endl;
		
		// Check if map pointer is valid and create reference for easier access
		if (map_ == NULL) throw std::runtime_error("Error in Node::putIntoWorld: Pointer to map_ is a NULL pointer!");
		
		// Transform 3D points of this node into the world frame and add them to the map
		cv::Mat R, RT = cv::Mat::eye(4, 4, CV_64F);
		cv::Rodrigues(rVecAbs_, R);
		R.copyTo(RT.colRange(0, 3).rowRange(0, 3));
		tVecAbs_.copyTo(RT.rowRange(0, 3).col(3));
		cv::perspectiveTransform(nodePoints_, worldPoints_, RT);
//		std::cout << "RT:	" << std::endl << RT << std::endl;

//		std::cout << std::endl << "Map size before adding new elements:	" << (*map_).size() << std::endl;
		mapIdxs_ = std::vector<unsigned int>(stereoPoints_l_.size(), -1);
		for (unsigned int i = 0; i < matchesToPrev_.size(); ++i)
			{
			// Check if distance between 3D points is below a threshold
			if (cv::norm(worldPoints_[matchesToPrev_[i].queryIdx] - previousNode.worldPoints_[matchesToPrev_[i].trainIdx]) < 0.3)
				{
				// Check if point of previous node is a point that was put into the map
				if (previousNode.mapIdxs_[matchesToPrev_[i].trainIdx] == -1)
					{
					mapIdxs_[matchesToPrev_[i].queryIdx] = map_->size(); // Use size before adding so we don't have to substract 1
					map_->push_back(worldPoints_[matchesToPrev_[i].queryIdx]);
					}
				else
					{
					mapIdxs_[matchesToPrev_[i].queryIdx] = previousNode.mapIdxs_[matchesToPrev_[i].trainIdx];
					}
				}
			}
		std::cout << std::endl << "New map size:		" << map_->size() << std::endl;
		return true;
		}
	
	};

cv::Ptr<cv::FeatureDetector> Node::fDetector = cv::FeatureDetector::create("SURF");
cv::Ptr<cv::DescriptorExtractor> Node::dExtractor = cv::DescriptorExtractor::create("ORB");
cv::Ptr<cv::DescriptorMatcher> Node::dMatcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

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


		/*
		 * Constructor.
		 * @param[in] nh The nodehandle to use.
		 * @param[in] stereoNamespace Name of the stereo namespace to use.
		 * */
		StereoSubscriber(ros::NodeHandle& nh, const std::string stereoNamespace) :
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


		/*
		 * Simple method to publish two openCV images in two ROS topics.
		 * @param[in] lImage Left image to publish.
		 * @param[in] rImage Right image to publish.
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

/**
 * Class for visual odometry from stereo images.
 */
class VisualOdometer
	{
	private:
		StereoSubscriber* stereoSub_ = NULL; /**< Pointer to stereoSubscriber to get new images from. */
		std::string parentFrame_; /**< Name of the world frame. */
		std::string childFrame_; /**< Name of the baselink frame. */
		std::string cameraFrame_; /**< Name of the camera frame. */
		unsigned int lastImgIdx_; /**< Index oof last image received. */
		tf::TransformListener tfListener_; /**< TransformListener to receive tf data. */
		tf::TransformBroadcaster tfBr_; /**< TransformBroadcaster to send tf data. */
		ros::Publisher posePub_; /**< Pose publisher. */
		ros::Publisher pcPub_; /**< PointCloud publisher. */
		unsigned int matchFails_; /**< Number of failed matches. */
	
	public:
		cv::Mat imgPrev_l_; /**< Previous left image. */
		cv::Mat imgPrev_r_; /**< Previous right image. */
		cv::Mat imgCurr_l_; /**< Current left image. */
		cv::Mat imgCurr_r_; /**< Current right image. */
		std::vector<Node> nodes_; /**< Contains all created nodes. */
		std::vector<tf::Transform> transforms_; /**< Contains all calculated Transforms (corresponding to nodes from nodes_). */
		std::vector<cv::Point3d> map_; /**< Map containing the 3D Points (in world frame) */
		
		/*
		 * Constructor.
		 * @param[in] stereoSub The stereoSubscriber to get the images from.
		 * @param[in] parentFrame Name of the world frame.
		 * @param[in] childFrame Name of the baselink frame.
		 * @param[in] cameraFrame Name of the camera frame.
		 * */
		VisualOdometer(StereoSubscriber& stereoSub, const std::string parentFrame, const std::string childFrame, const std::string cameraFrame) :
			parentFrame_(parentFrame),
			childFrame_(childFrame),
			cameraFrame_(cameraFrame),
			lastImgIdx_(0),
			stereoSub_(&stereoSub)
			{
			ros::NodeHandle nh;
			posePub_ = nh.advertise<geometry_msgs::PoseStamped>("sp_navigation/Pose", 50);
			pcPub_ = nh.advertise<PointCloud>("pointcloud", 50);
			}
		
		/*
		 * Grabs an image pair from the StereoSubscriber.
		 * @return true if new pair of pictures was received and successfully updated, false otherwise.
		 * */
		bool getImagePair()
			{
			if (!stereoSub_) throw std::runtime_error("Error in VisualOdometer::getImagePair: stereoSub_ is a NULL pointer!");
			if (stereoSub_->imgConstPtr_l_ != NULL && stereoSub_->imgConstPtr_r_ != NULL)
				{
				if (imgCurr_l_.empty() || imgCurr_r_.empty())
					{
					stereoSub_->imgConstPtr_l_->image.copyTo(imgCurr_l_);
					stereoSub_->imgConstPtr_r_->image.copyTo(imgCurr_r_);
					lastImgIdx_ = stereoSub_->imgIdx;
					std::cout << std::endl << "First image received!" << std::endl;
					return true;
					}
				else if (stereoSub_->imgIdx != lastImgIdx_)
					{
					imgCurr_l_.copyTo(imgPrev_l_);
					imgCurr_r_.copyTo(imgPrev_r_);
					stereoSub_->imgConstPtr_l_->image.copyTo(imgCurr_l_);
					stereoSub_->imgConstPtr_r_->image.copyTo(imgCurr_r_);
					lastImgIdx_ = stereoSub_->imgIdx;
					std::cout << std::endl << "Got new pair of images!" << std::endl;
					return true;
					}
				else return false;
				}
			else std::cout << "No new images received!" << std::endl;
			return false;
			}
		
		/*
		 * Creates a new node if a new pair of images could be grabbed from the StereoSubscriber.
		 * Checks if there are new images available in the StereoSubscriber and if so creates a new node.
		 * @param useBadPose If true, 3D points from bad poses will be used in the map.
		 * @see Node::putIntoWorld()
		 * @return 'true' if nothing happened (no new images) or node successfully created, 'false' if match to previous node failed.
		 * */
		bool update(bool useBadPose = false)
			{
			if (nodes_.empty() && getImagePair())
				{
				// Get Hand-Eye-transform of first node (node to baselink) to know where the 'first camera' was located in the world
				tf::StampedTransform tfInitialNodeBL;
				tfListener_.lookupTransform(childFrame_, cameraFrame_, ros::Time(0), tfInitialNodeBL);
				cv::Mat rVec, tVec;
				rtVecsFromTF(tfInitialNodeBL, rVec, tVec);
				
				std::cout << "nodes_.empty() && getImagePair() -> trying to create first node!" << std::endl;
				
				nodes_.push_back(Node::firstNode(map_, stereoSub_->stCamModel_.left().projectionMatrix(), stereoSub_->stCamModel_.right().projectionMatrix(), imgCurr_l_, imgCurr_r_, rVec, tVec));
				transforms_.push_back(tf::Transform::getIdentity());
				
				std::cout << std::endl << "Map filled by first node with " << map_.size() << " elements." << std::endl;
				return true;
				}
			else if (getImagePair())
				{
//				std::cout << "nodes_.empty() == false -> trying to create new node!" << std::endl;
				Node n(map_, stereoSub_->stCamModel_.left().projectionMatrix(), stereoSub_->stCamModel_.right().projectionMatrix(), imgCurr_l_, imgCurr_r_);
				if (n.putIntoWorld(nodes_.back(), useBadPose))
					{
					std::cout << "Node successfully put in relation to previous node and world, it will be added to the others!" << std::endl;
					nodes_.push_back(n);
					
					tf::Transform tfCurrNodeOdom = tfFromRTVecs(nodes_.back().rVecAbs_, nodes_.back().tVecAbs_);
					// Look up current hand-eye-transformation
					tf::StampedTransform tfNodeBL;
					tfListener_.lookupTransform(childFrame_, cameraFrame_, ros::Time(0), tfNodeBL);
					tf::Transform tfBLOdom = tfCurrNodeOdom*tfNodeBL.inverse();
					transforms_.push_back(tfBLOdom);
					
					std::cout << "Number of nodes:	" << nodes_.size() << std::endl;
					std::cout << "Number of fails:	" << matchFails_ << std::endl;
					return true;
					}
				else 
					{
					std::cout << "<><><><><> Match failed! <><><><><><>" << std::endl;
					matchFails_++;
					return false;
					}
				}
			else return true;
			}
		
		/*
		 * Publishes tf data and pose.
		 * Computes the baselink pose from the current camera pose if not yet computed and publishes it.
		 * */
		void publishTF()
			{
			// If tf data is available, publish it
			if (!transforms_.empty())
				{
				tf::Transform& tfBLOdom = transforms_.back();
				tfBr_.sendTransform(tf::StampedTransform(tfBLOdom, ros::Time::now(), parentFrame_, childFrame_));
		
				// Also publish pose as message
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
			else std::cout << "No tf data." << std::endl;
			}
		
		/*
		 * Creates a pointcloud from the current map and publishes it.
		 * If there is a sBA optimized map available it will use it otherwise it computes the mean map.
		 * @see computeMeanMap()
		 * */
		void publishPC()
			{
			if (!map_.empty())
				{
				PointCloud pointCloud;
				pointCloud.height = 1;
				pointCloud.width = map_.size();
				for (unsigned int i = 0; i < map_.size(); ++i) {pointCloud.points.push_back(PointT(map_[i].x, map_[i].y, map_[i].z));}
				pointCloud.header.frame_id = parentFrame_;
				pointCloud.header.stamp = ros::Time::now().toSec();
				pcPub_.publish(pointCloud);
				}
			}
		
		/*
		 * Runs sparse bundle adjustment over the gathered data.
		 * Processes all the data so that it can be run through bundle adjustment.
		 * @return The projection error if cvsba was used.
		 * */	
		double runSBA()
			{
			if (!map_.empty())
				{
				// Convert all Points into double format
				std::vector<std::vector<cv::Point2d> > imagePoints;
				std::vector<std::vector<int> > visibility;
				std::vector<cv::Mat> camMatrices(nodes_.size());
				std::vector<cv::Mat> tVecs(nodes_.size()), rMats(nodes_.size());
				std::vector<cv::Mat> distCoeffs(nodes_.size(), cv::Mat::zeros(5, 1, CV_64F));
			
				for (unsigned int i = 0; i < nodes_.size(); ++i)
					{
					camMatrices[i] = nodes_[i].projMat_l_.colRange(0,3);
					Rodrigues(nodes_[i].rVecAbs_, rMats[i]);
					tVecs[i] = nodes_[i].tVecAbs_;

					std::vector<cv::Point2d> imagePoint(map_.size());
					std::vector<int> vis(map_.size(), 0);
					for (unsigned int j = 0; j < nodes_[i].mapIdxs_.size(); ++j)
						{
						if (nodes_[i].mapIdxs_[j] != -1)
							{
							imagePoint[nodes_[i].mapIdxs_[j]] = cv::Point2d(nodes_[i].stereoPoints_l_[j].x, nodes_[i].stereoPoints_l_[j].y);
							vis[nodes_[i].mapIdxs_[j]] = 1;
							}
						}
					imagePoints.push_back(imagePoint);
					visibility.push_back(vis);
					}
				std::cout << "Last R, t before sba:		" << std::endl << rMats.back() << std::endl << tVecs.back() << std::endl;
			
				double tickTime = (double)cv::getTickCount();
			
// #######CVSBA######
//				cvsba::Sba sba;
//				cvsba::Sba::Params sbaParams(cvsba::Sba::MOTIONSTRUCTURE, 150, 1e-5, 5, 5, true);
//				sba.setParams(sbaParams);
//				double projErr = sba.run(map_, imagePoints, visibility, cameraMatrices, rVecs, tVecs, distCoeffs);
//				std::cout << "Initial error=" << sba.getInitialReprjError() << ". " << "Final error=" << sba.getFinalReprjError() << std::endl;

// ######openCV BA######
				cv::TermCriteria criteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 70, 1e-10);
				cv::LevMarqSparse::bundleAdjust(map_, imagePoints, visibility, camMatrices, rMats, tVecs, distCoeffs, criteria);
// #####################
			
				tickTime = ((double)cv::getTickCount() - tickTime) / cv::getTickFrequency();
				std::cout << "\nTime used:		" << tickTime*1000 << "ms" << std::endl << std::endl;
				std::cout << "Last rVec, tVec after sba:\n" << rMats.back() << std::endl << tVecs.back() << std::endl;
				std::cout << "tVec of last node:		" << nodes_.back().tVecAbs_ << std::endl;
			
				// At least cvsba seems to change the data where the Matrix headers of R and T point to, so it doesn't change the R, T in the Nodes.
				cv::Mat lastRVec;
				cv::Rodrigues(rMats.back(), lastRVec);
				tf::Transform tfCurrNodeFirstNode = tfFromRTVecs(lastRVec, tVecs.back()); // Change lastRVec to rVecs.back() if using cvsba with rodrigues format
				// Get current hand-eye-tf
				tf::StampedTransform tfNodeBL;
				tfListener_.lookupTransform(childFrame_, cameraFrame_, ros::Time(0), tfNodeBL);
				// Compute tf from the baselink of the robot to world frame and put it as the last transform in the vector
				tf::Transform tfBLOdom = tfCurrNodeFirstNode*tfNodeBL.inverse();
				transforms_.back() = tfBLOdom;
				}
			
//cvsba only			return projErr;
				return 0;
			}
		
		/*
		 * Draws matched features and publishes them.
		 * Draws stereo matches left<->right of current node and matches between current and previous node.
		 * */
		void visualizeMatches()
			{
			if (nodes_.size() >= 2)
				{
				Node& currNode = nodes_.back();
				Node& prevNode = nodes_[nodes_.size()-2];
				
				// Draw matches between previous and current node
				for (unsigned int i = 0; i < currNode.matchesToPrev_.size(); ++i)
					{
					cv::Point2f& pointPrev = prevNode.stereoPoints_l_[currNode.matchesToPrev_[i].trainIdx];
					cv::Point2f& pointCurr = currNode.stereoPoints_l_[currNode.matchesToPrev_[i].queryIdx];
					cv::circle(imgPrev_l_, pointPrev, 1, cv::Scalar(0,200,0));
					//cv::circle(pub_r, visualOdo.pointsCurr_l_[i], 5, cv::Scalar(0,200,200));
					cv::line(imgPrev_l_, pointPrev, pointCurr, cv::Scalar(0,200,0));
					}
				
				// Draw stereo matches after getting min and max disparity to set the color accordingly
				double minDisp = 1000., maxDisp = 0., dispRange;
				std::vector<double> disparities;
				for (unsigned int i = 0; i < currNode.stereoPoints_l_.size(); ++i)
					{
					double disparity = currNode.stereoPoints_l_[i].x - currNode.stereoPoints_r_[i].x;
					minDisp = MIN(disparity, minDisp);
					maxDisp = MAX(disparity, maxDisp);
					disparities.push_back(disparity);
					}
				dispRange = maxDisp - minDisp;
				for (unsigned int i = 0; i < currNode.stereoPoints_l_.size(); ++i)
					{
					cv::Point2f& pointLeft = currNode.stereoPoints_l_[i];
					cv::Point2f& pointRight = currNode.stereoPoints_r_[i];
					cv::line(imgCurr_r_, pointLeft, pointRight, cv::Scalar(0,255*(disparities[i]-minDisp)/dispRange,255));
					cv::circle(imgCurr_r_, pointRight, 1, cv::Scalar(0,255,255));
					//cv::circle(imgCurr_r_, pointLeft, 1, cv::Scalar(0,153,255));
					}
				}
			stereoSub_->publishCVImages(imgPrev_l_, imgCurr_r_);
			}
		
		/*
		 * Returns the latest translation vector from world frame to base_link.
		 * @return The latest translation vector.
		 * */
		tf::Vector3 getOrigin()
			{
			if (!transforms_.empty())
				{
				return transforms_.back().getOrigin();
				}
			else
				{
				return tf::Vector3(0, 0, 0);
				}
			}
		
		/*
		 * Returns the latest rotation Matrix from world frame to base_link.
		 * @return The latest rotation Matrix.
		 * */
		tf::Matrix3x3 getBasis()
			{
			if (!transforms_.empty())
				{
				return transforms_.back().getBasis();
				}
			else
				{
				return tf::Matrix3x3::getIdentity();
				}
			}
		
		/*
		 * Returns the latest rotation from world frame to base_link.
		 * @return The latest rotation as a quaternion.
		 * */
		tf::Quaternion getRotation()
			{
			if (!transforms_.empty())
				{
				return transforms_.back().getRotation();
				}
			else
				{
				return tf::Quaternion::getIdentity();
				}
			}
	};
} //End of namespace


int main(int argc, char** argv)
	{
	cv::initModule_nonfree();

	//Set SURF feature detector parameters
	sp_navigation::Node::fDetector->set("hessianThreshold", 100);
	sp_navigation::Node::fDetector->set("nOctaves", 4);
	sp_navigation::Node::fDetector->set("upright", true);
		
	// Set cross checking of features to true
	sp_navigation::Node::dMatcher->set("crossCheck", true);
	
	ros::init(argc, argv, "sp_navigation");
	ros::NodeHandle nh;
	std::string stereoNamespace = nh.resolveName("stereo");
	std::string worldFrame = nh.resolveName("world");
	std::string robotFrame = nh.resolveName("base_link");
	std::string cameraFrame = nh.resolveName("camera");
	
	sp_navigation::StereoSubscriber stereoSub(nh, stereoNamespace);
	sp_navigation::VisualOdometer visualOdo(stereoSub, worldFrame, robotFrame, cameraFrame);
	
	ros::Rate loopRate(10);
	ros::Time begin = ros::Time::now();
	ros::Duration collDur(9.0);
//	while (ros::ok() && ros::Time::now()-begin < collDur) // run a certain time
//	while (ros::ok() && visualOdo.nodes_.size() < 70) // Run until a certain amount of nodes have been created
	while (ros::ok())
		{
		// Take time of each loop
		double tickTime = (double)cv::getTickCount();
		ros::spinOnce();
		try
			{
			visualOdo.update(false);
			tickTime = ((double)cv::getTickCount() - tickTime) / cv::getTickFrequency();
			visualOdo.publishTF();
			visualOdo.publishPC();
			visualOdo.visualizeMatches();
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
	
	if (ros::ok()) visualOdo.runSBA();
	// continue publishing data after BA has been run
	while (ros::ok())
		{
		visualOdo.publishTF();
		visualOdo.publishPC();
		visualOdo.visualizeMatches();
		loopRate.sleep();
		}
	}
