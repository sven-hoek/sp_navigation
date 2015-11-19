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
#include <cvsba/cvsba.h>


namespace sp_navigation
{
// Converts openCV rVec and tVec to a ROS tf::Transform. Note that rVec, tVec must be double format.
tf::Transform tfFromRTVecs(const cv::Mat& rVec, const cv::Mat& tVec)
	{
	tf::Vector3 rVector(rVec.ptr<double>(0)[0], rVec.ptr<double>(1)[0], rVec.ptr<double>(2)[0]);
	tf::Vector3 tVector(tVec.ptr<double>(0)[0], tVec.ptr<double>(1)[0], tVec.ptr<double>(2)[0]);
	tf::Quaternion q = (rVector.length() == 0) ? tf::Quaternion::getIdentity() : tf::Quaternion(rVector, rVector.length());
	return tf::Transform(q, tVector);
	}

// Represents a viewpoint, e.g. a position of a stereo camera
struct Node
	{
	static cv::ORB oFDDE; /**< ORB feature detector and descriptor extractor.*/
	static cv::BFMatcher bfMatcher; /**< Bruteforce matcher with cross-checking of matches.*/
	
	std::vector< std::vector<cv::Point3f> >* map_; /**< Pointer to map consisting of Points, each with 3D information of each viewpoint */
	ros::Time timestamp_; /**< Time of creation of the node. */	
	cv::Mat projMat_l_, projMat_r_; /**< Projection Matrix of the left and right camera. */
	std::vector<cv::Point2f> stereoPoints_l_, stereoPoints_r_; /**< Stereo matched Points. */
	cv::Mat desc_l_, desc_r_; /**< Descriptors of stereo keypoints. */
	std::vector<cv::DMatch> matchesToPrev_; /**< Matches to previous node. */
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
	Node(std::vector< std::vector<cv::Point3f> >& map,const cv::Matx34d& projMat_l,const cv::Matx34d& projMat_r) :
		timestamp_(ros::Time::now()),
		projMat_l_(projMat_l),
		projMat_r_(projMat_r)
		{
		map_ = &map;
		}
	
	/* Constructor that already adds stereo matched points and triangulated 3D points.
	 *
	 * */
	Node(std::vector< std::vector<cv::Point3f> >& map, const cv::Matx34d& projMat_l, const cv::Matx34d& projMat_r, const cv::Mat& img_l, const cv::Mat& img_r) :
		timestamp_(ros::Time::now()),
		projMat_l_(projMat_l),
		projMat_r_(projMat_r)
		{
		map_ = &map;
		pointsFromImages(img_l, img_r);
		}
	
	/* Constructor that fills all information by an image pair and a previous node.
	 * The previous node needs to have R, T and 3D points filled.
	 *
	 * */
	Node(std::vector< std::vector<cv::Point3f> >& map, const cv::Matx34d& projMat_l, const cv::Matx34d& projMat_r, const cv::Mat& img_l, const cv::Mat& img_r, const Node& previousNode, bool mapFromBadPose = false) :
		timestamp_(ros::Time::now()),
		projMat_l_(projMat_l),
		projMat_r_(projMat_r)
		{
		map_ = &map;
		pointsFromImages(img_l, img_r);
		putIntoWorld(previousNode, mapFromBadPose);
		}
	
	/* Generates the first Node from an image pair.
	 *
	 * */
	static Node firstNode(std::vector< std::vector<cv::Point3f> >& map, const cv::Matx34d& projMat_l, const cv::Matx34d& projMat_r, const cv::Mat& img_l, const cv::Mat& img_r)
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
			n.map_->push_back(std::vector<cv::Point3f>());
			n.map_->at(i).push_back(n.nodePoints_[i]);
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

		std::cout << "Found features left:		" << keyPoints_l.size() << std::endl;
		std::cout << "Found features right:		" << keyPoints_r.size() << std::endl;
		
		// Match features with a descriptor(!)-distance below a threshold
		std::vector< std::vector<cv::DMatch> > matches;
		bfMatcher.radiusMatch(descriptors_l, descriptors_r, matches, 35);
		
		// Only use matches that fulfill the epipolar constraint, thus lying on a horizontal line
		std::vector<cv::Point2f> refinedPoints_l, refinedPoints_r;
		cv::Mat refinedDesc_l, refinedDesc_r;
		for(unsigned int i = 0; i < matches.size(); ++i)
			{
			if (matches[i].size() > 0 && fabs(keyPoints_l[matches[i][0].queryIdx].pt.y - keyPoints_r[matches[i][0].trainIdx].pt.y) <= 2)
				{
				refinedPoints_l.push_back(keyPoints_l[matches[i][0].queryIdx].pt);
				refinedPoints_r.push_back(keyPoints_r[matches[i][0].trainIdx].pt);
				
				refinedDesc_l.push_back(descriptors_l.row(matches[i][0].queryIdx));
				refinedDesc_r.push_back(descriptors_r.row(matches[i][0].trainIdx));
				}
			}
		std::cout << "Matches found (on epiline):	" << refinedPoints_l.size() << std::endl;
		
		// Remove outliers by RANSAC
		std::vector<unsigned char> inlierStatus;
		cv::findFundamentalMat(refinedPoints_l, refinedPoints_r, CV_FM_RANSAC, 3., 0.99, inlierStatus);
		for(unsigned int i = 0; i < inlierStatus.size(); ++i)
			{
			if(inlierStatus[i])
				{
				
				stereoPoints_l_.push_back(refinedPoints_l[i]);
				stereoPoints_r_.push_back(refinedPoints_r[i]);
				
				desc_l_.push_back(refinedDesc_l.row(i));
				desc_r_.push_back(refinedDesc_r.row(i));
				}
			}
		std::cout << "Matches found (after RANSAC):	" << stereoPoints_l_.size() << std::endl;
		
		if (!stereoPoints_l_.empty())
			{
			// Calculate 3D points
			cv::Mat triangulatedPointMat;
			cv::triangulatePoints(projMat_l_, projMat_r_, stereoPoints_l_, stereoPoints_r_, triangulatedPointMat);
			cv::convertPointsFromHomogeneous(triangulatedPointMat.t(), nodePoints_);
			}
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
		std::cout << "Matches prev-curr:		" << matches.size() << std::endl;
	
		
		// Remove outliers by RANSAC
		std::vector<unsigned char> inlierStatus;
		if (!useBadPose && matches.size() < 5) return false;
		else if (matches.size() >= 5) cv::findFundamentalMat(matchedPointsCurr, matchedPointsPrev, CV_FM_RANSAC, 3., 0.99, inlierStatus);
		std::vector<cv::Point2f> filteredMatchedPointsCurr;
		std::vector<cv::Point3f> filteredMatchedNodePointsPrev;
		for(unsigned int i = 0; i < inlierStatus.size(); ++i)
			{
			if(inlierStatus[i])
				{
				filteredMatchedPointsCurr.push_back(matchedPointsCurr[i]);
				filteredMatchedNodePointsPrev.push_back(matchedNodePointsPrev[i]);
				matchesToPrev_.push_back(matches[i]);
				}
			}
		std::cout << "Matches prev-curr after RANSAC:	" << matchesToPrev_.size() << std::endl;
		
		// Try to get pose between previous and current node (tf previous->current)
		if (!useBadPose && filteredMatchedPointsCurr.size() < 5) return false;
		else if (useBadPose && filteredMatchedPointsCurr.size() < 5)
			{
			std::cout << "Found less than 5 matches. useBadPose == true -> using rVec, tVec of previous node." << std::endl;
			rVecRel_ = previousNode.rVecRel_;
			tVecRel_ = previousNode.tVecRel_;
			}
		else cv::solvePnPRansac(filteredMatchedNodePointsPrev, filteredMatchedPointsCurr, projMat_l_.colRange(0,3), cv::noArray(), rVecRel_, tVecRel_, false, 500, 2.0);
		
//		rVecRel_ = (cv::Mat_<double>(3,1) << 0., 0., 0.);
//		tVecRel_ = (cv::Mat_<double>(3,1) << 0., 0., -0.1);

//		std::cout << "rVecRel_:	" << rVecRel_ << std::endl;
//		std::cout << "tVecRel_:	" << tVecRel_ << std::endl;
		
		// Calculate absolute pose (tf current->world/first)
		cv::composeRT(-rVecRel_, -tVecRel_, previousNode.rVecAbs_, previousNode.tVecAbs_, rVecAbs_, tVecAbs_);
//		std::cout << "rVecAbs_:	" << rVecAbs_ << std::endl;
//		std::cout << "tVecAbs_:	" << tVecAbs_ << std::endl;
		
		// Check if map pointer is valid and create reference for easier access
		if (map_ == NULL) throw std::runtime_error("Error in Node::putIntoWorld: Pointer to map_ is a NULL pointer!");
		std::vector< std::vector<cv::Point3f> >& mapRef = *map_;
		
		// Transform 3D points of this node into the world frame and add them to the map
		cv::Mat R, RT = cv::Mat::eye(4, 4, CV_64F);
		cv::Rodrigues(rVecAbs_, R);
		R.copyTo(RT.colRange(0, 3).rowRange(0, 3));
		tVecAbs_.copyTo(RT.rowRange(0, 3).col(3));
		std::vector<cv::Point3f> worldPoints;
		cv::perspectiveTransform(nodePoints_, worldPoints, RT);
		
//		std::cout << "RT:	" << std::endl << RT << std::endl;
//		std::cout << std::endl << "Map size before adding new elements:	" << (*map_).size() << std::endl;
		
		for (unsigned int i = 0; i < worldPoints.size(); ++i)
			{
//			std::cout << "i =	" << i << std::endl;
			// Check if point was already found by the last node
			bool foundMatch = false;
			unsigned int mapIdx = 0;
			for (unsigned int j = 0; j < matchesToPrev_.size(); ++j)
				{
//				std::cout << "j =	" << j << std::endl;
				if (matchesToPrev_[j].queryIdx == i)
					{
//					std::cout << "This point was found in previous node";
					mapIdx = previousNode.mapIdxs_[matchesToPrev_[j].trainIdx];
//					std::cout << ", mapIdx =	" << mapIdx << std::endl;
					foundMatch = true;
					break;
					}
				}
			if (!foundMatch)
				{
				mapRef.push_back(std::vector<cv::Point3f>());
				mapIdx = mapRef.size() - 1;
//				std::cout << "This point was not found in previous node, mapIdx =	" << mapIdx << std::endl;
				}
			mapRef[mapIdx].push_back(worldPoints[i]);
//			std::cout << "worldPoint[i=" << i << "] put at mapIdx " << mapIdx << std::endl;
			mapIdxs_.push_back(mapIdx);
			}
		std::cout << std::endl << "New map size:		" << mapRef.size() << std::endl;
		return true;
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
		StereoSubscriber* stereoSub_ = NULL;
		std::string parentFrame_, childFrame_, cameraFrame_;
		unsigned int lastImgIdx_;
		tf::TransformListener tfListener_;
		tf::TransformBroadcaster tfBr_;
		tf::StampedTransform tfInitialNodeBL;
		ros::Publisher posePub_;
		
		unsigned int matchFails_;
	
	public:
		cv::Mat imgPrev_l_, imgPrev_r_;
		cv::Mat imgCurr_l_, imgCurr_r_;
		std::vector<Node> nodes_;
		std::vector<tf::Transform> transforms_;
		std::vector< std::vector<cv::Point3f> > fullMap_; //Map containing all points (in coord system of first node)
		std::vector<cv::Point3d> map_;
		
		/* Constructor
		 *
		 * */
		VisualOdometer(StereoSubscriber& stereoSub, std::string parentFrame, std::string childFrame, std::string cameraFrame) :
			parentFrame_(parentFrame),
			childFrame_(childFrame),
			cameraFrame_(cameraFrame),
			lastImgIdx_(0)
			{
			stereoSub_ = &stereoSub;
			ros::NodeHandle nh;
			posePub_ = nh.advertise<geometry_msgs::PoseStamped>("sp_navigation/Pose", 50);
			}
		
		/* Fetches an stereo image pair from a sp_navigation::StereoSubscriber
		 * returns true if a new pair of pictures was received and successfully updated
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
		
		/* Creates a node from the 'current' pair of pictures if there was a new one. If it successfully computes it's pose, it will get added to
		 * the vector of nodes. This method returns 'false' only if the pose of the new node could not be computed.
		 *
		 * */
		bool update(bool useBadPose = false)
			{
			if (nodes_.empty() && getImagePair())
				{
				std::cout << "nodes_.empty() && getImagePair() -> trying to create first node!" << std::endl;
				nodes_.push_back(Node::firstNode(fullMap_, stereoSub_->stCamModel_.left().projectionMatrix(), stereoSub_->stCamModel_.right().projectionMatrix(), imgCurr_l_, imgCurr_r_));
				// Save Hand-Eye-transform of first node (node to baselink) to know where the 'first camera' was located in the world
				// All nodes relate to position of the first node.
				tfListener_.lookupTransform(childFrame_, cameraFrame_, ros::Time(0), tfInitialNodeBL);
				std::cout << std::endl << "Map filled by first node with " << fullMap_.size() << " elements." << std::endl;
				return true;
				}
			else if (getImagePair())
				{
				std::cout << "nodes_.empty() == false -> trying to create new node!" << std::endl;
				Node n(fullMap_, stereoSub_->stCamModel_.left().projectionMatrix(), stereoSub_->stCamModel_.right().projectionMatrix(), imgCurr_l_, imgCurr_r_);
				if (n.putIntoWorld(nodes_.back(), useBadPose))
					{
					std::cout << "Node successfully put in relation to previous node and world, it will be added to the others!" << std::endl;
					nodes_.push_back(n);
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
		
		/* Calculates and publishes the TF information of the last node.
		 *
		 * */
		void publishTF()
			{
			// If there was no tf calculated for the current node, calculate it and add it to the vector
			if (!nodes_.empty() && transforms_.size() < nodes_.size())
				{
				tf::Transform tfCurrNodeFirstNode = tfFromRTVecs(nodes_.back().rVecAbs_, nodes_.back().tVecAbs_);
				
				tf::StampedTransform tfNodeBL;
				tfListener_.lookupTransform(childFrame_, cameraFrame_, ros::Time(0), tfNodeBL);
				
				tf::Transform tfBLOdom = tfInitialNodeBL*(tfCurrNodeFirstNode*tfNodeBL.inverse());
				transforms_.push_back(tfBLOdom);
				}
			
			// If tf data is up to date, publish it
			if (!nodes_.empty() && transforms_.size() == nodes_.size())
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
		
		/* Calculates the mean of all points for each vector in fullMap_ and stores it in map_
		 *
		 * */	
		void computeMeanMap()
			{
			map_.clear();
			unsigned int maxSize = 0, maxIdx = 0;
			double average = 0;
			for (unsigned int i = 0; i < fullMap_.size(); ++i)
				{
				int size = fullMap_[i].size();
				if (size > maxSize)
					{
					maxSize = size;
					maxIdx = i;
					}
				average += fullMap_[i].size();
				
				cv::Point3d mapPoint;
				for (unsigned int j = 0; j < fullMap_[i].size(); ++j)
					{
					mapPoint += cv::Point3d(fullMap_[i][j].x, fullMap_[i][j].y, fullMap_[i][j].z);
					}
				mapPoint *= 1/(double)fullMap_[i].size();
				map_.push_back(mapPoint);
				std::cout << mapPoint << std::endl;
				}
			average = average / fullMap_.size();
			std::cout << "Max Size of points per point 	[" << maxIdx << "]:	" << maxSize << std::endl;
			std::cout << "Avg Size of points per point:		" << average << std::endl;
			std::cout << "Points in the full map:		" << fullMap_.size() << std::endl;
			std::cout << "Points in the meanizizized map:		" << map_.size() << std::endl;
			}
		
		/* draws the features/matches and publishes them
		 *
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
				
				// Draw stereo matches
				for (unsigned int i = 0; i < currNode.stereoPoints_l_.size(); ++i)
					{
					cv::Point2f& pointLeft = currNode.stereoPoints_l_[i];
					cv::Point2f& pointRight = currNode.stereoPoints_r_[i];
					cv::circle(imgCurr_r_, pointLeft, 1, cv::Scalar(0,153,255));
					cv::circle(imgCurr_r_, pointRight, 1, cv::Scalar(0,255,255));
					cv::line(imgCurr_r_, pointLeft, pointRight, cv::Scalar(255,153,0));
					}
				}
			stereoSub_->publishCVImages(imgPrev_l_, imgCurr_r_);
			}
	};

} //End of namespace


int main(int argc, char** argv)
	{
	int nfeatures = 3000;
	float scaleFactor = argc > 1 ? std::atof(argv[1]) : 1.2f;
	int nlevels = argc > 2 ? std::atoi(argv[2]) : 12;
	int edgeThreshold = argc > 3 ? std::atoi(argv[3]) : 21;
	int firstLevel = 0;
	int WTA_K = argc > 4 ? std::atoi(argv[4]) : 2;
	int norm = WTA_K == 2 ? cv::NORM_HAMMING : cv::NORM_HAMMING2;
	int scoreType = cv::ORB::HARRIS_SCORE;
	int patchSize = edgeThreshold;
	
	sp_navigation::Node::oFDDE = cv::ORB(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize);
	sp_navigation::Node::bfMatcher = cv::BFMatcher(norm, true); // Only cross-checked matches will be used
	
	ros::init(argc, argv, "sp_navigation");
	ros::NodeHandle nh;
	sp_navigation::StereoSubscriber stereoSub(nh, "stereo");
	sp_navigation::VisualOdometer visualOdo(stereoSub, std::string("odom"), std::string("base_link"), std::string("VRMAGIC"));
	
	ros::Rate loopRate(10);
	while (ros::ok() && visualOdo.nodes_.size() < 80)
		{
		// Take time of each loop
		double tickTime = (double)cv::getTickCount();
		ros::spinOnce();
		try
			{
			visualOdo.update(false);
			tickTime = ((double)cv::getTickCount() - tickTime) / cv::getTickFrequency();
			visualOdo.publishTF();
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
	
	visualOdo.computeMeanMap();
	
	while (ros::ok())
		{
		ros::spinOnce();
		visualOdo.publishTF();
		visualOdo.visualizeMatches();
		loopRate.sleep();
		}
	}
