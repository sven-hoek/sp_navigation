#include "sp_navigation/node.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <stdexcept>

namespace sp_navigation
{
/*
 * Default Constructor.
 * */
Node::Node() {}

/*
 * Constructor that creates an empty node with given projection matrices.
 * @param[in] map Map to be filled with all Points.
 * @param[in] projMat_l Projection matrix for left camera.
 * @param[in] projmat_r projection matrix for right camera.
 * */
Node::Node(std::vector<cv::Point3d>& map,const cv::Matx34d& projMat_l,const cv::Matx34d& projMat_r) :
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
Node::Node(std::vector<cv::Point3d>& map, const cv::Matx34d& projMat_l, const cv::Matx34d& projMat_r, const cv::Mat& img_l, const cv::Mat& img_r) :
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
Node::Node(std::vector<cv::Point3d>& map, const cv::Matx34d& projMat_l, const cv::Matx34d& projMat_r, const cv::Mat& img_l, const cv::Mat& img_r, const Node& previousNode, bool mapFromBadPose = false) :
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
Node Node::firstNode(std::vector<cv::Point3d>& map, const cv::Matx34d& projMat_l, const cv::Matx34d& projMat_r, const cv::Mat& img_l, const cv::Mat& img_r)
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
Node Node::firstNode(std::vector<cv::Point3d>& map, const cv::Matx34d& projMat_l, const cv::Matx34d& projMat_r, const cv::Mat& img_l, const cv::Mat& img_r, const cv::Mat& rVec, const cv::Mat& tVec)
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
void Node::pointsFromImages(const cv::Mat& img_l, const cv::Mat& img_r)
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
bool Node::putIntoWorld(const Node& previousNode, bool useBadPose = false)
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
//		std::cout << "meanDist prev-curr:		" << meanDist << std::endl;
//		std::cout << "Matches1 prev-curr:		" << matches1.size() << std::endl;
	
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
//		std::cout << "Matches prev-curr:		" << matches.size() << std::endl;
	
	// Remove outliers by double RANSAC to yield better results
	std::vector<unsigned char> inlierStatus1;
	if (!useBadPose && matches.size() < 8) return false;
	else if (matches.size() >= 8) cv::findFundamentalMat(matchedPointsCurr, matchedPointsPrev, CV_FM_RANSAC, 1., 0.99, inlierStatus1);
	std::vector<cv::Point2f> filteredMatchedPointsCurr1;
	std::vector<cv::Point3f> filteredMatchedNodePointsPrev1;
	std::vector<cv::DMatch> matchesToPrev;
	for(unsigned int i = 0; i < inlierStatus1.size(); ++i)
		{
		if(inlierStatus1[i])
			{
			filteredMatchedPointsCurr1.push_back(matchedPointsCurr[i]);
			filteredMatchedNodePointsPrev1.push_back(matchedNodePointsPrev[i]);
			matchesToPrev.push_back(matches[i]);
			}
		}
//		std::cout << "Matches prev-curr after RANSAC1:	" << matchesToPrev.size() << std::endl;
	
	// Try to get pose between previous and current node (tf previous->current)
	std::vector<unsigned char> inlierStatus;
	if (!useBadPose && filteredMatchedPointsCurr1.size() < 8) return false;
	else if (useBadPose && filteredMatchedPointsCurr1.size() < 8)
		{
		std::cout << "Found less than 5 matches. useBadPose == true -> using rVec, tVec of previous node." << std::endl;
		rVecRel_ = previousNode.rVecRel_;
		tVecRel_ = previousNode.tVecRel_;
		}
	else cv::solvePnPRansac(filteredMatchedNodePointsPrev1, filteredMatchedPointsCurr1, projMat_l_.colRange(0,3), cv::noArray(), rVecRel_, tVecRel_, false, 500, 2.0, 150, inlierStatus);
	
	for(unsigned int i = 0; i < inlierStatus.size(); ++i)
		{
		if(inlierStatus[i]) matchesToPrev_.push_back(matchesToPrev[i]);
		}
//		std::cout << "Matches prev-curr after RANSAC2:	" << matchesToPrev_.size() << std::endl;
//		std::cout << "rVecRel_:	" << rVecRel_ << std::endl;
//		std::cout << "tVecRel_:	" << tVecRel_ << std::endl;
	
	// Calculate absolute pose (tf current->world/first)
	cv::composeRT(-rVecRel_, -tVecRel_, previousNode.rVecAbs_, previousNode.tVecAbs_, rVecAbs_, tVecAbs_);
//		std::cout << "rVecAbs_:	" << rVecAbs_ << std::endl;
//		std::cout << "tVecAbs_:	" << tVecAbs_ << std::endl;
	
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
		if (cv::norm(worldPoints_[matchesToPrev_[i].queryIdx] - previousNode.worldPoints_[matchesToPrev_[i].trainIdx]) < 0.05)
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
//		std::cout << std::endl << "New map size:		" << map_->size() << std::endl;
	return true;
	}
}

