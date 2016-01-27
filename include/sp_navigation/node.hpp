#ifndef NODE_HPP
#define NODE_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace sp_navigation
{
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
	
	Node();
	Node(std::vector<cv::Point3d>& map,const cv::Matx34d& projMat_l,const cv::Matx34d& projMat_r);
	Node(std::vector<cv::Point3d>& map, const cv::Matx34d& projMat_l, const cv::Matx34d& projMat_r, const cv::Mat& img_l, const cv::Mat& img_r);
	Node(std::vector<cv::Point3d>& map, const cv::Matx34d& projMat_l, const cv::Matx34d& projMat_r, const cv::Mat& img_l, const cv::Mat& img_r, const Node& previousNode, bool mapFromBadPose);
	static Node firstNode(std::vector<cv::Point3d>& map, const cv::Matx34d& projMat_l, const cv::Matx34d& projMat_r, const cv::Mat& img_l, const cv::Mat& img_r);
	static Node firstNode(std::vector<cv::Point3d>& map, const cv::Matx34d& projMat_l, const cv::Matx34d& projMat_r, const cv::Mat& img_l, const cv::Mat& img_r, const cv::Mat& rVec, const cv::Mat& tVec);
	void pointsFromImages(const cv::Mat& img_l, const cv::Mat& img_r);
	bool putIntoWorld(const Node& previousNode, bool useBadPose);
	};
}
#endif
