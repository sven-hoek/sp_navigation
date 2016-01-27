#ifndef STEREO_ODOMETER_HPP
#define STEREO_ODOMETER_HPP

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include "sp_navigation/stereo_subscriber.hpp"
#include "sp_navigation/node.hpp"

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
tf::Transform tfFromRTVecs(const cv::Mat& rVec, const cv::Mat& tVec);

/**
 * Converts a ROS StampedTransform into openCV rVec and tVec.
 * @param[in] tf The Transform to be converted.
 * @param[out] rVec The resulting rotation Vector of type double in Rodrigues-format.
 * @param[out] tVec The resulting translation Vector of type double.
 */
void rtVecsFromTF(const tf::StampedTransform tf, cv::Mat& rVec, cv::Mat& tVec);

/**
 * Class for visual odometry from stereo images.
 */
class VisualOdometer
	{
	private:
		StereoSubscriber* stereoSub_ = NULL; /**< Pointer to stereoSubscriber to get new images from. */
		std::string worldFrame_; /**< Name of the world frame. */
		std::string robotFrame_; /**< Name of the baselink frame. */
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
		
		VisualOdometer(StereoSubscriber& stereoSub, const std::string worldFrame, const std::string robotFrame, const std::string cameraFrame);
		void reset();
		void deleteOldData();
		bool getImagePair();
		bool update(bool useBadPose);
		void publishTF();
		void publishPC();
		bool runSBA(int maxIterations, bool useCVSBA);
		void visualizeMatches();
		tf::Vector3 getOrigin();
		tf::Matrix3x3 getBasis();
		tf::Quaternion getRotation();
	};
}

#endif
