#include "sp_navigation/visual_odometer.hpp"
#include <cvsba/cvsba.h>	// External library for sparse bundle adjustment
#include <opencv2/contrib/contrib.hpp>


namespace sp_navigation
{
tf::Transform tfFromRTVecs(const cv::Mat& rVec, const cv::Mat& tVec)
	{
	tf::Vector3 rVector(rVec.ptr<double>(0)[0], rVec.ptr<double>(1)[0], rVec.ptr<double>(2)[0]);
	tf::Vector3 tVector(tVec.ptr<double>(0)[0], tVec.ptr<double>(1)[0], tVec.ptr<double>(2)[0]);
	tf::Quaternion q = (rVector.length() == 0) ? tf::Quaternion::getIdentity() : tf::Quaternion(rVector, rVector.length());
	return tf::Transform(q, tVector);
	}

void rtVecsFromTF(const tf::StampedTransform tf, cv::Mat& rVec, cv::Mat& tVec)
	{
	rVec = (cv::Mat_<double>(3,1) << tf.getRotation().getAxis().getX(), tf.getRotation().getAxis().getY(), tf.getRotation().getAxis().getZ());
	rVec *= tf.getRotation().getAngle();
	tVec = (cv::Mat_<double>(3,1) << tf.getOrigin().getX(), tf.getOrigin().getY(), tf.getOrigin().getZ());
	}

/*
 * Constructor.
 * @param[in] stereoSub The stereoSubscriber to get the images from.
 * @param[in] worldFrame Name of the world frame.
 * @param[in] robotFrame Name of the baselink frame.
 * @param[in] cameraFrame Name of the camera frame.
 * */
VisualOdometer::VisualOdometer(StereoSubscriber& stereoSub, const std::string worldFrame, const std::string robotFrame, const std::string cameraFrame) :
	worldFrame_(worldFrame),
	robotFrame_(robotFrame),
	cameraFrame_(cameraFrame),
	lastImgIdx_(0),
	stereoSub_(&stereoSub)
	{
	ros::NodeHandle nh;
	posePub_ = nh.advertise<geometry_msgs::PoseStamped>("sp_navigation/Pose", 50);
	pcPub_ = nh.advertise<PointCloud>("sp_navigation/pointcloud", 50);
	}

/*
 * Resets all collected data.
 * */
void VisualOdometer::reset()
	{
	matchFails_ = 0;
	nodes_.clear();
	map_.clear();
	transforms_.clear();
	}

/*
 * Deletes all points in the map and all nodes and transforms except for the last one.
 * This might be helpful if not too much data should be processed by sBA for faster compution.
 * */
void VisualOdometer::deleteOldData()
	{
	Node n = nodes_.back();
	for (unsigned int i = 0; i < n.mapIdxs_.size(); ++i) n.mapIdxs_[i] = -1;
	tf::Transform tf = transforms_.back();
	reset();
	nodes_.push_back(n);
	transforms_.push_back(tf);
	}

/*
 * Grabs an image pair from the StereoSubscriber.
 * @return true if new pair of pictures was received and successfully updated, false otherwise.
 * */
bool VisualOdometer::getImagePair()
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
 * @param[in] useBadPose If true, 3D points from bad poses will be used in the map.
 * @see Node::putIntoWorld()
 * @return 'true' if nothing happened (no new images) or node successfully created, 'false' if match to previous node failed.
 * */
bool VisualOdometer::update(bool useBadPose = false)
	{
	if (nodes_.empty() && getImagePair())
		{
		// Get Hand-Eye-transform of first node (node to baselink) to know where the 'first camera' was located in the world
		tf::StampedTransform tfInitialNodeBL;
		tfListener_.lookupTransform(robotFrame_, cameraFrame_, ros::Time(0), tfInitialNodeBL);
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
			nodes_.push_back(n);
			
			tf::Transform tfCurrNodeOdom = tfFromRTVecs(nodes_.back().rVecAbs_, nodes_.back().tVecAbs_);
			// Look up current hand-eye-transformation
			tf::StampedTransform tfNodeBL;
			tfListener_.lookupTransform(robotFrame_, cameraFrame_, ros::Time(0), tfNodeBL);
			tf::Transform tfBLOdom = tfCurrNodeOdom*tfNodeBL.inverse();
			transforms_.push_back(tfBLOdom);
			
			double yaw, pitch, roll;
			tfBLOdom.getBasis().getEulerYPR(yaw, pitch, roll);
			std::cout << "Robot position:" << std::endl;
			std::cout << "tVec:		[" << tfBLOdom.getOrigin().getX() << ", " << tfBLOdom.getOrigin().getY() << ", " << tfBLOdom.getOrigin().getZ() << "]\n";
			std::cout << "Yaw:		" << yaw << "\n";
			std::cout << "Pitch:		" << pitch << "\n";
			std::cout << "Roll:		" << roll << "\n";
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
void VisualOdometer::publishTF()
	{
	// If tf data is available, publish it
	if (!transforms_.empty())
		{
		tf::Transform& tfBLOdom = transforms_.back();
		tfBr_.sendTransform(tf::StampedTransform(tfBLOdom, ros::Time::now(), worldFrame_, robotFrame_));

		// Also publish pose as message
		geometry_msgs::PoseStamped poseMSG;
		poseMSG.header.stamp = ros::Time::now();
		poseMSG.header.frame_id = worldFrame_;
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
 * */
void VisualOdometer::publishPC()
	{
	if (!map_.empty())
		{
		PointCloud pointCloud;
		pointCloud.height = 1;
		pointCloud.width = map_.size();
		for (unsigned int i = 0; i < map_.size(); ++i) {pointCloud.points.push_back(PointT(map_[i].x, map_[i].y, map_[i].z));}
		pointCloud.header.frame_id = worldFrame_;
		pointCloud.header.stamp = ros::Time::now().toSec();
		pcPub_.publish(pointCloud);
		}
	}

/*
 * Runs sparse bundle adjustment over the gathered data.
 * Processes all the data so that it can be run through bundle adjustment.
 * @param[in] maxIterations The maximum number of iterations to run.
 * @param[in] useCVSBA Set this to true if cvSBA should be used rather than the openCV sBA algorithm.
 * @return True if pose after bundle adjustment gave a plausible result.
 * */	
bool VisualOdometer::runSBA(int maxIterations = 70, bool useCVSBA = false)
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
		if (useCVSBA)
			{
			cvsba::Sba sba;
			cvsba::Sba::Params sbaParams(cvsba::Sba::MOTIONSTRUCTURE, maxIterations, 1e-5, 5, 5, true);
			sba.setParams(sbaParams);
			double projErr = sba.run(map_, imagePoints, visibility, camMatrices, rMats, tVecs, distCoeffs);
			std::cout << "Initial error=" << sba.getInitialReprjError() << "\n";
			std::cout << "Final error=" << sba.getFinalReprjError() << "\n";
			std::cout << "Projection error=" << projErr << std::endl;
			}
		else
			{
			cv::TermCriteria criteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, maxIterations, 1e-10);
			cv::LevMarqSparse::bundleAdjust(map_, imagePoints, visibility, camMatrices, rMats, tVecs, distCoeffs, criteria);
			}
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
		tfListener_.lookupTransform(robotFrame_, cameraFrame_, ros::Time(0), tfNodeBL);
		// Compute tf from the baselink of the robot to world frame and put it as the last transform in the vector
		tf::Transform tfBLOdom = tfCurrNodeFirstNode*tfNodeBL.inverse();
		
		// Discard changes when pitch or yaw is too high (the robot is moving on a plane) after sBA -> rotation axis is too far off the z-axis
		double yaw, pitch, roll;
		tfBLOdom.getBasis().getEulerYPR(yaw, pitch, roll);
		std::cout << "tVec:		[" << tfBLOdom.getOrigin().getX() << ", " << tfBLOdom.getOrigin().getY() << ", " << tfBLOdom.getOrigin().getZ() << "]\n";
		std::cout << "rVec:		[" << tfBLOdom.getRotation().getAxis().getX() << ", " << tfBLOdom.getRotation().getAxis().getY() << ", " << tfBLOdom.getRotation().getAxis().getZ() << "]\n";
		std::cout << "Yaw:		" << yaw << "\n";
		std::cout << "Pitch:		" << pitch << "\n";
		std::cout << "Roll:		" << roll << "\n";
		if (std::fabs(pitch) < 0.12 && std::fabs(yaw) < 0.12 && std::fabs(tfBLOdom.getOrigin().getZ()) < 0.15)
			transforms_.back() = tfBLOdom;
			return true;
		}
		else 
			{
			std::cout << "Bad sBA result!\n";
			return false;
			}
	}

/*
 * Draws matched features and publishes them.
 * Draws stereo matches left<->right of current node and matches between current and previous node.
 * */
void VisualOdometer::visualizeMatches()
	{
	if (nodes_.size() >= 2)
		{
		Node& currNode = nodes_.back();
		Node& prevNode = nodes_[nodes_.size()-2];
		
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
		
		// Draw matches between previous and current node
		for (unsigned int i = 0; i < currNode.matchesToPrev_.size(); ++i)
			{
			cv::Point2f& pointPrev = prevNode.stereoPoints_l_[currNode.matchesToPrev_[i].trainIdx];
			cv::Point2f& pointCurr = currNode.stereoPoints_l_[currNode.matchesToPrev_[i].queryIdx];
			cv::circle(imgPrev_l_, pointPrev, 1, cv::Scalar(0,255,255));
			//cv::circle(pub_r, visualOdo.pointsCurr_l_[i], 5, cv::Scalar(0,200,200));
			cv::line(imgPrev_l_, pointPrev, pointCurr, cv::Scalar(0,255*(disparities[currNode.matchesToPrev_[i].queryIdx]-minDisp)/dispRange,255));
			}
		}
	stereoSub_->publishCVImages(imgPrev_l_, imgCurr_r_);
	}

/*
 * Returns the latest translation vector.
 * @return The latest translation vector.
 * */
tf::Vector3 VisualOdometer::getOrigin()
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
 * Returns the latest rotation Matrix.
 * @return The latest rotation Matrix.
 * */
tf::Matrix3x3 VisualOdometer::getBasis()
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
 * Returns the latest rotation.
 * @return The latest rotation as a quaternion.
 * */
tf::Quaternion VisualOdometer::getRotation()
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
}
