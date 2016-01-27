#include <stdexcept>
#include <ros/ros.h>
#include <opencv2/nonfree/nonfree.hpp>
#include <sp_navigation/node.hpp>
#include <sp_navigation/stereo_subscriber.hpp>
#include <sp_navigation/visual_odometer.hpp>


namespace sp_navigation
{
cv::Ptr<cv::FeatureDetector> Node::fDetector = cv::FeatureDetector::create("ORB");
cv::Ptr<cv::DescriptorExtractor> Node::dExtractor = cv::DescriptorExtractor::create("ORB");
cv::Ptr<cv::DescriptorMatcher> Node::dMatcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

/**
 * Class to navigate a robot using visual odomotry.
 */
class robotMove
	{
	private:
		VisualOdometer odometer_; /**< Visual odometer */
		std::string moveTopic_; /**< Name of topic to pubish velocity. */
		std::string destTopic_; /**< Name of the topic to receive movement destination. */
		ros::Publisher velPub_; /**< Velocity publisher. */
		ros::Subscriber destSub_; /**< Destination subscriber. */
	
	public:
	
		/*
		 * Constructor.
		 * @param[in] stereoSub The stereoSubscriber to get the images from.
		 * @param[in] worldFrame Name of the world frame.
		 * @param[in] robotFrame Name of the baselink frame.
		 * @param[in] cameraFrame Name of the camera frame.
		 * @param[in] Name of the topic to publish velocity.
		 * @param[in] Name of the topic to receive movement dest.
		 * */
		robotMove(StereoSubscriber& stereoSub, std::string worldFrame, std::string robotFrame, std::string cameraFrame, std::string moveTopic, std::string destTopic) :
			odometer_(stereoSub, worldFrame, robotFrame, cameraFrame), 
			moveTopic_(moveTopic),
			destTopic_(destTopic)		
			{
			ros::NodeHandle nh;
			velPub_ = nh.advertise<geometry_msgs::Twist>(moveTopic, 10, true);
			destSub_ = nh.subscribe(destTopic, 1, &robotMove::destCallback, this);
			}
		
		/*
		 * Updates the visual odometer.
		 * @param[in] useBadPose Set this to true if last R, T should be used when none could be found this time.
		 * @return The time that has been used (in seconds).
		 * */
		double update(bool useBadPose = false)
			{
			ros::spinOnce();
			double tickTime = (double)cv::getTickCount();
			odometer_.update(useBadPose);
			tickTime = ((double)cv::getTickCount() - tickTime) / cv::getTickFrequency();
			return tickTime;
			}
		
		/*
		 * Publishes the data from the odometer like pose, 3D map and pictures with matches drawn.
		 * @param[in] publishTF If true, tf data will be published.
		 * @param[in] publishPC If true, map will be published as pointcloud.
		 * @param[in] visualizeMatches If true, matches will be drawn into images and published.
		 * */
		void publishOdomData(bool publishTF = true, bool publishPC = true, bool visualizeMatches = true)
			{
			if (publishTF) odometer_.publishTF();
			if (publishPC) odometer_.publishPC();
			if (visualizeMatches) odometer_.visualizeMatches();
			}
		
		/*
		 * Publishes a twist containing angular and linear velocity to move the robot.
		 * @param[in] velX Velocity in x-direction.
		 * @param[in] velY Velocity in y-direction.
		 * @param[in] velOmega Angular velocity.
		 * */
		void publishVel(double velX, double velY, double velOmega)
			{
			geometry_msgs::Twist velMSG;
			velMSG.linear.x = velX;
			velMSG.linear.y = velY;
			velMSG.angular.z = velOmega;

			velPub_.publish(velMSG);
			}
		
		/*
		 * Calculates the angle between the translation vector pointing from the robot to the point and the x-axis of the robot base.
		 * @param[in] x The x coordinate of the point.
		 * @param[in] y The y coordinate of the point.
		 * @return The angle between the translation vector pointing to the destination and the x-axis of the robot base.
		 * */
		double getAlpha(double x, double y)
			{
			// Transform destination into robot frame and scale vector
			tf::Vector3 localDest = odometer_.transforms_.back().inverse()(tf::Vector3(x, y, 0));
			return std::atan2(localDest.getY(), localDest.getX());
			}
		
		/*
		 * Calculates the distance between the robot and a given point
		 * @param[in] x The x coordinate of the point.
		 * @param[in] y The y coordinate of the point.
		 * @return The distance between the robot and the point.
		 * */
		double getDistance(double x, double y)
			{
			return odometer_.getOrigin().distance(tf::Vector3(x, y, 0.0));
			}
		
		/*
		 * Turns the robot to have a certain yaw relative to the world frame
		 * @param[in] destYaw The yaw to be set in the end [-PI, +PI].
		 * @param[in] speed Scale factor of the speed.
		 * @param[in] deleteData If true, all old data of the odometer will be deleted beforehand.
		 * */
		void setYaw(double destYaw, double speed = 1.0, bool deleteData = true)
			{
			if (destYaw < -M_PI || destYaw > M_PI) throw std::runtime_error("Error in robotMove::setYaw: parameter destYaw is out of bounds.");
			double yaw, pitch, roll, angle;
			odometer_.getBasis().getEulerYPR(yaw, pitch, roll);
			angle = destYaw - yaw;
			if (angle > M_PI) angle = -2.0*M_PI + angle;
			else if (angle < -M_PI) angle = 2.0*M_PI + angle;
			turnByAngle(angle, speed, deleteData);
			}
		
		/*
		 * Turns the robot by a given angle.
		 * @param[in] angle The amount to turn, [-PI, +PI]
		 * @param[in] speed Scale factor of the speed.
		 * @param[in] deleteData If true, all old data of the odometer will be deleted beforehand.
		 * */
		void turnByAngle(double angle, double speed = 1.0, bool deleteData = true)
			{
			if (angle < -M_PI || angle > M_PI) throw std::runtime_error("Error in robotMove::turnByAngle: parameter angle is out of bounds.");
			else if (deleteData) odometer_.deleteOldData();
			double yaw, pitch, roll, destYaw;
			odometer_.getBasis().getEulerYPR(yaw, pitch, roll);
			destYaw = yaw + angle;
			// Check if destYaw is out of [-PI, PI]
			int overflow = 0;
			if (destYaw > M_PI) overflow = 1;
			else if (destYaw < -M_PI) overflow = -1;
			
			speed *= 0.5;
			// Turn
			ros::Rate loopRate(10);
			while (ros::ok() && std::fabs(angle) > 0.06)
				{
				double angVel = angle;
				if (angVel >= 0)
					{
					if (angVel > speed) angVel = speed;
					else if (angVel < 0.2) angVel = 0.2;
					}
				else
					{
					if (angVel < -speed) angVel = -speed;
					else if (angVel > -0.2) angVel = -0.2;
					}
				publishVel(0.0, 0.0, angVel);
				try
					{
					update();
					publishOdomData();
					}
				catch(std::exception& e)
					{
					std::cout << e.what() << std::endl;
					}
				catch(...)
					{
					std::cout << "Something unknown went wrong in robotMove::turnByAngle" << std::endl;
					}
				odometer_.getBasis().getEulerYPR(yaw, pitch, roll);
				if (overflow == 1 && yaw < 0.0) yaw = 2.0*M_PI + yaw;
				else if (overflow == -1 && yaw > 0.0) yaw = -2.0*M_PI + yaw;
				angle = destYaw - yaw;
				loopRate.sleep();
				}
			// Stop
			publishVel(0.0, 0.0, 0.0);
			}
			
		/*
		 * Turns the robot so it looks towards a point.
		 * @param[in] x The x coordinate of the point.
		 * @param[in] x The y coordinate of the point.
		 * @param[in] speed Scale factor of the speed.
		 * @param[in] deleteData If true, all old data of the odometer will be deleted beforehand.
		 * */
		void lookTowards(double x, double y, double speed = 1.0, bool deleteData = true)
			{
			double alpha = getAlpha(x, y);
			turnByAngle(alpha, speed, deleteData);
			}
			
		/*
		 * Searches for the area with the most features visible between a given angle to the left and to the right.
		 * @param[in] angle The maximum angle it will turn to the left and the right (in rad).
		 * @param[in] speed Scale factor of the speed.
		 * @param[in] deleteData If true, all old data of the odometer will be deleted beforehand.
		 * @return The yaw value where the most features have been found.
		 * */
		double searchMaxFeatures(double angle, double speed = 1.0, bool deleteData = true)
			{
			double yaw, pitch, roll, yawBegin;
			odometer_.getBasis().getEulerYPR(yaw, pitch, roll);
			yawBegin = yaw;
			turnByAngle(angle, speed, deleteData);
			// Memorize at which node it started
			int nodeIdxBegin = odometer_.nodes_.size() - 1;
			turnByAngle(-2 * angle, speed, false);
			int nodeIdxEnd = odometer_.nodes_.size() - 1;
			// Turn back so the robot has the same orientation as in the beginning
			setYaw(yawBegin, speed, false);
			
			// Now search for the node with the most features and store its yaw
			unsigned int maxFeatures = 0, maxIndex;
			for (unsigned int i = nodeIdxBegin; i <= nodeIdxEnd; ++i)
				{
				if (odometer_.nodes_[i].stereoPoints_l_.size() > maxFeatures)
					{
					maxFeatures = odometer_.nodes_[i].stereoPoints_l_.size();
					maxIndex = i;
					}
				}
			// Get yaw
			odometer_.transforms_[maxIndex].getBasis().getEulerYPR(yaw, pitch, roll);
			return yaw;
			}
		
		/*
		 * Moves the robot to a given position without changing the direction of view.
		 * @param[in] x The x coordinate (in the world frame).
		 * @param[in] y The y coordinate (in the world frame).
		 * @param[in] speed Scale factor of the speed.
		 * @param[in] deleteData If true, all old data of the odometer will be deleted beforehand.
		 * */
		void goTo(double x, double y, double speed = 1.0, bool deleteData = true)
			{
			if (deleteData) odometer_.deleteOldData();
			speed *= 0.5;
			
			// Transform destination into robot frame and scale vector
			tf::Vector3 localDest = odometer_.transforms_.back().inverse()(tf::Vector3(x, y, 0));
			localDest.normalize();
			
			// Move
			ros::Rate loopRate(10);
			double dist = getDistance(x, y);
			tf::Vector3 speedVec;
			while (ros::ok() && dist > 0.03)
				{
				// Set speed according to distance
				double newSpeed = speed * dist * 3.;
				if (newSpeed > speed) newSpeed = speed;
				else if (newSpeed < 0.06) newSpeed = 0.06;
				speedVec = localDest * newSpeed;
				publishVel(speedVec.getX(), speedVec.getY(), 0.0);
				// Update odometer
				try
					{
					update();
					publishOdomData();
					}
				catch(std::exception& e)
					{
					std::cout << e.what() << std::endl;
					}
				catch(...)
					{
					std::cout << "Something unknown went wrong in robotMove::goTo." << std::endl;
					}
				// Update distance
				dist = getDistance(x, y);
				localDest = odometer_.transforms_.back().inverse()(tf::Vector3(x, y, 0));
				localDest.normalize();
				loopRate.sleep();
				}
			// Stop moving
			publishVel(0.0, 0.0, 0.0);
			}
		
		/*
		 * Callback function of destination subscriber.
		 * Sets dest_ to received pose.
		 * @param[in] dest Pointer to received pose.
		 * */
		void destCallback(const geometry_msgs::Pose::ConstPtr& dest)
			{
			double destX = dest->position.x;
			double destY = dest->position.y;
			double currX = odometer_.getOrigin().getX();
			double currY = odometer_.getOrigin().getY();
			// Look towards destination
			lookTowards(destX, destY);
			// Search for area with most features between +-30 degrees
			double bestYaw = searchMaxFeatures(0.5, 0.8);
			double distance = getDistance(destX, destY);
			
			// If distance is further than 1m, go halfway first
			if (distance > 1.0)
				{
				// Get distance to destination and angle to the area with most features to plan path
				double yaw, pitch, roll, alpha;
				odometer_.getBasis().getEulerYPR(yaw, pitch, roll);
				alpha = bestYaw - yaw;
				// Get stopover point
				tf::Vector3 vec(destX - currX, destY - currY, 0.0);
				vec = vec.rotate(tf::Vector3(0, 0, 1), alpha);
				vec *= 0.5/cos(alpha);
				double midX = currX + vec.getX();
				double midY = currY + vec.getY();
				// Turn to area with most features and move to the point in mid distance
				setYaw(bestYaw);
				goTo(midX, midY);
				// Run sBA to optimize Pose
				odometer_.runSBA(50, false);
				// Recursively repeat until distance < 1m
				destCallback(dest);
				}
			else
				{
				setYaw(bestYaw);
				// Move to destination
				goTo(destX, destY);
				// Try to optimize pose again and correct position
				odometer_.runSBA(50, false);
				goTo(destX, destY);
				}
			}
	};
} //End of namespace


int main(int argc, char** argv)
	{
	cv::initModule_nonfree();
	
	//Set SURF feature detector parameters
//	sp_navigation::Node::fDetector->set("hessianThreshold", 100);
//	sp_navigation::Node::fDetector->set("nOctaves", 4);
//	sp_navigation::Node::fDetector->set("upright", true);

	// ORB
	sp_navigation::Node::fDetector->set("nFeatures", 3000);
	
// SIFT
//	sp_navigation::Node::fDetector->set("contrastThreshold", 0.01);
//	sp_navigation::Node::fDetector->set("edgeThreshold", 20);
	
	// HARRIS
//	sp_navigation::Node::fDetector->set("nfeatures", 5000);
//	sp_navigation::Node::fDetector->set("qualityLevel", 0.01);
//	sp_navigation::Node::fDetector->set("minDistance", 2);
//	sp_navigation::Node::fDetector->set("k", 0.001);
	
	// Set cross checking of features to true
	sp_navigation::Node::dMatcher->set("crossCheck", true);
	
	ros::init(argc, argv, "sp_navigation");
	ros::NodeHandle nh;
	std::string stereoNamespace = nh.resolveName("stereo");
	std::string worldFrame = nh.resolveName("world");
	std::string robotFrame = nh.resolveName("base_link");
	std::string cameraFrame = nh.resolveName("camera");
	std::string destTopic = nh.resolveName("setdest");
	
	sp_navigation::StereoSubscriber stereoSub(nh, stereoNamespace);
	sp_navigation::robotMove robot(stereoSub, worldFrame, robotFrame, cameraFrame, "/cmd_vel", destTopic);

	
	ros::Rate loopRate(10);
	ros::Time begin = ros::Time::now();
	double time;
	while (ros::ok())
		{
		try
			{
			time = robot.update();
			robot.publishOdomData();
			}
		catch(std::exception& e)
			{
			std::cout << e.what() << std::endl;
			}
		catch(...)
			{
			std::cout << "Something unknown went wrong." << std::endl;
			}
		std::cout << "Time used:		" << time*1000 << "ms" << std::endl << std::endl;
		loopRate.sleep();
		}
	return 0;
	}
