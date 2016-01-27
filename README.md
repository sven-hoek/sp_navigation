# sp_navigation
This is a ROS package including a basic visual odometry system to navigate a robot to a given destination. It grabs rectified stereo images and the corresponding camera info to calculate the robot's pose. In the current configuration the robot uses a simple proportional controller like method to control its movements. Furthermore, it searches for feature rich areas close to the destination and instead of moving directly towards the destination it moves towards the feature rich area to keep it in its field of view. When the half distance to the destination has been reached, it runs a bundle adjustment algorithm over the gathered data, again searches for a feature rich area and moves to the next stopover point. When the distance to the destination is below a set threshold, the robot will move directly towards the destination.

The pose of the camera in the robot has to be published to /tf in order to correctly navigate the robot. The launch files included in this package already start a static tf broadcaster that has to be adjusted or replaced by a different publishing method. Also the camera node and topic need to be adjusted accordingly.

Right now the package is designed to navigate a ground robot with 3 DOF but can be adjusted to fit different vehicles.

(Note that the 'node' struct in this package does not represent a ROS node but a camera position at a certain time.
