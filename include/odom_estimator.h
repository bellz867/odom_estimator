#ifndef ODOMESTIMATOR_H
#define ODOMESTIMATOR_H

#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <helper_functions.h>

struct OdomEstimator
{
	std::string name;
	ros::NodeHandle nh;
	ros::Subscriber arucoPoseSub;
	ros::Subscriber poseSub;
	ros::Subscriber cameraInfoSub;
	ros::Publisher odomPub;
	ros::Publisher velPub;
	ros::Time tLast;
	bool firstMocap;
	bool initialized;
	Eigen::Matrix<float,13,1> xHat;
	Eigen::Matrix<float,13,13> P;
	Eigen::Matrix<float,12,12> Q;
	Eigen::Matrix<float,7,7> R;
	Eigen::Matrix<float,13,13> F;
	Eigen::Matrix<float,13,12> L;
	Eigen::Matrix<float,7,13> H;
	Eigen::Matrix<float,13,7> HT;
	Eigen::Vector4f qww;
    Eigen::Matrix<float,7,1> z;

	OdomEstimator(std::string nameInit);

	void ArucoposeCB(const geometry_msgs::PoseStamped::ConstPtr& msg);

	void MocapPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);

	void cameraInfoCB(const sensor_msgs::CameraInfoConstPtr& msg);

	void publish_state();

	Eigen::Matrix4f partialqDotq(Eigen::Vector3f w);

	Eigen::Matrix<float,4,3> partialqDotw(Eigen::Vector4f q);

	Eigen::Matrix<float,13,1> xDot(Eigen::Matrix<float,13,1> x);
};

#endif
