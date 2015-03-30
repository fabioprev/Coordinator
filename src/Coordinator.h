#pragma once

#include "Utils/Timestamp.h"
#include <ros/node_handle.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <PTrackingBridge/TargetEstimations.h>

class Coordinator
{
	private:
		enum CoordinatorState
		{
			Chasing = 0,
			Idle,
			Patroling
		};
		
		ros::NodeHandle nodeHandle;
		ros::Publisher publisherCommandPath, publisherTargetChased;
		ros::Subscriber subscriberRobotPose, subscriberTargetChased, subscriberTargetEstimations;
		std::pair<int,geometry_msgs::Point32> currentChasingTarget;
		boost::mutex mutex;
		PTracking::Timestamp lastTaskAssignment;
		geometry_msgs::Point32 chasingTarget;
		std::string targetChased;
		CoordinatorState coordinatorState;
		double robotPoseX, robotPoseY, robotPoseTheta;
		int agentId;
		
	public:
		Coordinator();
		
		virtual ~Coordinator();
		
		void exec();
		void updateTargetChased(const std_msgs::String::ConstPtr& message);
		void updateTargetEstimations(const PTrackingBridge::TargetEstimations::ConstPtr& message);
		void updateRobotPose(const nav_msgs::Odometry::ConstPtr& message);
};
