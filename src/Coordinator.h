#pragma once

#include "Utils/Timestamp.h"
#include <ros/node_handle.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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
		
		inline std::string getCoordinatorState() const
		{
			if (coordinatorState == Chasing) return "Chasing";
			else if (coordinatorState == Idle) return "Idle";
			else if (coordinatorState == Patroling) return "Patroling";
			else throw new std::runtime_error("Unknown coordinator state!");
		}
		
	public:
		Coordinator();
		
		virtual ~Coordinator();
		
		void exec();
		void updateTargetChased(const std_msgs::String::ConstPtr& message);
		void updateTargetEstimations(const PTrackingBridge::TargetEstimations::ConstPtr& message);
		void updateRobotPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message);
};
