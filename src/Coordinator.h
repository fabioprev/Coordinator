#pragma once

#include "Utils/Timestamp.h"
#include <ros/node_handle.h>
#include <std_msgs/String.h>
#include <Coordinator/TargetChased.h>
#include <PTrackingBridge/TargetEstimations.h>

namespace Coordination
{
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
			ros::Publisher publisherCommandPath;
			ros::Subscriber subscriberTargetChased, subscriberTargetEstimations;
			PTracking::Timestamp lastTaskAssignment;
			CoordinatorState coordinatorState;
		
		public:
			Coordinator();
		
			virtual ~Coordinator();
		
			void exec();
			void updateTargetChased(const Coordinator::TargetChased::ConstPtr& message);
			void updateTargetEstimations(const PTrackingBridge::TargetEstimations::ConstPtr& message);
	};
}
