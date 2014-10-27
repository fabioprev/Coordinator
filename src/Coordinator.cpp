#include "Coordinator.h"
#include <geometry_msgs/Point32.h>
#include <vector>

using namespace std;
using geometry_msgs::Point32;
using std_msgs::String;
using PTracking::Timestamp;
using PTrackingBridge::TargetEstimations;

#define TASK_ASSIGNMENT_TIME 1000	/// In milliseconds.
#define MINIMUM_DISTANCE 1			/// In meters.

#define ERR(x)  cerr << "\033[22;31;1m" << x << "\033[0m";
#define WARN(x) cerr << "\033[22;33;1m" << x << "\033[0m";
#define INFO(x) cerr << "\033[22;37;1m" << x << "\033[0m";
#define DEBUG(x)  cerr << "\033[22;34;1m" << x << "\033[0m";

namespace Coordination
{
	Coordinator::Coordinator() : nodeHandle("~"), coordinatorState(Idle)
	{
		subscriberTargetChased = nodeHandle.subscribe("targetChased",1024,&Coordinator::updateTargetChased,this);
		subscriberTargetEstimations = nodeHandle.subscribe("targetEstimations",1024,&Coordinator::updateTargetEstimations,this);
		
		publisherCommandPath = nodeHandle.advertise<String>("PointsListString",1);
		
		/// Waiting that the WaypointNavigation node has started.
		usleep(5e6);
	}
	
	Coordinator::~Coordinator() {;}
	
	void Coordinator::exec()
	{
		if (coordinatorState == Chasing)
		{
			if ((Timestamp() - lastTaskAssignment).getMs() > TASK_ASSIGNMENT_TIME) coordinatorState = Idle;
		}
		
		if (coordinatorState == Idle)
		{
			String message;
			
			message.data = "Path_0";
			
			publisherCommandPath.publish(message);
			
			coordinatorState = Patroling;
		}
	}
	
	void Coordinator::updateTargetChased(const String::ConstPtr& message)
	{
		
	}
	
	void Coordinator::updateTargetEstimations(const TargetEstimations::ConstPtr& message)
	{
		static Point32 currentChasingTarget;
		static unsigned long pathCounter = 1;
		
		stringstream s;
		float theta;
		int counter;
		
		if ((Timestamp() - lastTaskAssignment).getMs() < TASK_ASSIGNMENT_TIME) return;
		
		lastTaskAssignment.setToNow();
		
		ERR("**************************************************" << endl);
		
		counter = 0;
		
		for (std::vector<int>::const_iterator it = message->identities.begin(); it != message->identities.end(); ++it, ++counter)
		{
			const Point32& position = message->positions.at(counter);
			
			INFO("[" << *it << "] -> (" << position.x << "," << position.y << ")" << endl);
		}
		
		if (message->positions.size() == 0) return;
		
		const Point32& chasingTarget = message->positions.at(0);
		
		if ((fabs(currentChasingTarget.x - chasingTarget.x) > MINIMUM_DISTANCE) ||
			(fabs(currentChasingTarget.y - chasingTarget.y) > MINIMUM_DISTANCE))
		{
			theta = atan2(chasingTarget.y,chasingTarget.x);
			
			s << "Path_-" << pathCounter++ << " 0 " << chasingTarget.x << " " << chasingTarget.y << " " << (theta * 180 / 3.1415);
			
			String dataToBePublished;
			
			dataToBePublished.data = s.str();
			
			publisherCommandPath.publish(dataToBePublished);
			
			coordinatorState = Chasing;
			currentChasingTarget = chasingTarget;
		}
	}
}
