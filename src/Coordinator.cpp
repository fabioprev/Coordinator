#include "Coordinator.h"
#include <limits>
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

Coordinator::Coordinator() : nodeHandle("~"), coordinatorState(Idle)
{
	nodeHandle.getParam("agentId",agentId);
	
	subscriberTargetChased = nodeHandle.subscribe("targetChased",1024,&Coordinator::updateTargetChased,this);
	subscriberTargetEstimations = nodeHandle.subscribe("targetEstimations",1024,&Coordinator::updateTargetEstimations,this);
	
	publisherCommandPath = nodeHandle.advertise<String>("PointsListString",1);
	publisherTargetChased = nodeHandle.advertise<String>("targetChased",1);
	
	/// Waiting that the WaypointNavigation node has started.
	usleep(5e6);
}

Coordinator::~Coordinator() {;}

void Coordinator::exec()
{
	if (coordinatorState == Chasing)
	{
		if ((Timestamp() - lastTaskAssignment).getMs() > TASK_ASSIGNMENT_TIME)
		{
			String message;
			
			message.data = "";
			
			publisherTargetChased.publish(message);
			
			coordinatorState = Idle;
		}
	}
	
	if (coordinatorState == Idle)
	{
		chasingTarget.x = FLT_MAX;
		chasingTarget.y = FLT_MAX;
		
		String message;
		
		message.data = "Path_0";
		
		publisherCommandPath.publish(message);
		
		coordinatorState = Patroling;
	}
}

void Coordinator::updateTargetChased(const String::ConstPtr& message)
{
	mutex.lock();
	
	targetChased = message->data;
	
	mutex.unlock();
}

void Coordinator::updateTargetEstimations(const TargetEstimations::ConstPtr& message)
{
	static unsigned long pathCounter = 1;
	
	vector<int> targetChasedTeam;
	stringstream s;
	float theta;
	int counter;
	
	if ((Timestamp() - lastTaskAssignment).getMs() < TASK_ASSIGNMENT_TIME) return;
	
	lastTaskAssignment.setToNow();
	
	if (message->positions.size() == 0) return;
	
	mutex.lock();
	
	s << targetChased;
	
	mutex.unlock();
	
	while (s.good())
	{
		if (s.eof()) break;
		
		int identity;
		
		s >> identity;
		
		targetChasedTeam.push_back(identity);
	}
	
	counter = 0;
	
	for (vector<int>::const_iterator it = message->identities.begin(); it != message->identities.end(); ++it, ++counter)
	{
		if (find(targetChasedTeam.begin(),targetChasedTeam.end(),currentChasingTarget.first) != targetChasedTeam.end())
		{
			currentChasingTarget.first = *it;
			currentChasingTarget.second = message->positions.at(counter);
			
			break;
		}
		
		if (find(targetChasedTeam.begin(),targetChasedTeam.end(),*it) == targetChasedTeam.end())
		{
			currentChasingTarget.first = *it;
			currentChasingTarget.second = message->positions.at(counter);
		}
	}
	
	INFO("Agent (" << agentId << ") chasing target [" << currentChasingTarget.first << "]" << endl);
	
	if ((fabs(currentChasingTarget.second.x - chasingTarget.x) > MINIMUM_DISTANCE) ||
		(fabs(currentChasingTarget.second.y - chasingTarget.y) > MINIMUM_DISTANCE))
	{
		chasingTarget.x = currentChasingTarget.second.x;
		chasingTarget.y = currentChasingTarget.second.y;
		
		theta = atan2(chasingTarget.y,chasingTarget.x);
		
		s.str("");
		s.clear();
		
		s << "Path_-" << pathCounter++ << " 0 " << chasingTarget.x << " " << chasingTarget.y << " " << (theta * 180 / 3.1415);
		
		String dataToBePublished;
		
		dataToBePublished.data = s.str();
		
		publisherCommandPath.publish(dataToBePublished);
		
		coordinatorState = Chasing;
		
		currentChasingTarget.first = message->identities.at(0);
		currentChasingTarget.second = chasingTarget;
		
		String targetChased;
		
		s.str("");
		s.clear();
		
		s << currentChasingTarget.first;
		
		targetChased.data = s.str();
		
		publisherTargetChased.publish(targetChased);
	}
}
