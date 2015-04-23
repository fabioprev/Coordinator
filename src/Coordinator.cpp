#include "Coordinator.h"
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <limits>
#include <vector>

using namespace std;
using geometry_msgs::Quaternion;
using geometry_msgs::Point32;
using nav_msgs::Odometry;
using std_msgs::String;
using PTracking::Timestamp;
using PTrackingBridge::TargetEstimations;

static const float MINIMUM_ANGLE		= 30.0;		/// In degrees.
static const float MINIMUM_DISTANCE		= 1;		/// In meters.
static const int TASK_ASSIGNMENT_TIME	= 1000;		/// In milliseconds.

#define ERR(x)  cerr << "\033[22;31;1m" << x << "\033[0m";
#define WARN(x) cerr << "\033[22;33;1m" << x << "\033[0m";
#define INFO(x) cerr << "\033[22;37;1m" << x << "\033[0m";
#define DEBUG(x)  cerr << "\033[22;34;1m" << x << "\033[0m";

Coordinator::Coordinator() : nodeHandle("~"), coordinatorState(Idle)
{
	nodeHandle.getParam("agentId",agentId);
	
	subscriberTargetChased = nodeHandle.subscribe("targetChased",1024,&Coordinator::updateTargetChased,this);
	subscriberTargetEstimations = nodeHandle.subscribe("targetEstimations",1024,&Coordinator::updateTargetEstimations,this);
	subscriberRobotPose = nodeHandle.subscribe("base_pose_ground_truth",1024,&Coordinator::updateRobotPose,this);
	
	publisherCommandPath = nodeHandle.advertise<String>("PointsListString",1);
	publisherTargetChased = nodeHandle.advertise<String>("targetChased",1);
	
	/// Waiting that the WaypointNavigation node has started.
	usleep(5e6);
}

Coordinator::~Coordinator() {;}

void Coordinator::exec()
{
	//ERR("State: " << getCoordinatorState() << endl);
	
	if (coordinatorState == Chasing)
	{
		if ((Timestamp() - lastTaskAssignment).getMs() > (1.5 * TASK_ASSIGNMENT_TIME))
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

void Coordinator::updateRobotPose(const Odometry::ConstPtr& message)
{
	double roll, pitch, yaw;
	
	const Quaternion& q = message->pose.pose.orientation;
	
	tf::Matrix3x3(tf::Quaternion(q.x,q.y,q.z,q.w)).getRPY(roll,pitch,yaw);
	
	mutex.lock();
	
	robotPoseX = message->pose.pose.position.x;
	robotPoseY = message->pose.pose.position.y;
	robotPoseTheta = (yaw * 180) / M_PI;
	
	mutex.unlock();
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
	
	//INFO("Agent (" << agentId << ") chasing target [" << currentChasingTarget.first << "]" << endl);
	
	float theta;
	
	mutex.lock();
	
	theta = (atan2(currentChasingTarget.second.y - robotPoseY,currentChasingTarget.second.x - robotPoseX) * 180.0) / M_PI;
	
	mutex.unlock();
	
	if ((fabs(currentChasingTarget.second.x - chasingTarget.x) > MINIMUM_DISTANCE) ||
		(fabs(currentChasingTarget.second.y - chasingTarget.y) > MINIMUM_DISTANCE) ||
		(fabs(theta - robotPoseTheta) > MINIMUM_ANGLE))
	{
		chasingTarget.x = currentChasingTarget.second.x;
		chasingTarget.y = currentChasingTarget.second.y;
		
		s.str("");
		s.clear();
		
		if ((fabs(theta - robotPoseTheta) > MINIMUM_ANGLE) && (fabs(robotPoseX - chasingTarget.x) <= MINIMUM_DISTANCE) && (fabs(robotPoseY - chasingTarget.y) <= MINIMUM_DISTANCE))
		{
			s << "Path_-" << pathCounter++ << " 0 " << robotPoseX << " " << robotPoseY << " " << theta;
		}
		else s << "Path_-" << pathCounter++ << " 0 " << chasingTarget.x << " " << chasingTarget.y << " " << theta;
		
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
