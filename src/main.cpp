#include "Coordinator.h"
#include <ros/ros.h>

int main (int argc, char** argv)
{
	ros::init(argc,argv,"Coordinator");
	
	Coordinator coordinator;
	
	while (ros::ok())
	{
		ros::spinOnce();
		
		coordinator.exec();
		
		usleep(100e3);
	}
	
	return 0;
}
