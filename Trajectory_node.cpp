#include "Trajectory_Planner_finished.h"
#include <ros/ros.h>

ros::Publisher pub;
void publishInfo()
{
	Pose start(277, 540, M_PI);
	Pose goal(892, 73, 0);
	Image img("path10.png");
	bool imagefound =img.formImage();
	if(imagefound)
	{
		waitKey();
		img.insert_borders();
		waitKey();
		img.planner(start, goal);
		waitKey();
		
	}
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "Trajectory Node");
	ros::NodeHandle nh;
	ros::Rate::Rate(10);
	pub = n.advertise<>
	ros::Subscriber sub = nh.subscribe(
}
