#include "Trajectory_Planner_finished.h"
/*#include "ros/ros.h"

ros::Publisher pub;
void publishInfo()
{
}
*/
int main(int argc, char ** argv)
{
	Pose start(122, 454, M_PI);
	Pose goal(647, 169, 0);
	Image img("path8.png");
	bool imagefound =img.formImage();
	if(imagefound)
	{
		waitKey();
		img.insert_borders();
		waitKey();
		img.planner(start, goal);
		waitKey();
		
	}
	/*ros::init(argc, argv, "Trajectory Node");
	ros::NodeHandle nh;
	ros::Rate::Rate(10);
	pub = n.advertise<>
	ros::Subscriber sub = nh.subscribe(
	*/
}
