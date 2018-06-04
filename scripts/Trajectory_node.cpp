#include "Trajectory_Planner_finished.h"
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Header.h"

ros::Publisher pub;
bool doPlanning(Pose &start, Pose &goal, Mat & original)
{
	Image img(original);
	waitKey();
	img.insert_borders();
	waitKey();
	bool pathFound = img.planner(start, goal);
	waitKey(0);
	return pathFound;
}
void publishInfo(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	std::cout<<"got it"<<std::endl;
	//disect start and goal from msg later on
	Pose start(277, 540, M_PI);
	Pose goal(892, 73, 0);
	cv::Mat original((int)msg->info.width, (int)msg->info.height, CV_8UC1);
	for(int i =0 , k=0; i<(int)msg->info.width; i++)
	{
		for(int j=0; j<(int)msg->info.height;j++)
		{
			original.at<uchar>(i,j) = msg->data.data()[k]*-255;
			k++;
		}
	}
	doPlanning(start, goal, original);
}
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "TrajectoryNode");
	ros::NodeHandle n;
	pub = n.advertise<nav_msgs::OccupancyGrid>("/PictureTopic", 1000);
	ros::Subscriber sub = n.subscribe("/imageTopic", 1000, publishInfo);
	ros::spin();
	return 0;
}
