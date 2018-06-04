#include "Trajectory_Planner_finished.h"
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"


ros::Publisher pub;
void publishInfo(const nav_msgs::OccupancyGrid::ConstPtr& msg) //0 is free and 1 is occupied, 0.5 for unknown
{
	std::cout<<"got it"<<std::endl;
	//disect start and goal from msg later on
	Pose start(540, 277, M_PI); //reverse (y,x)
	Pose goal(76, 222); //reverse (y,x)
	Matrix original((int)msg->info.width, vector<float>((int)msg->info.height));
	for(int i =0 , k=0; i<(int)msg->info.width; i++)
	{
		for(int j=0; j<(int)msg->info.height;j++)
		{
			original[i][j] = -msg->data[k++];
		}
	}
	Image img(original);
	img.insert_borders();
	nav_msgs::Path path;
	if(img.planner(start, goal))
	{
		std::cout << "Found Path" << std::endl;
		std::vector<Pose> t = img.getBSpline();
	}
	else{
		std::cout<<("Could NOT find path")<<std::endl;
	}
	pub.publish(path);
}
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "TrajectoryNode");
	ros::NodeHandle n;
	pub = n.advertise<nav_msgs::Path>("/PictureTopic", 1000);
	ros::Subscriber sub = n.subscribe("/imageTopic", 1000, publishInfo);
	ros::spin();
	return 0;
}
