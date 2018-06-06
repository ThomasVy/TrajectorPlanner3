#include "Trajectory_Planner_finished.h"
#include "ros/ros.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <tf/transform_broadcaster.h>


ros::Publisher pub;
void sendTransform()
{
	static tf::TransformBroadcaster broadcaster;
	broadcaster.sendTransform(
			tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),   //Says the scan is the same position as the baselink.
		ros::Time::now(), "map", "path" ));
}
void publishInfo(const nav_msgs::OccupancyGrid::ConstPtr& msg) //0 is free and 1 is occupied, 0.5 for unknown
{
	std::cout<<"got it"<<std::endl;
	//disect start and goal from msg later on
	Pose start(1876, 1872, 0); //reverse (y,x)
	Pose goal(1635,2071); //reverse (y,x)
	Matrix original((int)msg->info.width, vector<float>((int)msg->info.height));
	for(int i =0 , k=0; i<(int)msg->info.width; i++)
	{
		for(int j=0; j<(int)msg->info.height;j++)
		{
			//make start and end
			original[i][j] = msg->data[k++];
		}
	}
	Image img(original);
	img.insert_borders();
	nav_msgs::Path path;
	static int num =0;
	path.header.seq = num++;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = "path";
	if(img.planner(start, goal))
	{
		std::cout << "Found Path" << std::endl;
		path.poses = img.getBSpline();
	}
	else
	{
		std::cout<<"Could NOT find path"<<std::endl;
	}
	sendTransform();
	pub.publish(path);
	ros::spinOnce();
}
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "TrajectoryNode");
	ros::NodeHandle n;
	pub = n.advertise<nav_msgs::Path>("path", 1000);
	ros::Subscriber sub = n.subscribe("map", 1000, publishInfo);
	ros::spin();
	return 0;
}
