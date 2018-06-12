#include "trajectory_planner/Trajectory_Definitions.hpp"
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "nav_msgs/OccupancyGrid.h"

ros::Publisher pub;
tf::TransformListener * plr;
void sendTransform()
{
	static tf::TransformBroadcaster broadcaster;
	broadcaster.sendTransform(
			tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.0)),   //Says the scan is the same position as the baselink.
		ros::Time::now(), "map", "path" ));
}
void publishInfo(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	tf::StampedTransform transform;
  try{
    plr->lookupTransform("/map", "/base_link",
                             msg->info.map_load_time , transform);
  }
  catch (tf::TransformException ex){
		return;
  }
	int grid_x = (transform.getOrigin().x() - (int)msg->info.origin.position.x) / msg->info.resolution;
  int	grid_y = (transform.getOrigin().y() - (int)msg->info.origin.position.y) / msg->info.resolution;


	Pose start(grid_x,grid_y, transform.getRotation().getAngle());
	Pose goal(2151, 1628);
	matrix original((int)msg->info.height, vector<double>((int)msg->info.width));
	for(int y =0 , k=0; y<(int)msg->info.height; y++)
	{
		for(int x=0; x<(int)msg->info.width ;x++)
		{
			//make start and end
			original[x][y] = msg->data[k++];
		}
	}
	if(original[goal.x][goal.y]!=0 ||original[start.x][start.y]!=0)
	{
		return;
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
		ROS_INFO("Found Path");
		path.poses = img.getBSpline();
		for(int i =0; i<path.poses.size();i++)
		{
			path.poses[i].pose.position.x = (path.poses[i].pose.position.x*msg->info.resolution+msg->info.origin.position.x);
			path.poses[i].pose.position.y = (path.poses[i].pose.position.y*msg->info.resolution+msg->info.origin.position.y);
		}
	}
	else
	{
		ROS_ERROR("Could NOT find path");
		return;
	}
	sendTransform();
	pub.publish(path);
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "TrajectoryNode");
	ros::NodeHandle n;
	pub = n.advertise<nav_msgs::Path>("path", 1000);
	ros::Subscriber sub = n.subscribe("map", 1000, publishInfo);
	tf::TransformListener listener;
	plr = &listener;
	ros::spin();
	return 0;
}
