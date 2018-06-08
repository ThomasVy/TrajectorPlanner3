#include "Trajectory_Planner_finished.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

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
void publishInfo(const nav_msgs::OccupancyGrid::ConstPtr& msg) //0 is free and 1 is occupied, 0.5 for unknown
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
	//disect start and goal from msg later on
	Pose start(grid_x,grid_y, transform.getRotation().getAngle()); //reverse (y,x)
	Pose goal(2151, 1628); //reverse (y,x)
	Matrix original((int)msg->info.height, vector<float>((int)msg->info.width));
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
		path.poses = img.getBSpline(msg);
	}
	else
	{
		ROS_ERROR("Could NOT find path");
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
