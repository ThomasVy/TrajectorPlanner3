#include "trajectory_planner/Constants.hpp"
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "nav_msgs/OccupancyGrid.h"
#include <iostream>
using namespace std;
/*
	Uses ros to subscibe to the /map and publishes a path to /path
	Author:Thomas Vy
	Date: June 12 2018
	Email: thomas.vy@ucalgary.ca
*/
ros::Publisher pub; //publisher to path
tf::TransformListener * plr; //transform listener
Pose finalGoal(2001, 1999); //the goal position
//sends the transform of the path relative to the map
void sendTransform()
{
	static tf::TransformBroadcaster broadcaster;
	broadcaster.sendTransform(
			tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.0)),   // map and path do need need to be tranformed.
				ros::Time::now(), "map", "path" ));
}
//publishes the path to /path
void publishInfo(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	tf::StampedTransform transform;
  try{
    plr->lookupTransform("/map", "/base_link",
                             msg->info.map_load_time , transform); //looks for the transform between map and the robot
  }
  catch (tf::TransformException ex){
		return; //returns if there is an error in the transformation
  }
	int grid_x = (transform.getOrigin().x() - (int)msg->info.origin.position.x) / msg->info.resolution; //changes the robot real position to the grid
  int grid_y = (transform.getOrigin().y() - (int)msg->info.origin.position.y) / msg->info.resolution;//changes the robot real position to the grid
	ROS_INFO("current pose: %d, %d", grid_x, grid_y);
	int firstx =(int)msg->info.width, firsty =(int)msg->info.height , lastx =-1,lasty =-1; //reduces the size of the map.
	Pose start(grid_x,grid_y, tf::getYaw(transform.getRotation())); //the start position (the robot's current position)
	Pose goal = finalGoal;
	matrix original((int)msg->info.height, vector<double>((int)msg->info.width)); //the original map in a 2d vector
	for(int y =0 , k=0; y<(int)msg->info.height; y++)
	{
		for(int x=0; x<(int)msg->info.width ;x++)
		{
			int value =  msg->data[k++];
			//make start and end
			original[x][y] = value;
			if(value!=-1) //checks for empty or wall space in the map
			{
				if(x<firstx)
					firstx = x;
				if(y<firsty)
					firsty = y;
				if(x>lastx)
					lastx = x;
				if(y>lasty)
					lasty =y;
			}
		}
	}
	if(lastx<firstx || lasty<firsty) //If there are no bounds.
		return;
	Pose first(firstx, firsty);
	Pose last(lastx, lasty);
	Image img(original, first, last); //creates a converted image bsaed off original map
	img.insert_borders(); //creates cost map
	if(original[goal.x][goal.y]!=0) // Cannot reach the goal position at the moment
	{
		Pose goal = img.findNearestFreeSpace(finalGoal, start);
	}
	nav_msgs::Path path;
	static int num =0;
	path.header.seq = num++;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = "path";
	if(img.planner(start, goal)) //plans the path if it find a path it publishes it
	{
		ROS_INFO("Found a path");
		path.poses = img.getPath();
		for(int i =0; i<path.poses.size();i++)
		{
			path.poses[i].pose.position.x = (path.poses[i].pose.position.x*msg->info.resolution+msg->info.origin.position.x);
			path.poses[i].pose.position.y = (path.poses[i].pose.position.y*msg->info.resolution+msg->info.origin.position.y);
		}
	}
	else
	{
		ROS_ERROR("Could NOT find a path");
		return;
	}
	sendTransform();
	pub.publish(path);
	if(goal == finalGoal)
	{
		ROS_INFO("Reach destination with path./n Now exiting program...");
		exit(0);
	}
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "TrajectoryNode"); //init the node
	ros::NodeHandle n;
	pub = n.advertise<nav_msgs::Path>("path", 1000);// publishes to path
	ros::Subscriber sub = n.subscribe("map", 1, publishInfo);// subscribes to map
	tf::TransformListener listener; //tf listener
	plr = &listener;
	ros::spin();
	return 0;
}
