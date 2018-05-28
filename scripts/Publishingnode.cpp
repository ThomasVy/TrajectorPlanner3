#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Header.h"
#include <string>
#include <iostream>
#include <fstream>
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "PublishingNode");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("/imageTopic", 1000);
	//read image 
	nav_msgs::OccupancyGrid map;
	nav_msgs::MapMetaData mapData;
	std::string imageName;
	std::cout<<"Pls enter the image name"<<std::endl;
	std::cin>>imageName;
	std::ifstream file(imageName.c_str(), std::ios::binary| std::ios::in);
	if(imageName == "QUIT"||!file)
	{
		std::cout<<"Goodbye"<<std::endl;
		exit(1);
	}
	file.seekg(0, file.end);
	int length = file.tellg();
	file.seekg(0, file.beg);
	std::cout<<"reading "<<length<<" characters"<<std::endl;
	std::vector<signed char, std::allocator<signed char> > array(length);
	file.read((char*)&array[0], length);
	map.data = array;
	mapData.map_load_time = ros::Time::now();
	mapData.resolution = 23;
	mapData.width = 480;
	mapData.height = 360;
	geometry_msgs::Point point;
	point.x =0;
	point.y =0;
	point.z =0;
	geometry_msgs::Quaternion quat;
	quat.x =0;
	quat.y =0;
	quat.z=0;
	quat.w=0;
	mapData.origin.position = point;
	mapData.origin.orientation = quat;
	map.info = mapData;
	int count =1;
	while(ros::ok())
	{
		std_msgs::Header header;
		header.seq = count++;
		header.stamp =  ros::Time::now();
		header.frame_id = "0";
		map.header = header;
		pub.publish(map);
		ros::spinOnce();
		loop_rate.sleep();
	}
	cout<<"Shutting down"<<endl;
	return 0;
}
