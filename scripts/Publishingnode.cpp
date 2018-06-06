#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "std_msgs/Header.h"
#include <string>
#include <iostream>
#include <fstream>
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "PublishingNode");
	ros::NodeHandle n;
	ros::Rate loop_rate(0.1);
	ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("/imageTopic", 1000);
	//read image
	nav_msgs::OccupancyGrid map;
	nav_msgs::MapMetaData mapData;
	std::string imageName = "/home/thomas/catkin_ws/src/dataset_reader/src/map.pgm";
	cv::Mat file = cv::imread(imageName, 0);
	if(!file.data)
	{
		std::cout<<"failed"<<std::endl;
		exit(1);
	}
	cv::threshold(file, file, 100, 255,cv::THRESH_BINARY);
	int length = file.rows*file.cols;
	std::vector<signed char> array(length);
	std::cout<<"reading "<<length<<" characters"<<std::endl;
	for(int i =0; i<file.rows;i++)
	{
		for(int j=0; j<file.cols;j++)
		{
			map.data.push_back(file.at<uchar>(i,j));
		}
	}
	mapData.map_load_time = ros::Time::now();
	mapData.resolution = 4;
	mapData.width = file.rows;
	mapData.height = file.cols;
	map.info = mapData;
	int count =1;
	while(ros::ok()&&count<3)
	{
		std_msgs::Header header;
		header.seq = count++;
		header.stamp =  ros::Time::now();
		header.frame_id = "0";
		map.header = header;
		pub.publish(map);
		ros::spinOnce();
		std::cout<<"sent"<<std::endl;
		loop_rate.sleep();
	}
	std::cout<<"Shutting down"<<std::endl;
	return 0;
}
