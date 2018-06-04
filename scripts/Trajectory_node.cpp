#include "Trajectory_Planner_finished.h"
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Header.h"

ros::Publisher pub;
void publishInfo(const nav_msgs::OccupancyGrid::ConstPtr& msg) //0 is free and 1 is occupied, 0.5 for unknown
{
	std::cout<<"got it"<<std::endl;
	//disect start and goal from msg later on
	Pose start(24, 4, 0);
	Pose goal(1,15, 0);
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
	if(img.planner(start, goal))
	{
		Matrix t = img.getBSpline();
		cv::Mat show(t.size(), t[0].size(), CV_8UC3);
		for(int i =0; i< t.size(); i++)
		{
			for(int j=0; j<t[0].size();j++)
			{
				if(t[i][j] == 150)
				{
					show.at<cv::Vec3b>(i,j) =cv::Vec3b(0,0,150);
				}
				else
				{
					show.at<uchar>(i,j) = t[i][j];
				}
			}
		}
		cv::namedWindow("Display",CV_WINDOW_FREERATIO | CV_GUI_EXPANDED);
		cv::imshow("Display", show);
		std::cout << ("Could find path")<<std::endl;
		cv::waitKey();
	}
	else{
		std::cout<<("Could NOT find path")<<std::endl;
		cv::Mat show(original.size(), original[0].size(), CV_8UC1);
		for(int i =0; i< original.size(); i++)
		{
			for(int j=0; j<original[0].size();j++)
			{
				{
					show.at<uchar>(i,j) = original[i][j];
				}
			}
		}
		show.at<uchar>(goal.x,goal.y) = 150;
		show.at<uchar>(start.x,start.y) = 150;
		cv::namedWindow("Display",CV_WINDOW_FREERATIO | CV_GUI_EXPANDED);
		cv::imshow("Display", show);
		cv::waitKey();
		exit(1);
	}
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
