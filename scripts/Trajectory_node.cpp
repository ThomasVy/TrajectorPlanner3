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
	if(img.planner(start, goal))
	{
		std::vector<Pose> t = img.getBSpline();
		cv::Mat show = cv::Mat::zeros(cv::Size((int)msg->info.height, (int)msg->info.width), CV_8UC3);
		for(int i =0 ; i<t.size();i++)
		{
			Pose point = t[i];
			show.at<cv::Vec3b>(point.x, point.y) = cv::Vec3b(0, 0, 255);
		}
		/*for(int i =0; i< t.size(); i++)
		{
			for(int j=0; j<t[0].size();j++)
			{
				if(t[i][j] == PATH)
				{
					show.at<cv::Vec3b>(i,j) =cv::Vec3b(0,0,255);//The trail
				}
				else
				{
					show.at<cv::Vec3b>(i,j) = cv::Vec3b(t[i][j]*255, t[i][j]*255, t[i][j]*255) ;
				}
			}
		}*/
		cv::namedWindow("Display",CV_WINDOW_FREERATIO | CV_GUI_EXPANDED);
		cv::imshow("Display", show);
		std::cout <<"Found path"<<std::endl;
		cv::waitKey();
	}
	else{
		std::cout<<("Could NOT find path")<<std::endl;
	}
	exit(1);
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
