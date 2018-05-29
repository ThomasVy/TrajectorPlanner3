#include "Trajectory_Planner_finished.h"

int main()
{
	Pose start(277, 540, M_PI);
	Pose goal(892, 73, 0);
	Mat image = imread("path10.png",0);
	if(image.data)
	{
		threshold(image, image, 100, 255,cv::THRESH_BINARY); 
		Image img(image);
		waitKey();
		img.insert_borders();
		waitKey();
		img.planner(start, goal);
		waitKey();
		
	}
}
