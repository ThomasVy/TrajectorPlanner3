#include "TrajectoryPlanner3.h"

int main()
{
	Point start(4,24);
	Point goal(15,1);
	RESOLUTION =30;
	Image img("path5.png");
	bool imagefound =img.formImage();
	if(imagefound)
	{
		waitKey();
		img.insert_borders();
		waitKey();
		//img.planner(start, goal);
		waitKey(0);
		
	}
}
