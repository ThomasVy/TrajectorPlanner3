#include "Trajectory_Planner.h"

int main()
{
	Point start(16 ,97);
	Point goal(121,51);
	Image img("path5.png");
	bool imagefound =img.formImage();
	if(imagefound)
	{
		waitKey();
		img.insert_borders();
		waitKey();
		img.planner(start, goal);
		waitKey(0);
		
	}
}
