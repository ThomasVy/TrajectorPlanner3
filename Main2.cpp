#include "Trajectory_Planner2.h"

int main()
{
	Pose start(277, 540, M_PI);
	Pose goal(892, 73, 0);
	Image img("path10.png");
	bool imagefound =img.formImage();
	if(imagefound)
	{
		waitKey();
		img.insert_borders();
		waitKey();
		img.planner(start, goal);
		waitKey();
		
	}
}
