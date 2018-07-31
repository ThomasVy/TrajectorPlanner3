Author:Thomas Vy
Email: thomas.vy@ucalgary.ca
Date: June 12, 2018

Credit to Gayan Brahmanage for the formulation of the path planner first in his python code. Thanks Gayan!

Highly recommended that your run rviz to see the path planned 

how to install:
	1. clone trajectory_planner to your catkin_workspace
	2. change your goal to match the occupancy grid location 
	2. do catkin_make
	3. Run your robot or clone dataset_reader from git@github.com:ThomasVy/datareader.git 
	4. Run your gmapping with dataset_reader ro your robot
	6. Run trajectory_planner by typing "rosrun trajectory_planner traject"


Implementation:
	The implmentation of the trajectory planner is created by it recieving an occupancy grid given by gmapping over ros and turning the occupancy grid into a 2d matrix. The 2d matrix is then converted to a map that contains values for empty space, walls, and unknown parameters. The trajectory planner then uses the converted map to make a cost map. The planning of the map starts with the current position of the robot as the start and the goal on the cost map (These can be any points as you desire as long as the points are in an empty space). The planner then uses an A* algorithm to plan for the path of the robot. The planner starts with the robot's current position then calculates the next valid positions(straight, left curve, and right curve) and calculates their respective costs and places them in a priority queue. The planner then places the old position into a vector and grabs the next lowest cost from the priority queue and tries to find valid the neighbours for that position. If the priority queue becomes empty, then there is no valid path to the goal. The planner continues that process until the current position to goal is less than 5 cells. Then we grab the point that found its way to the goal first and back track it. The path is then published with the correct information along with the transform of the path to the map. 


There is also a b-spline curve implmentation to smooth out the path that is created. If you would like to see the smooth version of the path just go to "publishingPath" branch of this repository and do the same steps as in this branch.
	
