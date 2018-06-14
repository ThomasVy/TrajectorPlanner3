#include <math.h>
#include <cstring>
#include "trajectory_planner/Constants.hpp"
/*
	all the definitions for classes in Trajectory_Constants
	Author:Thomas Vy
	Date: June 12 2018
	email:thomas.vy@ucalgary.ca
	*/
float distanceToGoal(Pose & pose, Pose & goal)
{
			float dx = goal.x - pose.x;
			float dy = goal.y - pose.y;
			return sqrt(pow(dx,2) + pow(dy,2));
}

//Pose class definitons
Pose::Pose(const Pose & rhs)
{
	this->x =rhs.x;
	this->y =rhs.y;
	this->radian =rhs.radian;
}
Pose& Pose::operator=(const Pose & rhs)
{
	if(this!=&rhs)
	{
		this->x = rhs.x;
		this->y = rhs.y;
		this->radian = rhs.radian;
	}
	return *this;

}
Pose Pose::endPose(float curvature, float length)
{
	Pose pose;
	if(curvature == 0.0)//going sraight
	{
		float x1, y1;
		x1 = x + length*cos(radian);
		y1 = y + length*sin(radian);
		pose = Pose(x1, y1, radian);
	}
	else//moving to left or right curve
	{
		float tx = cos(radian);
		float ty = sin(radian);
		float radius = 1.0/curvature;
		float xc = x - radius*ty;
		float yc = y + radius *tx;
		float angle = length / radius;
		float cosa = cos(angle);
		float sina = sin(angle);
		float nx = xc +radius *(cosa * ty +sina * tx);
		float ny = yc + radius * (sina * ty - cosa *tx);
		float nradian = fmod((radian + angle + M_PI), (2*M_PI)) - M_PI;
		pose = Pose(nx, ny, nradian);
	}
	return pose;
}

//Position class Definitions
Position::Position(Pose & pose, float total_cost, float cost, Position * prePosition)
{
	this->pose = pose;
	this->cost=cost;
	this->total_cost = total_cost;
	this->prePosition = prePosition;
}
Position::Position(Pose & pose, float cost, Position * prePosition)
{
	this->pose = pose;
	this->cost=cost;
	this->total_cost = cost;
	this->prePosition = prePosition;
}
Position::Position(const Position & rhs)
{
	this->pose = rhs.pose;
	this->cost = rhs.cost;
	this->total_cost = rhs.total_cost;
	this->prePosition = rhs.prePosition;
}
Position& Position::operator=(const Position & rhs)
{
	if(this!=&rhs)
	{
		this->cost = rhs.cost;
		this->total_cost = rhs.total_cost;
		this->pose = rhs.pose;
		this->prePosition = rhs.prePosition;
	}
	return *this;
}
listOfPositions Position::getNeighbours (matrix & walls, Pose & goal)
{
	listOfPositions neighbours;
	for(int i =-1; i<2;i++)//checks left, straight, and right movement.
	{
		float curvature = i*CURVATURE;
		Pose newPoint = pose.endPose(curvature, LENGTH); //gets new potenial position
		if(newPoint.x>=0 && newPoint.x<walls.size() && newPoint.y>=0 &&newPoint.y < walls[0].size())//checks if within range
		{
			if(checkNeighbour(pose, newPoint, walls))//checks if the new potenial spot is valid
			{
					float space = walls[(int)newPoint.x][(int)newPoint.y];
					float temp = LENGTH;
					if(i!=0)
						temp =LENGTH*sqrt(2);
					float new_cost = cost + temp;
					float neighbourTotalCost = distanceToGoal(newPoint, goal) + new_cost + space;// calculates the total cost of moving to that spot
					neighbours.push_back(Position(newPoint, neighbourTotalCost, new_cost, this));// pushes it in the list of potential positions to move
			}
		}
	}
	return neighbours;
}
bool Position::checkNeighbour (Pose & current, Pose & next, matrix & walls)
{
	int diffx = current.x - next.x;
	int diffy = current.y - next.y;
	int incrementx = 1, incrementy =1;
	if(diffx<0)
		incrementx =-1;
	if(diffy<0)
		incrementy =-1;
	int y,x;
	for(x=0;abs(x)<=abs(diffx)&&diffy>1; x+=incrementx)//checks the area around a diagonal movement
	{
		for (y=0; abs(y)<=abs(diffy); y+=incrementy)
		{
			if(walls[current.x+x][current.y+y]==UNKNOWN||walls[current.x+x][current.y+y] == WALL)
				return false;
		}
	}
	while(abs(y)<=abs(diffy))//checks the area around vertical movement
	{
		for(int i=-LENGTH/2;i<LENGTH/2;i++) //gives it width
		{
			if(walls[current.x+i][current.y+y]==UNKNOWN||walls[current.x+i][current.y+y] == WALL)
				return false;
		}
		y+=incrementy;
	}
	while(abs(x)<=abs(diffx))//checks the area around horizatontal movement
	{
		for(int i=-LENGTH/2;i<LENGTH/2;i++)//gives it width
		{
			if(walls[current.x+x][current.y+i]==UNKNOWN||walls[current.x+x][current.y+i] == WALL)
				return false;
		}
		x+=incrementx;
	}
	return true;
}

//Image class definitions
void Image::dilation(Wall & wall)
{
	if(wall.direction[0] == true) //checks if its an outside wall block
	{
		for(int i =-RESOLUTION;i<=RESOLUTION;i++)
		{
			if(wall.x+i >=0 && wall.x+i<convertedImage.size())//checks if valid x-range-
			{
				for(int j =-RESOLUTION; j<=RESOLUTION;j++)
				{
					if(wall.y+j >= 0 && wall.y+j <convertedImage[0].size())//checks if valid y-range
					{
						if(arena[wall.x+i][wall.y+j]!=WALL &&arena[wall.x+i][wall.y+j]!=UNKNOWN)//makes sure the current position is not a wall or unknown
						{
							float cost_gradient = (float)70/sqrt(pow(i,2) +pow(j,2));
							if(i<0 && j<0 && wall.direction[8]) //bottom left
							{
								if(arena[wall.x+i][wall.y+j] + cost_gradient>=255.0)
									arena[wall.x+i][wall.y+j] = 255.0;
								else
									arena[wall.x+i][wall.y+j] += cost_gradient;
							}
							else if(i<0 && j>0 && wall.direction[6]) //top left
							{
								if(arena[wall.x+i][wall.y+j]+ cost_gradient>=255.0)
									arena[wall.x+i][wall.y+j] = 255.0;
								else
									arena[wall.x+i][wall.y+j] += cost_gradient;
							}
							else if (i<0 && j ==0 && wall.direction[2])//left
							{
								if((arena[wall.x+i][wall.y+j] + cost_gradient>=255.0))//left and down
									arena[wall.x+i][wall.y+j]= 255.0;
								else
									arena[wall.x+i][wall.y+j] += cost_gradient;
							}
							else if (i ==0 && j< 0 && wall.direction[4]) //down
							{
								if((arena[wall.x+i][wall.y+j] + cost_gradient>=255.0))//down and left corner
									arena[wall.x+i][wall.y+j] = 255.0;
								else
									arena[wall.x+i][wall.y+j] += cost_gradient;
							}
							else if (i ==0 && j>0 && wall.direction[3])//up
							{
								if((arena[wall.x+i][wall.y+j] + cost_gradient>=255.0))
									arena[wall.x+i][wall.y+j] = 255.0;
								else
									arena[wall.x+i][wall.y+j] += cost_gradient;
							}
							else if (i>0 && j<0 && wall.direction[7])//bottom right
							{
								if(arena[wall.x+i][wall.y+j]+ cost_gradient>=255.0)
									arena[wall.x+i][wall.y+j] = 255.0;
								else
									arena[wall.x+i][wall.y+j]+= cost_gradient;
							}
							else if (i>0 && j ==0 && wall.direction[1])//right
							{
								if((arena[wall.x+i][wall.y+j] + cost_gradient>=255.0))
									arena[wall.x+i][wall.y+j] = 255.0;
								else
									arena[wall.x+i][wall.y+j] += cost_gradient;
							}
							else if (i>0 && j>0 && wall.direction[5]) //top right
							{
								if(arena[wall.x+i][wall.y+j] + cost_gradient>=255.0)
									arena[wall.x+i][wall.y+j] = 255.0;
								else
									arena[wall.x+i][wall.y+j] += cost_gradient;
							}
						}
					}
				}
			}
		}
	}

}


Image::Image(const matrix & oriImage)
{
	convertedImage = oriImage;
	for(int i =0; i< convertedImage.size();i++)
	{
		for (int j =0; j< convertedImage[i].size();j++)
		{
			if(convertedImage[i][j] == 0)
				convertedImage[i][j] = EMPTY_SPACE;
			else if(convertedImage[i][j] == -1)
				convertedImage[i][j] =UNKNOWN;
			else
				convertedImage[i][j] = WALL;
		}
	}
}
void Image::insert_borders ()
{
	arena = convertedImage;
	for(int i =0; i< convertedImage.size();i++)
	{
		for (int j =0; j< convertedImage[i].size();j++)
		{
			if(convertedImage[i][j]==WALL)
			{
				Wall wall(i, j, checkSpace(i,j));
				dilation(wall);
			}
		}
	}
}
vector<bool> Image::checkSpace (int i, int j)
{
	vector<bool> direct(9, false);
	if(i+1<convertedImage.size())
	{
		if(convertedImage[i+1][j] == EMPTY_SPACE)//right space
		{
			direct[0] =true;
			direct[1] =true;
		}
		if(j+1<convertedImage[i].size() && convertedImage[i+1][j+1]== EMPTY_SPACE)//top right space
		{
			direct[0] =true;
			direct[5] =true;
		}
			if(j-1 >=0 && convertedImage[i+1][j-1] == EMPTY_SPACE)//bottom right space
		{
			direct[0] =true;
			direct[7] =true;
		}
	}
	if(i-1>=0)
	{
		if(convertedImage[i-1][j] == EMPTY_SPACE)//left space
		{
			direct[0] =true;
			direct[2] =true;
		}
		if(j+1<convertedImage[i].size() && convertedImage[i-1][j+1]== EMPTY_SPACE)//top left space
		{
			direct[0] =true;
			direct[6] =true;
		}
		if(j-1 >=0 && convertedImage[i-1][j-1] == EMPTY_SPACE)//bottom left space
		{
			direct[0] =true;
			direct[8] =true;
		}
		if(convertedImage[i][j+1] == EMPTY_SPACE)//up space
		{
			direct[0] =true;
			direct[3] = true;
		}
		if(convertedImage[i][j-1] == EMPTY_SPACE)//down space
		{
			direct[0] =true;
			direct[4] = true;
		}
	}

	return direct;

}
bool Image::planner (Pose & start, Pose & goal)
{
	positionPriorityQueue openList; //the priority_queue to hold the potenial moves
	vectorOfPointers closedList;// holds the already checked positions
	if (arena.size() == 0 ||arena[0].size()==0) //checks if the size is valid
				return false;
	matrix space(arena.size(), std::vector<double>(arena[0].size())); //the grid to check if the space has been visited already
	openList.push(Position(start, 0, 0)); //pushes the start postion first
	Pose currentPoint;// the current point being checked
	while(distanceToGoal(currentPoint, goal)>5){ //keep checking if the current point is greater than 5 cells away from the goal
		if(openList.empty())//check if there are no moves left in the priority queue
		{
			return false;
		}
		closedList.push_back(std::unique_ptr<Position>(new Position(openList.top())));//grabs the lowest cost in the priority queue
		currentPoint = closedList.back()->pose;//gets the point from priority queue
		openList.pop();// gets rid of the lowest cost from the priority queue
		if(space[currentPoint.x][currentPoint.y]== 0)//checks if the space is not been visited yet
		{
			space[currentPoint.x][currentPoint.y]= 1;
			std::vector<Position> neighbours = closedList.back()->getNeighbours(arena, goal); // gets the neighbours to the current point
			for(int i =0; i<neighbours.size(); i++)
			{
				openList.push(neighbours[i]); //push all the neighbours to the currnt point to the priority queue
			}
		}
	}
	if(closedList.empty()) //if the list is empty put the start and end point to the list.
	{
			closedList.push_back(std::unique_ptr<Position>(new Position(start, 0, 0)));
	}
	closedList.push_back(std::unique_ptr<Position>(new Position(goal , 0, closedList.back().get()))); // push the goal to the list
	Position *currentPose = closedList.back().get();
	poseVector points;
	while(currentPose!=0) //gets the path that made it to the goal first
	{
		points.insert(points.begin(), currentPose->pose);
		currentPose = currentPose->prePosition;
	}
	path = pathMessage(points.size());
	for(int i =0; i<points.size();i++)
	{
		path[i].header.seq = i+1;
		path[i].header.stamp = ros::Time::now();
		path[i].header.frame_id = "path";
		path[i].pose.position.x = points[i].x;
		path[i].pose.position.y = points[i].y;
	}
	//Bezier(points); //makes the bpline curve
	return true;
}
void Image::Bezier (poseVector & points, int num)
{

	int firstnum = 0;
	int endnum = 1;
	double result = (double)(endnum -firstnum)/(num-1);
	doubleVector arr (num); //t = np.linspace(0, 1, num=num)
	for(int i =0;i<num; i++)
	{
		arr[i] =firstnum + i*result;
	}
	path = pathMessage(num);
	for(int ii =0;ii<points.size();ii++)
	{
		doubleVector berst = Berstein(arr, points.size()-1, ii);
		multipleVectors(berst, points[ii]);
	}
	for(int i =0;i<path.size();i++)
	{
		path[i].header.seq = i;
		path[i].header.stamp = ros::Time::now();
		path[i].header.frame_id = "path";
	}
}

void Image::multipleVectors(doubleVector & berst, Pose point)
{
	for(int i =0;i<berst.size();i++)
	{
		path[i].pose.position.x = path[i].pose.position.x + berst[i] *  point.x;
		path[i].pose.position.y = path[i].pose.position.y + berst[i] *  point.y;
	}
}

doubleVector Image::Berstein(doubleVector & arr, int n, int k)
{
 	doubleVector returnVector;
	double coeff = binomialCoeff(n,k);
	for(int i =0 ; i<arr.size();i++)
	{
		returnVector.push_back(coeff*pow(arr[i],k)*pow(1-arr[i], n-k));
	}
	return returnVector;
}

double Image::binomialCoeff(int n, int k)
{
	double C[k+1];
	memset(C, 0, sizeof(C));

	C[0] = 1;

	for (int i = 1; i <= n; i++)
	{
		for (int j = min(i, k); j > 0; j--)
			C[j] = C[j] + C[j-1];
	}
	return C[k];

}

const pathMessage & Image::getPath ()
{
	return path;
}
