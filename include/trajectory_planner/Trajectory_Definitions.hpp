#ifndef TRAJECTORY_PLANNER
#define TRAJECTORY_PLANNER
#include <queue>
#include <math.h>
#include <list>
#include <cstring>
#include <memory>
#include "trajectory_planner/Constants.hpp"

float distanceToGoal(Pose & pose, Pose & goal)
{
			float dx = goal.x - pose.x;
			float dy = goal.y - pose.y;
			return sqrt(pow(dx,2) + pow(dy,2));
}

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
	if(curvature == 0.0)
	{
		float x1, y1;
		x1 = x + length*cos(radian);
		y1 = y + length*sin(radian);
		pose = Pose(x1, y1, radian);
	}
	else
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
	for(int i =-1; i<2;i++)
	{
		float curvature = i*CURVATURE;
		Pose newPoint = pose.endPose(curvature, LENGTH);
		if(newPoint.x>=0 && newPoint.x<walls.size() && newPoint.y>=0 &&newPoint.y < walls[0].size())
		{
			if(checkSpot(pose, newPoint, walls))
			{
					float space = walls[(int)newPoint.x][(int)newPoint.y];
					float temp = LENGTH;
					if(i!=0)
						temp =LENGTH*sqrt(2);
					float new_cost = cost + temp;
					float neighbourTotalCost = distanceToGoal(newPoint, goal) + new_cost + 2*space;
					neighbours.push_back(Position(newPoint, neighbourTotalCost, new_cost, this));
			}
		}
	}
	return neighbours;
}
bool Position::checkSpot (Pose & current, Pose & next, matrix & walls)
{
	int diffx = current.x - next.x;
	int diffy = current.y - next.y;
	int incrementx = 1, incrementy =1;
	if(diffx<0)
		incrementx =-1;
	if(diffy<0)
		incrementy =-1;
	int y =0;
	for(int x =0;abs(x)<=abs(diffx); x+=incrementx)
	{
		for (y=0; abs(y)<=abs(diffy); y+=incrementy)
		{
			if(walls[current.x+x][current.y+y]==UNKNOWN||walls[current.x+x][current.y+y] == WALL)
				return false;
		}
	}
	while(abs(y)<=abs(diffy))
	{
		if(walls[current.x][current.y+y]==UNKNOWN||walls[current.x][current.y+y] == WALL)
			return false;
		y+=incrementy;
	}
	return true;
}

void Image::dilation(Wall & wall) //need to check direction now
{
	if(wall.direction[0] == true) //checks if its an outside wall block
	{
		for(int i =-RESOLUTION;i<=RESOLUTION;i++)
		{
			if(wall.x+i >=0 && wall.x+i<convertedImage.size())
			{
				for(int j =-RESOLUTION; j<=RESOLUTION;j++)
				{
					if(wall.y+j >= 0 && wall.y+j <convertedImage[0].size())
					{
						if(arena[wall.x+i][wall.y+j]!=WALL &&arena[wall.x+i][wall.y+j]!=UNKNOWN)
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
				if(convertedImage[i+1][j] == EMPTY_SPACE)//right
				{
					direct[0] =true;
					direct[1] =true;
				}
				if(j+1<convertedImage[i].size() && convertedImage[i+1][j+1]== EMPTY_SPACE)//top right
				{
					direct[0] =true;
					direct[5] =true;
				}
					if(j-1 >=0 && convertedImage[i+1][j-1] == EMPTY_SPACE)//bottom right
				{
					direct[0] =true;
					direct[7] =true;
				}
			}
			if(i-1>=0)
			{
				if(convertedImage[i-1][j] == EMPTY_SPACE)//left
				{
					direct[0] =true;
					direct[2] =true;
				}
				if(j+1<convertedImage[i].size() && convertedImage[i-1][j+1]== EMPTY_SPACE)//top left
				{
					direct[0] =true;
					direct[6] =true;
				}
				if(j-1 >=0 && convertedImage[i-1][j-1] == EMPTY_SPACE)//bottom left
				{
					direct[0] =true;
					direct[8] =true;
				}
			}
			if(convertedImage[i][j+1] == EMPTY_SPACE)//up
			{
				direct[0] =true;
				direct[3] = true;
			}
			if(convertedImage[i][j-1] == EMPTY_SPACE)//down
			{
				direct[0] =true;
				direct[4] = true;
			}
			return direct;

		}
		bool Image::planner (Pose & start, Pose & goal)
		{
			positionPriorityQueue openList;
			vectorOfPointers closedList;
			positionLinkedList finalTrail;
			if (arena.size() == 0 ||arena[0].size()==0)
						return false;
			matrix space(arena.size(), std::vector<double>(arena[0].size()));
			openList.push(Position(start, 0, 0));
			Pose currentPoint;
			while(distanceToGoal(currentPoint, goal)>5){
				if(openList.empty())
				{
					return false;
				}
				closedList.push_back(std::unique_ptr<Position>(new Position(openList.top())));
				currentPoint = closedList.back()->getPoint();
				openList.pop();
				if(space[currentPoint.x][currentPoint.y]== 0)
				{
					space[currentPoint.x][currentPoint.y]= closedList.back()->getCost();
					std::vector<Position> neighbours = closedList.back()->getNeighbours(arena, goal);
					for(int i =0; i<neighbours.size(); i++)
					{
						openList.push(neighbours[i]);
					}
				}
			}
			if(closedList.empty())
			{
					closedList.push_back(std::unique_ptr<Position>(new Position(start, 0, 0)));
			}
			closedList.push_back(std::unique_ptr<Position>(new Position(goal , 0, closedList.back().get())));
			Position *currentPose = closedList.back().get();
			while(currentPose!=0)
			{
				finalTrail.push_front(*currentPose);
				currentPose = currentPose->prePosition;
			}
			positionLinkedList::iterator it = finalTrail.begin();
			poseVector points;
			while(it != finalTrail.end())
			{
				Pose point1((*it).getPoint());
				points.push_back(point1);
				it++;
			}
			Bezier(points);
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
			b_splineImage = pathMessage(num);
			for(int ii =0;ii<points.size();ii++)
			{
				doubleVector berst = Berstein(arr, points.size()-1, ii);
				multipleVectors(berst, points[ii]);
			}
			for(int i =0;i<b_splineImage.size();i++)
			{
				b_splineImage[i].header.seq = i;
				b_splineImage[i].header.stamp = ros::Time::now();
				b_splineImage[i].header.frame_id = "path";
			}
		}

		void Image::multipleVectors(doubleVector & berst, Pose point)
		{
			for(int i =0;i<berst.size();i++)
			{
				b_splineImage[i].pose.position.x = b_splineImage[i].pose.position.x + berst[i] *  point.x;
				b_splineImage[i].pose.position.y = b_splineImage[i].pose.position.y + berst[i] *  point.y;
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

		const pathMessage & Image::getBSpline ()
		{
			return b_splineImage;
		}
#endif
