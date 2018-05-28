#ifndef TRAJECTORY_PLANNER
#define TRAJECTORY_PLANNER
#include <iostream>
#include <string>
#include <queue>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <list>
#include <vector>
using namespace std;
using namespace cv;
const int WALL=254; //B,G,R
const int EMPTY_SPACE =150;
const int RESOLUTION = 4;
//trying to fixed the image translation //works good
class Position{
	private:
		float cost;
		float total_cost;
		Point pose;
	public:
		Position * prePosition;
		Position(Point & pose, float total_cost, float cost, Position * prePosition)
		{
			this->pose = pose;
			this->cost=cost;
			this->total_cost = total_cost;
			this->prePosition = prePosition;
		}
		Position(Point & pose, float cost, Position * prePosition)
		{
			this->pose = pose;
			this->cost=cost;
			this->total_cost = cost;
			this->prePosition = prePosition;
		}
		Position(const Position & rhs)
		{
			this->pose = rhs.pose;
			this->cost = rhs.cost;
			this->total_cost = rhs.total_cost;
			this->prePosition = rhs.prePosition;
		}
		Position(){}
		Position& operator=(const Position & rhs)
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
		bool operator>(Position const& right) const{
			return total_cost > right.total_cost;
		}
		std::vector<Position> getNeighbours (Mat & walls, Point & goal) 
		{
			std::vector<Position> neighbours;
			float diagonal_cost= RESOLUTION*sqrt(2);
			for(int i =-1; i<2;i++)
			{
				for(int j =-1; j<2;j++)
				{
					if(j!=0 || i!=0)
					{
						Point newPoint = Point(i+pose.x,j+pose.y);
						if(newPoint.x>=0 && newPoint.x<walls.rows && newPoint.y>=0 &&newPoint.y < walls.cols)
						{
							Vec3b & space = walls.at<Vec3b>(newPoint.x, newPoint.y);
							if(space[2] != WALL)
							{
								float neighbour_cost =0;
								if(i==j || i+j ==0) //diagonal movement
								{
									neighbour_cost = diagonal_cost;
								}
								else
								{
									neighbour_cost = RESOLUTION;
								}
								float new_cost = cost + neighbour_cost;
								float neighbourTotalCost = Position::distanceToGoal(newPoint, goal) + new_cost + space[1]/2 ;
								neighbours.push_back(Position(newPoint, neighbourTotalCost, new_cost, this));
							}
						}
					}
				}
			}
			return neighbours;
		}
		static float distanceToGoal(Point & pose, Point & goal)
		{
			float dx = goal.x - pose.x;
			float dy = goal.y - pose.y;
			return sqrt(std::pow(dx,2) + std::pow(dy,2));
		}
		const Point & getPoint()
		{
			return pose;
		}
		const float & getCost()
		{
			return cost;
		}
		
};








typedef struct Wall{
	int x;
	int y;
	bool * direction;
	Wall(int x, int y, bool *direction)
	{
		this->x =x;
		this->y =y;
		this->direction =direction;
	}
	~Wall()
	{
		delete[] direction;
	}
		
}Wall;

class Image{
	private:
		String picture;
		Mat image;
		Mat convertedImage;
		Mat arena;
		void dilation(Wall & wall) //need to check direction now
		{
			if(wall.direction[0] == true) //checks if its an outside wall block
			{
				for(int i =-RESOLUTION;i<=RESOLUTION;i++)
				{
					if(wall.x+i >=0 && wall.x+i<convertedImage.rows)
					{
						for(int j =-RESOLUTION; j<=RESOLUTION;j++)
						{
							if(wall.y+j >= 0 && wall.y+j <convertedImage.cols)
							{
								float cost_gradient = (float)40/sqrt(pow(i,2) +pow(j,2));
								if(i<0 && j<0 && wall.direction[8]) //bottom left
								{
									if(arena.at<Vec3b>(wall.x+i, wall.y+j)[1] + cost_gradient>=255.0)
										arena.at<Vec3b>(wall.x+i, wall.y+j)[1] = 255.0;
									else
										arena.at<Vec3b>(wall.x+i, wall.y+j)[1] += cost_gradient;
								}
								else if(i<0 && j>0 && wall.direction[6]) //top left
								{
									if(arena.at<Vec3b>(wall.x+i, wall.y+j)[1] + cost_gradient>=255.0)
										arena.at<Vec3b>(wall.x+i, wall.y+j)[1] = 255.0;
									else
										arena.at<Vec3b>(wall.x+i, wall.y+j)[1] += cost_gradient;
								}
								else if (i<0 && j ==0 && wall.direction[2])//left
								{
									if(arena.at<Vec3b>(wall.x+i, wall.y+j)[1] + cost_gradient>=255.0)
										arena.at<Vec3b>(wall.x+i, wall.y+j)[1] = 255.0;
									else
										arena.at<Vec3b>(wall.x+i, wall.y+j)[1] += cost_gradient;
								}
								else if (i ==0 && j< 0 && wall.direction[4]) //down
								{
									if(arena.at<Vec3b>(wall.x+i, wall.y+j)[1] + cost_gradient>=255.0)
										arena.at<Vec3b>(wall.x+i, wall.y+j)[1] = 255.0;
									else
										arena.at<Vec3b>(wall.x+i, wall.y+j)[1] += cost_gradient;
								}
								else if (i ==0 && j>0 && wall.direction[3])//up
								{
									if(arena.at<Vec3b>(wall.x+i, wall.y+j)[1] + cost_gradient>=255.0)
										arena.at<Vec3b>(wall.x+i, wall.y+j)[1] = 255.0;
									else
										arena.at<Vec3b>(wall.x+i, wall.y+j)[1] += cost_gradient;
								}
								else if (i>0 && j<0 && wall.direction[7])//bottom right
								{
									if(arena.at<Vec3b>(wall.x+i, wall.y+j)[1] + cost_gradient>=255.0)
										arena.at<Vec3b>(wall.x+i, wall.y+j)[1] = 255.0;
									else
										arena.at<Vec3b>(wall.x+i, wall.y+j)[1] += cost_gradient;
								}
								else if (i>0 && j ==0 && wall.direction[1])//right
								{
									if(arena.at<Vec3b>(wall.x+i, wall.y+j)[1] + cost_gradient>=255.0)
										arena.at<Vec3b>(wall.x+i, wall.y+j)[1] = 255.0;
									else
										arena.at<Vec3b>(wall.x+i, wall.y+j)[1] += cost_gradient;
								}
								else if (i>0 && j>0 && wall.direction[5]) //top right
								{
									if(arena.at<Vec3b>(wall.x+i, wall.y+j)[1] + cost_gradient>=255.0)
										arena.at<Vec3b>(wall.x+i, wall.y+j)[1] = 255.0;
									else
										arena.at<Vec3b>(wall.x+i, wall.y+j)[1] += cost_gradient;
								}
							}
						}
					}
				}
			}
						
		}
		
	public:
		Image(const String& picture)
		{
			this->picture =picture;
			
		}
		bool formImage() //gets image from folder. Returns true if found, otherwise false
		{ 
			image =imread(picture, 0);
			if(!image.data)
			{
				std::cout<< "Could not find file" << std::endl;
				return false;
			}
			convertedImage = Mat::zeros(image.size(),CV_8UC3);
			int value = 0;
			for(int i =0; i< image.rows;i++)
			{
				for (int j =0; j< image.cols;j++)
				{ 
					char colour = image.at<uchar>(i,j);
					if(colour)
						convertedImage.at<Vec3b>(i,j)[0]=EMPTY_SPACE;
					else
						convertedImage.at<Vec3b>(i,j)[2]=WALL;
				}
			}
			resize(convertedImage, convertedImage, Size(128, 128), 0, 0, INTER_NEAREST);
			namedWindow( "Display window", CV_WINDOW_FREERATIO |  CV_GUI_EXPANDED);// Create a window for display.
            imshow( "Display window", image );   // Show our image inside it.
            namedWindow("ConvertedImage", CV_WINDOW_FREERATIO |  CV_GUI_EXPANDED);
            imshow("ConvertedImage", convertedImage);
	        return true;
		}
		void insert_borders ()
		{
			arena = convertedImage.clone();
			for(int i =0; i< convertedImage.rows;i++)
			{
				for (int j =0; j< convertedImage.cols;j++)
				{ 
					if(convertedImage.at<Vec3b>(i,j)[2]==WALL)
					{
						Wall wall(i, j, checkSpace(i,j));
						dilation(wall);
					}
				}
			}
			namedWindow("newWindow", CV_WINDOW_FREERATIO |  CV_GUI_EXPANDED);
			imshow("newWindow", arena);
		}		
		bool * checkSpace (int i, int j)
		{
			bool * direct = new bool[9];
			for(int t =0;t<9;t++)
			{
				direct[t] =false;
			}
			if(i+1<convertedImage.rows)
			{
				if(convertedImage.at<Vec3b>(i+1,j)[0] == EMPTY_SPACE)//right
				{
					direct[0] =true;
					direct[1] =true;
				}
				if(j+1<convertedImage.cols && convertedImage.at<Vec3b>(i+1,j+1)[0] == EMPTY_SPACE)//top right
				{
					direct[0] =true;
					direct[5] =true;
				}
					if(j-1 >=0 && convertedImage.at<Vec3b>(i+1,j-1)[0] == EMPTY_SPACE)//bottom right
				{
					direct[0] =true;
					direct[7] =true;
				}
			}
			if(i-1>=0)
			{
				if(convertedImage.at<Vec3b>(i-1,j)[0] == EMPTY_SPACE)//left
				{
					direct[0] =true;
					direct[2] =true;
				}
				if(j+1<convertedImage.cols && convertedImage.at<Vec3b>(i-1, j+1)[0] == EMPTY_SPACE)//top left
				{
					direct[0] =true;
					direct[6] =true;
				}
				if(j-1 >=0 && convertedImage.at<Vec3b>(i-1, j-1)[0] == EMPTY_SPACE)//bottom left
				{
					direct[0] =true;
					direct[8] =true;
				}
			}
			if(convertedImage.at<Vec3b>(i,j+1)[0] == EMPTY_SPACE)//up
			{
				direct[0] =true;
				direct[3] = true;
			}
			if(convertedImage.at<Vec3b>(i,j-1)[0] == EMPTY_SPACE)//down
			{
				direct[0] =true;
				direct[4] = true;
			}
			return direct;
			
		}		 		 
		void planner (Point & start, Point & goal)
		{
			swap(start);
			swap(goal);
			std::priority_queue<Position,vector<Position>, std::greater<Position> > openList;
			vector<Position*> closedList;
			list<Position> finalTrail;
			Mat space = Mat::zeros(arena.size(), CV_8UC3); 
			openList.push(Position(start, 0, NULL));
			Point currentPoint(-100000, -100000);
			while(Position::distanceToGoal(currentPoint, goal)>1){
				if(openList.empty())
					break;
				closedList.push_back(new Position(openList.top())); 
				currentPoint = closedList.back()->getPoint();
				openList.pop();
				if(space.at<Vec3b>(currentPoint.x, currentPoint.y)[1] == 0)
				{
					space.at<Vec3b>(currentPoint.x, currentPoint.y)= Vec3b(closedList.back()->getCost()/25,200, 0);
					std::vector<Position> neighbours = closedList.back()->getNeighbours(arena, goal);
					for(int i =0; i<neighbours.size(); i++)
					{
						openList.push(neighbours[i]);
					}
				}
			}
			Position *currentPose = closedList.back();
			while(currentPose!=NULL)
			{
				finalTrail.push_front(*currentPose);
				currentPose = currentPose->prePosition;
			}
			image.convertTo(image, CV_8UC3);
			Mat outimage(arena.size(), CV_8UC3);
			cv::resize(image, outimage, Size(128,128),0,0, INTER_NEAREST);
			std::list<Position>::iterator it = finalTrail.begin();
			while(it != finalTrail.end())
			{
				const Point & point1 = (*it).getPoint();
				outimage.at<uchar>(point1.x, point1.y)= 150;
				it++;
			}
			outimage.at<uchar>(start.x, start.y) = 10;
			outimage.at<uchar>(goal.x, goal.y)=10;
			namedWindow("Hello", CV_WINDOW_FREERATIO |  CV_GUI_EXPANDED);
			imshow("Hello", space);
			
			namedWindow("Pls?" , CV_WINDOW_FREERATIO |  CV_GUI_EXPANDED);
			imshow("Pls?", outimage);
			
		}
		void swap (Point & point)
		{
			float temp = point.x;
			point.x=point.y;
			point.y=temp;
		}
									

				
};
#endif
