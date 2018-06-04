#ifndef TRAJECTORY_PLANNER
#define TRAJECTORY_PLANNER
#include <queue>
#include <math.h>
#include <list>
#include <vector>
#include <cstring>
using namespace std;
const int PRECISION = 200;
const int WALL=255;
const int EMPTY_SPACE =1;
const int RESOLUTION = 1;
const float LENGTH = 3;
const float CURVATURE = 0.25;
//modify cost
//use .pgm files
//ros node
typedef vector< vector<float> > Matrix;

typedef struct Pose{
	double x;
	double y;
	double theta;
	Pose(){}
	Pose(double x, double y, double theta){
		this->x =x;
		this->y =y;
		this->theta =theta;
	}
	Pose(double x, double y)
	{
		this->x =x;
		this->y =y;
		this->theta = 0.0;
	}
	Pose(const Pose & rhs)
	{
		this->x =rhs.x;
		this->y =rhs.y;
		this->theta =rhs.theta;
	}
	Pose& operator=(const Pose & rhs)
	{
		if(this!=&rhs)
		{
			this->x = rhs.x;
			this->y = rhs.y;
			this->theta = rhs.theta;
		}
		return *this;

	}
	static float distanceToGoal(Pose & pose, Pose & goal)
	{
			float dx = goal.x - pose.x;
			float dy = goal.y - pose.y;
			return sqrt(pow(dx,2) + pow(dy,2));
	}
	Pose * endPose(float curvature, float length)
	{
		Pose *pose = 0;
		if(curvature == 0.0)
		{
			float x1, y1;
			x1 = x + length*cos(theta);
			y1 = y + length*sin(theta);
			pose = new Pose(x1, y1, theta);
		}
		else
		{
			float tx = cos(theta);
			float ty = sin(theta);
			float radius = 1.0/curvature;
			float xc = x - radius*ty;
			float yc = y + radius *tx;
			float angle = length / radius;
			float cosa = cos(angle);
			float sina = sin(angle);
			float nx = xc +radius *(cosa * ty +sina * tx);
			float ny = yc + radius * (sina * ty - cosa *tx);
			float ntheta = fmod((theta + angle + M_PI), (2*M_PI)) - M_PI;
			pose = new Pose(nx, ny, ntheta);
		}
		return pose;
	}

}Pose;


class Position{
	private:
		float cost;
		float total_cost;
		Pose pose;
	public:
		Position * prePosition;
		Position(Pose & pose, float total_cost, float cost, Position * prePosition)
		{
			this->pose = pose;
			this->cost=cost;
			this->total_cost = total_cost;
			this->prePosition = prePosition;
		}
		Position(Pose & pose, float cost, Position * prePosition)
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
		vector<Position> getNeighbours (Matrix & walls, Pose & goal)
		{
			std::vector<Position> neighbours;
			for(int i =-1; i<2;i++)
			{
				float curvature = i*CURVATURE;
				Pose* endP = pose.endPose(curvature, LENGTH);
				Pose newPoint = *endP;
				delete endP;
				if(newPoint.x>=0 && newPoint.x<walls.size() && newPoint.y>=0 &&newPoint.y < walls[0].size())
				{
					float space = walls[(int)newPoint.x][(int)newPoint.y];
					if(space != WALL)
					{
						float temp = LENGTH;
						if(i!=0)
							temp =LENGTH*sqrt(2);
						float new_cost = cost + temp;
						float neighbourTotalCost = Pose::distanceToGoal(newPoint, goal) + new_cost + 2*space;
						neighbours.push_back(Position(newPoint, neighbourTotalCost, new_cost, this));
					}
				}
			}
			return neighbours;
		}
		Pose & getPoint()
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
		Matrix b_splineImage;
		Matrix convertedImage;
		Matrix arena;
		Matrix oriImage;
		void dilation(Wall & wall) //need to check direction now
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
									if(wall.direction[4]||wall.direction[3]||(arena[wall.x+i][wall.y+j] + cost_gradient>=255.0))//left and down
										arena[wall.x+i][wall.y+j]= 255.0;
									else
										arena[wall.x+i][wall.y+j] += cost_gradient;
								}
								else if (i ==0 && j< 0 && wall.direction[4]) //down
								{
									if(wall.direction[2]||wall.direction[1]||(arena[wall.x+i][wall.y+j] + cost_gradient>=255.0))//down and left corner
										arena[wall.x+i][wall.y+j] = 255.0;
									else
										arena[wall.x+i][wall.y+j] += cost_gradient;
								}
								else if (i ==0 && j>0 && wall.direction[3])//up
								{
									if(wall.direction[2]||wall.direction[1]||(arena[wall.x+i][wall.y+j] + cost_gradient>=255.0))
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
									if(wall.direction[4]||wall.direction[3]||(arena[wall.x+i][wall.y+j] + cost_gradient>=255.0))
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


	public:
		Image(const Matrix & oriImage)
		{
			this->oriImage = oriImage;
			convertedImage = oriImage;
			for(int i =0; i< convertedImage.size();i++)
			{
				for (int j =0; j< convertedImage[i].size();j++)
				{
					if(convertedImage[i][j])
						convertedImage[i][j] = EMPTY_SPACE;
					else
						convertedImage[i][j] = WALL;
				}
			}
		}
		void insert_borders ()
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
		bool * checkSpace (int i, int j)
		{
			bool * direct = new bool[9];
			for(int t =0;t<9;t++)
			{
				direct[t] =false;
			}
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
		bool planner (Pose & start, Pose & goal)
		{
			std::priority_queue<Position,vector<Position>, std::greater<Position> > openList;
			vector<Position*> closedList;
			list<Position> finalTrail;
			if (arena.size() == 0 ||arena[0].size()==0)
						return false;
			Matrix space(arena.size(), std::vector<float>(arena[0].size()));
			openList.push(Position(start, 0, 0));
			Pose currentPoint;
			while(Pose::distanceToGoal(currentPoint, goal)>1){
				if(openList.empty())
				{
					return false;
				}
				closedList.push_back(new Position(openList.top()));
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
			closedList.push_back(new Position(goal , 0, closedList.back()));
			Position *currentPose = closedList.back();
			while(currentPose!=0)
			{
				finalTrail.push_front(*currentPose);
				currentPose = currentPose->prePosition;
			}
			std::list<Position>::iterator it = finalTrail.begin();
			vector<Pose> points;
			while(it != finalTrail.end())
			{
				Pose point1((*it).getPoint());
				points.push_back(point1);
				it++;
			}
			Bezier(points);
			return true;
		}
		void Bezier (vector<Pose> & points, int num =PRECISION)
		{
			b_splineImage = oriImage;
			int firstnum = 0;
			int endnum = 1;
			double result = (double)(endnum -firstnum)/(num-1);
			vector <double> arr; //t = np.linspace(0, 1, num=num)
			for(int i =0;i<num; i++)
			{
				arr.push_back(firstnum + i*result);
			}
			vector<Pose> curve;
			for(int i =0; i<num;i++)
			{
				curve.push_back(Pose(0.0, 0.0));
			}
			for(int ii =0;ii<points.size();ii++)
			{
				vector<double> berst = Berstein(arr, points.size()-1, ii);
				multipleVectors(curve, berst, points[ii]);
			}
			for(int t=0; t<curve.size();t++)
			{
				Pose point = curve[t];
				b_splineImage[(int)point.x][(int)point.y] = 150;
			}
		}
		void multipleVectors(vector<Pose> & curve, vector<double> & berst, Pose point)
		{
			for(int i =0;i<berst.size();i++)
			{
				double x = curve[i].x + berst[i] *  point.x;
				double y = curve[i].y + berst[i] *  point.y;
				curve[i] = Pose(x,y);
			}
		}
		vector<double> Berstein(vector<double> & arr, int n, int k)
		{
			vector<double> returnVector;
			double coeff = binomialCoeff(n,k);
			for(int i =0 ; i<arr.size();i++)
			{
				returnVector.push_back(coeff*pow(arr[i],k)*pow(1-arr[i], n-k));
			}
			return returnVector;
		}
		double binomialCoeff(int n, int k)
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
		const Matrix & getBSpline ()
		{
			return b_splineImage;
		}
};
#endif
