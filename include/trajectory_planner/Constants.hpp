#ifndef TRAJECTCONST
#define TRAJECTCONST
#include <vector>
#include "geometry_msgs/PoseStamped.h"
/**
  The constants for the trajectory planner
  Author: Thomas Vy(thomas.vy@ucalgary.ca)
  Date: June 12, 2018
**/
using namespace std;
class Image;
class Position;
struct Wall;
struct Pose;

const int PRECISION = 100; //The number of dots for the bspline curve
const int UNKNOWN =-2; //The unknown space value
const int WALL=-1; //The wall space value
const int EMPTY_SPACE =1; //The empty space value
const int RESOLUTION = 10; //The distance from wall to calculate(for cost)
const float LENGTH = 2; //The distance between points(higher number = fast computation but inaccurate path)
const float CURVATURE = 0.2; //The curvature the robot takes

typedef vector< vector<double> > matrix; //Holds the map cells
typedef vector<geometry_msgs::PoseStamped> pathMessage;//The path message
typedef vector<Position> listOfPositions; //list of positions
typedef priority_queue<Position,vector<Position>, std::greater<Position> > positionPriorityQueue; //the priority queue
typedef vector<unique_ptr<Position> > vectorOfPointers; //vector of Pointer
typedef list<Position> positionLinkedList; //Linked list for position
typedef vector<Pose> poseVector; //vector of poses
typedef vector<double> doubleVector; //vector of doubles

typedef struct Wall{
	int x;
	int y;
	std::vector<bool> direction;
	Wall(int x, int y, vector<bool> direction):x(x), y(y), direction(direction){};
}Wall;


typedef struct Pose{
	double x;
	double y;
	double radian;
	Pose():x(0),y(0),radian(0){};
	Pose(double x, double y, double radian):x(x),y(y),radian(radian) {};
	Pose(double x, double y):x(x),y(y),radian(0){};
	Pose(const Pose & rhs);
	Pose& operator=(const Pose & rhs);
	Pose endPose(float curvature, float length);

}Pose;

class Position{
	private:
		float cost;
		float total_cost;
		Pose pose;
	public:
		Position * prePosition;
		Position(Pose & pose, float total_cost, float cost, Position * prePosition);
		Position(Pose & pose, float cost, Position * prePosition);
		Position(const Position & rhs);
		Position():cost(0),total_cost(0){}
		Position& operator=(const Position & rhs);
		bool operator>(Position const& right) const {return total_cost > right.total_cost;}
		listOfPositions getNeighbours (matrix & walls, Pose & goal);
		bool checkSpot (Pose & current, Pose & next, matrix & walls);
		Pose & getPoint(){return pose;}
		const float & getCost(){return cost;}
};
		
class Image{
	private:
		pathMessage b_splineImage;
		matrix convertedImage;
		matrix arena;
		void dilation(Wall & wall);
	public:
		Image(const matrix & oriImage);
		void insert_borders ();
		vector<bool> checkSpace (int i, int j);
		bool planner (Pose & start, Pose & goal);
		void Bezier (poseVector & points, int num =PRECISION);
		void multipleVectors(doubleVector & berst, Pose point);
		doubleVector Berstein(doubleVector & arr, int n, int k);
		double binomialCoeff(int n, int k);
		const pathMessage & getBSpline ();
};
#endif
