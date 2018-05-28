from math import sin, cos, pi,tan, atan2,fmod,sqrt
import numpy as np
from pylab import *
from itertools import groupby
from operator import itemgetter
import matplotlib.pyplot as plt
from scipy import interpolate
from numpy.linalg import inv
from heapq import heappush, heappop
from scipy import misc
from scipy import ndimage
from scipy.special import binom


def show_pose(x,y,theta,gx,gy,d):
	ax = plt.axes()
	ax.arrow(x, y, d*cos(theta), d*sin(theta), head_width=10, head_length=40, fc='k', ec='k')
	plt.text(x, y, r'S')
	plt.text(gx, gy, r'G')
	
def Bernstein(n, k):
    coeff = binom(n, k)
    def _bpoly(x):
		return coeff * x ** k * (1 - x) ** (n - k)
    return _bpoly

def Bezier(points, num=200):
    N = len(points) #length of the list
    t = np.linspace(0, 1, num=num) #creates interval between 0 and 1 with 200 samples
    print t
    print type(t[0])
    curve = np.zeros((num, 2)) # creates a 200 rows by 2 column array filled with zeros
    for ii in range(N):
		print type(Bernstein(N - 1, ii)(t)[0])
		curve += np.outer(Bernstein(N - 1, ii)(t), points[ii])#multiplying two vectors
    return curve
    
def end_pose(start_pose, curvature, length):
	x, y, theta = start_pose
	if curvature == 0.0:
		x += length * cos(theta)
		y += length * sin(theta)
		return (x, y, theta)
	else:
		tx = cos(theta)
		ty = sin(theta)
		radius = 1.0/curvature
		xc = x - radius * ty 
		yc = y + radius * tx
		angle = length / radius
		cosa = cos(angle)
		sina = sin(angle)
		nx = xc + radius * (cosa * ty + sina * tx)
		ny = yc + radius * (sina * ty - cosa * tx)
		ntheta = (theta + angle + pi) % (2*pi) - pi
		return (nx, ny, ntheta)
            
def insert_border(ospace,d):
	struct = ndimage.generate_binary_structure(2, 2)
	arena=ospace
	for i in range(1,d):
		b=ndimage.binary_dilation(arena, structure=struct).astype(arena.dtype)
		arena=arena+b
	#print arena
	plt.imshow(arena,interpolation='nearest')
	show()
	return arena

		
def distanceTogoal(pose,goal):
	dx=goal[0]-pose[0]
	dy=goal[1]-pose[1]
	r=sqrt(dx**2+dy**2)
	return r
		
def getimage(d):
	m= misc.imread('path10.png')
	ospace=np.zeros(m.shape[0]*m.shape[1]).reshape(m.shape[0],m.shape[1])
	for i in xrange(m.shape[0]):
		for j in xrange(m.shape[1]):
			ospace[i,j]=-float(m[i,j][0])*1/255+1
	#plt.imshow(m,interpolation='nearest')
	#show()
	return m,ospace
				

def planner(start_pose,goal,ospace,d):
	l=3.0 #resolution
	c=1/10.0
	neighbour=[[c,l],[0.0,l],[-c,l]] # pose,cost
	front=[]
	visited=[]
	space=np.zeros_like(ospace)
	heappush(front,(0.00001,0.00001,start_pose,(None,None,None)))
	pose=start_pose
	path={}
	r=100000.0
	#for i in range(0,2600):
	while(r>5):
		total_cost,cost,pose,ppose=heappop(front)
		if space[int(pose[1]),int(pose[0])]==0.0:
			visited.append((cost,pose,ppose))
			space[int(pose[1]),int(pose[0])]=cost
			key=(pose[0],pose[1])
			path[key]=(ppose[0],ppose[1])
			for n in neighbour:
				p=end_pose(pose, n[0], l)
				new_cost,new_pose,pre_pose=cost+n[1],(p[0],p[1],p[2]),(pose[0],pose[1],pose[2])				
				if (new_pose[0]>=0 and new_pose[0]<space.shape[1]) and (new_pose[1]>=0 and new_pose[1]<space.shape[0]):
					if ospace[int(new_pose[1]),int(new_pose[0])]<h:
						r=distanceTogoal(new_pose,goal)
						total_cost=r+new_cost+ospace[int(new_pose[1]),int(new_pose[0])]
						heappush(front,(total_cost,new_cost,new_pose,pre_pose))						
	pose=(pre_pose[0],pre_pose[1])
	trajectory=[]
	while pose[0]!=None:
		trajectory.append(pose)
		pose=path[pose]
	trajectory.reverse()
	plt.imshow(space,interpolation='nearest')
	show()
	return visited,space,trajectory				
	
if __name__ == '__main__':
	x,y=[],[]
	start_pose=(280,560,pi)	
	#start=(600,300)
	goal=(850,40)
	#goal=(700,100)
	d=30
	h=3
	m,ospace=getimage(d)
	arena=insert_border(ospace,d)
	visited,space,trajectory=planner(start_pose,goal,arena,d)
	for p in trajectory:
		x.append(p[0])
		y.append(p[1])
	xp, yp = Bezier(list(zip(x, y))).T #transpose the list
	plt.imshow(m,interpolation='nearest')
	plt.scatter(start_pose[0],start_pose[1],c='r', s=40)
	plt.scatter(goal[0],goal[1],c='g', s=40)
	plot(x,y,color="m",label='curve_segments')
	plt.scatter(xp,yp,label='B-spline')
	show_pose(start_pose[0],start_pose[1],start_pose[2],goal[0],goal[1],80)#labels for Start and goal
	legend(loc='upper left')
	
	show()
