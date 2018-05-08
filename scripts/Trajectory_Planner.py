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
from scipy.special import binom

           
def insert_border(ospace,d):
	shape=ospace.shape
	arena=np.zeros_like(ospace)
	for i in xrange(shape[0]):
		for j in xrange(shape[1]):
			if ospace[i,j]==d:
				for k in xrange(1,d):
					if j+k<shape[1]:
						arena[i,j+k]=max(arena[i,j+k],float(d-k),ospace[i,j+k])
					if j-k>=0:
						arena[i,j-k]=max(arena[i,j-k],float(d-k),ospace[i,j-k])
					if i+k<shape[0]:
						arena[i+k,j]=max(arena[i+k,j],float(d-k),ospace[i+k,j])
					if i-k>0:
						arena[i-k,j]=max(arena[i-k,j],float(d-k),ospace[i-k,j])
	plt.imshow(arena,interpolation='nearest')
	show()
	return arena
		
def distanceTogoal(pose,goal):
	dx=goal[0]-pose[0]
	dy=goal[1]-pose[1]
	r=sqrt(dx**2+dy**2)
	return r
		
def getimage(d):
	m= misc.imread('path5.png')
	ospace=np.zeros(m.shape[0]*m.shape[1]).reshape(m.shape[0],m.shape[1])
	for i in xrange(m.shape[0]):
		for j in xrange(m.shape[1]):
			ospace[i,j]=-float(m[i,j][0])*d/255.0+d
	#plt.imshow(ospace,interpolation='nearest')
	#show()
	return m,ospace
	
def planner1(start,goal,ospace):
	r=1.0 #resolution
	neighbours=[[r,0,r],[0,r,r],[-r,0,r],[0,-r,r],[r,r,r*sqrt(2)],[-r,r,r*sqrt(2)],[-r,-r,r*sqrt(2)],[r,-r,r*sqrt(2)]] # x,y,cost
	front=[]
	visited=[]
	space=np.zeros_like(ospace)
	heappush(front,(0.00001,start))
	pose=start
	print front
	while(pose!=goal):
		cost,pose=heappop(front)
		if space[int(pose[1]),int(pose[0])]==0.0:
			visited.append((cost,pose))
			space[int(pose[1]),int(pose[0])]=cost
			
			for n in neighbours:
				new_cost,new_pose=cost+n[2],(pose[0]+n[0],pose[1]+n[1])
				if (new_pose[0]>=0 and new_pose[0]<space.shape[1]) and (new_pose[1]>=0 and new_pose[1]<space.shape[0]):
					if ospace[int(new_pose[1]),int(new_pose[0])]==255.0:
						heappush(front,(new_cost,new_pose))
	return visited,space
	
def planner2(start,goal,ospace):
	r=1.0 #resolution
	neighbours=[[r,0,r],[0,r,r],[-r,0,r],[0,-r,r],[r,r,r*sqrt(2)],[-r,r,r*sqrt(2)],[-r,-r,r*sqrt(2)],[r,-r,r*sqrt(2)]] # x,y,cost
	front=[]
	visited=[]
	space=np.zeros_like(ospace)
	heappush(front,(0.00001,start,None))
	pose=start
	path={}
	while(pose!=goal):
		cost,pose,ppose=heappop(front)
		if space[pose[1],pose[0]]==0.0:
			visited.append((cost,pose,ppose))
			space[pose[1],pose[0]]=cost
			key=pose
			path[key]=ppose
			for n in neighbours:
				new_cost,new_pose,pre_pose=cost+n[2],(pose[0]+n[0],pose[1]+n[1]),(pose[0],pose[1])
				if (new_pose[0]>=0 and new_pose[0]<space.shape[1]) and (new_pose[1]>=0 and new_pose[1]<space.shape[0]):
					if ospace[new_pose[1],new_pose[0]]==255.0:
						heappush(front,(new_cost,new_pose,pre_pose))
	
	trajectory=[]
	if pose==goal:
		while pose:
			trajectory.append(pose)
			pose=path[pose]
		trajectory.reverse()
	return visited,space,trajectory		

def planner3(start,goal,ospace):
	r=1.0 #resolution
	neighbours=[[r,0,r],[0,r,r],[-r,0,r],[0,-r,r],[r,r,r*sqrt(2)],[-r,r,r*sqrt(2)],[-r,-r,r*sqrt(2)],[r,-r,r*sqrt(2)]] # x,y,cost
	front=[]
	visited=[]
	space=np.zeros_like(ospace)
	heappush(front,(0.00001,0.00001,start,None))
	pose=start
	path={}
	while(pose!=goal):
		total_cost,cost,pose,ppose=heappop(front)
		if space[int(pose[1]),int(pose[0])]==0.0:
			visited.append((cost,pose,ppose))
			space[int(pose[1]),int(pose[0])]=cost
			key=pose
			path[key]=ppose
			for n in neighbours:
				new_cost,new_pose,pre_pose=cost+n[2],(pose[0]+n[0],pose[1]+n[1]),(pose[0],pose[1])
				r=distanceTogoal(new_pose,goal)
				total_cost=r+new_cost
				if (new_pose[0]>=0 and new_pose[0]<space.shape[1]) and (new_pose[1]>=0 and new_pose[1]<space.shape[0]):
					if ospace[int(new_pose[1]),int(new_pose[0])]==255.0:
						heappush(front,(total_cost,new_cost,new_pose,pre_pose))	
	trajectory=[]
	if pose==goal:
		while pose:
			trajectory.append(pose)
			pose=path[pose]
		trajectory.reverse()
	return visited,space,trajectory				

def planner(start,goal,ospace,d):
	r=1.0 #resolution
	neighbours=[[r,0,r],[0,r,r],[-r,0,r],[0,-r,r],[r,r,r*sqrt(2)],[-r,r,r*sqrt(2)],[-r,-r,r*sqrt(2)],[r,-r,r*sqrt(2)]] # x,y,cost
	front=[]
	visited=[]
	space=np.zeros_like(ospace)
	heappush(front,(0.00001,0.00001,start,None))
	pose=start
	path={}
	while(pose!=goal):
		total_cost,cost,pose,ppose=heappop(front)
		if space[int(pose[1]),int(pose[0])]==0.0:
			visited.append((cost,pose,ppose))
			space[int(pose[1]),int(pose[0])]=cost
			key=pose
			path[key]=ppose
			for n in neighbours:
				new_cost,new_pose,pre_pose=cost+n[2],(pose[0]+n[0],pose[1]+n[1]),(pose[0],pose[1])				
				if (new_pose[0]>=0 and new_pose[0]<space.shape[1]) and (new_pose[1]>=0 and new_pose[1]<space.shape[0]):
					if ospace[int(new_pose[1]),int(new_pose[0])]!=d:
						r=distanceTogoal(new_pose,goal)
						total_cost=r+new_cost+ospace[int(new_pose[1]),int(new_pose[0])]
						heappush(front,(total_cost,new_cost,new_pose,pre_pose))	
	trajectory=[]
	if pose==goal:
		while pose:
			trajectory.append(pose)
			pose=path[pose]
		trajectory.reverse()
	#plt.imshow(space,interpolation='nearest')
	#show()
	return visited,space,trajectory				
	
if __name__ == '__main__':
	x,y=[],[]
	start=(4,24)
	#start=(600,300)
	goal=(15,1)
	#goal=(330,50)
	d=3
	m,ospace=getimage(d)
	arena=insert_border(ospace,d)
	trajectory=[]

	#visited,space=planner1(start,goal,ospace)
	#visited,space,trajectory=planner2(start,goal,ospace)
	#visited,space,trajectory=planner3(start,goal,ospace)
	print "planning..."
	visited,space,trajectory=planner(start,goal,arena,d)
	for p in trajectory:
		x.append(p[0])
		y.append(p[1])
	plt.imshow(m,interpolation='nearest')
	plt.scatter(start[0],start[1],c='r', s=40)
	plt.scatter(goal[0],goal[1],c='g', s=40)
	plot(x,y,color="m",linewidth=2.0)

	show()
	


    




