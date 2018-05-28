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
np.set_printoptions(threshold='nan')

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

def insert_border1(ospace,d):
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
	m= misc.imread('path7.png')
	ospace=np.zeros(m.shape[0]*m.shape[1]).reshape(m.shape[0],m.shape[1])
	for i in xrange(m.shape[0]):
		for j in xrange(m.shape[1]):
			ospace[i,j]=-float(m[i,j][0])*1/255+1
	#plt.imshow(m,interpolation='nearest')
	#show()
	return m,ospace
				

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
					if ospace[int(new_pose[1]),int(new_pose[0])]<h:
						r=distanceTogoal(new_pose,goal)
						total_cost=r+new_cost+ospace[int(new_pose[1]),int(new_pose[0])]
						heappush(front,(total_cost,new_cost,new_pose,pre_pose))	
	trajectory=[]
	if pose==goal:
		while pose:
			trajectory.append(pose)
			pose=path[pose]
		trajectory.reverse()
	plt.imshow(space,interpolation='nearest')
	show()
	return visited,space,trajectory				
	
if __name__ == '__main__':
	x,y=[],[]
	start=(4,24)
	#start=(600,300)
	goal=(15,1)
	goal=(700,350)
	d=80
	h=d
	m,ospace=getimage(d)
	arena=insert_border(ospace,d)
	#visited,space=planner1(start,goal,ospace)
	#visited,space,trajectory=planner2(start,goal,ospace)
	#visited,space,trajectory=planner3(start,goal,ospace)
	visited,space,trajectory=planner(start,goal,arena,d)
	for p in trajectory:
		x.append(p[0])
		y.append(p[1])

	plt.imshow(m,interpolation='nearest')
	plt.scatter(start[0],start[1],c='r', s=40)
	plt.scatter(goal[0],goal[1],c='g', s=40)
	plot(x,y,color="m",linewidth=2.0)

	show()
