import rospy
from std_msgs.msg import String
import numpy as np
from math import sin, cos, pi,sqrt
from heapq import heappush, heappop
from scipy import ndimage
from scipy.special import binom
from std_msgs.msg import Float32MultiArray

class Trajectory_Planner:
	@staticmethod
	def Bernstein(n, k):
	    coeff = binom(n, k)
	    def _bpoly(x):
	        return coeff * x ** k * (1 - x) ** (n - k)
	    return _bpoly
	    
	@staticmethod
	def Bezier(points, num):
	    N = len(points)
	    t = np.linspace(0, 1, num=num)
	    curve = np.zeros((num, 2))
	    for ii in range(N):
	        curve += np.outer(Trajectory_Planner.Bernstein(N - 1, ii)(t), points[ii])
	    return curve
	    
	@staticmethod   
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
			
	@staticmethod           
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
	
	@staticmethod		
	def distanceTogoal(pose,goal):
		dx=goal[0]-pose[0]
		dy=goal[1]-pose[1]
		r=sqrt(dx**2+dy**2)
		return r
	
	@staticmethod		
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
	
