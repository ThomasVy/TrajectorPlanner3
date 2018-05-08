#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np
from math import sin, cos, pi,sqrt,atan2
from Trajectory_Planner_class import Trajectory_Planner
from std_msgs.msg import Float32MultiArray	
from nav_msgs.msg import Odometry
import itertools
from numpy import ones,vstack
from numpy.linalg import lstsq

from pylab import *
from itertools import groupby
from operator import itemgetter
import matplotlib.pyplot as plt

def get_trajectory(xp,yp):
	xt,yt=[],[]
	m=len(xp)
	for i in xrange(m-1):
		p=int(100*sqrt((xp[i]-xp[i+1])**2+(yp[i]-yp[i+1])**2))
		if p>1: #increase number of points
			xtemp=np.linspace(xp[i],xp[i+1],p)
			ytemp=np.linspace(yp[i],yp[i+1],p)
			for j in xrange(len(xtemp)-1):
				xt.append(xtemp[j])
				yt.append(ytemp[j])	
		else:
			xt.append(xp[i])
			yt.append(yp[i])
				
	return xt,yt

def bspline(x,y):
	num=200
	xp, yp = Trajectory_Planner.Bezier(list(zip(x, y)),num).T
	return xp,yp
	
def odometryCb(msg):
    pose[0]=msg.pose.pose.position.x
    pose[1]=msg.pose.pose.position.y
    q0=msg.pose.pose.orientation.w
    q1=msg.pose.pose.orientation.x
    q2=msg.pose.pose.orientation.y
    q3=msg.pose.pose.orientation.z
    omega=msg.twist.twist.angular.z;
    psi = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))
    pose[2]=psi
    #print pose
    				
def path_publisher(xt,yt,xd,yd):
	d=0.2
	msg  = Float32MultiArray()
	p=sqrt((xd-pose[0])**2+(yd-pose[1])**2)
	print p	
	if (p<d) and xt!=[]:
		xd,yd=xt[0],yt[0]
		xt.pop(0)
		yt.pop(0)	
	msg.data=[xd,yd,0.0,0.0]
	path_pub.publish(msg)
	return xd,yd	
if __name__ == "__main__":	
	
	
    rospy.init_node('Path_publisher_node', anonymous=True) #make node 
    path_pub=rospy.Publisher('trajectory',Float32MultiArray,queue_size=10)
    odom_sub=rospy.Subscriber('odom',Odometry,odometryCb)  
    rate = rospy.Rate(1000) # 30hz
   

    d=0.2
    pose=[0.0,0.0,0.0] # x,y,theta
    x=np.linspace(0.0 ,1.0,10)
    y=-x**2
    xd,yd=0.0,0.0
    xp,yp=bspline(x,y)
    xt,yt=get_trajectory(xp,yp)
    #plt.scatter(xp,yp)
    #plt.scatter(xt,yt)
    #show()

    while not rospy.is_shutdown():		
		xd,yd=path_publisher(xt,yt,xd,yd)
		rate.sleep()	
		

		


