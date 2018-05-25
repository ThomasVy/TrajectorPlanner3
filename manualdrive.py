#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
from math import sin, cos, pi,sqrt, atan2
from Trajectory_Planner_class import Trajectory_Planner
from std_msgs.msg import Float32MultiArray	
import sys,tty,termios

pose=[0.0,0.0,0.0]
new_pose=[0.0,0.0,0.0]

class _Getch:
    def __call__(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(3)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

def get():
        inkey = _Getch()
        while(1):
                k=inkey()
                if k!='':break
        if k=='\x1b[A':
                command=0.0
        elif k=='\x1b[B':
                command=None
                sys.exit()
        elif k=='\x1b[C':
                command=-1.0
        elif k=='\x1b[D':
                command=1.0
        else:
                command=None
                sys.exit()             
	return command
              
                				
def path_publisher(new_pose):
	print new_pose
	msg  = Float32MultiArray()
	msg.data=[new_pose[0],new_pose[1],new_pose[2],0]
	path_pub.publish(msg)
	

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

def end_pose(start_pose, curvature, length):
	k=0.8
	p=sqrt((new_pose[0]-pose[0])**2+(new_pose[1]-pose[1])**2)
	if p<k*length:		
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
	else:
		return new_pose
			
		    	
if __name__ == "__main__":
	
    rospy.init_node('manualdrive', anonymous=True) #make node 
    path_pub=rospy.Publisher('trajectory',Float32MultiArray,queue_size=10)
    odom_sub=rospy.Subscriber('odom',Odometry,odometryCb)
    length=0.2
    rate = rospy.Rate(30) # 30hz	
    while not rospy.is_shutdown():
		command=get()
		new_pose=end_pose(pose, command, length)		
		path_publisher(new_pose)
		rate.sleep()
		
		#command=get()
		print command
		



		
	
