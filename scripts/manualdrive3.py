#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
from math import sin, cos, pi,sqrt, atan2
from Trajectory_Planner_class import Trajectory_Planner
from std_msgs.msg import Float32MultiArray	
import sys,tty,termios

i=1
rt=0.35
fw=0.1
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
                command=2.0
        elif k=='\x1b[C':
                command=-1.0
        elif k=='\x1b[D':
                command=1.0
        else:
                command=None
                sys.exit()          
	return command
              
                				
def path_publisher(command):
	msg  = Float32MultiArray()
	if command==0.0:
		Vr=fw
		Wr=0.0
	elif command==-1.0:
		Vr=0.0#fw
		Wr=-rt
	elif command==1.0:
		Vr=fw
		Wr=rt
	elif command==2.0:
		Vr=0.0
		Wr=0.0
	else:
		Vr=0.0
		Wr=0.0
		sys.exit()
	print command
	msg.data=[Vr,Wr]
	path_pub.publish(msg)
	
		    	
if __name__ == "__main__":
	
    rospy.init_node('manualdrive', anonymous=True) #make node 
    path_pub=rospy.Publisher('trajectory',Float32MultiArray,queue_size=10)

    rate = rospy.Rate(10) # 30hz	
    while not rospy.is_shutdown():
		command=get()	
		path_publisher(command)
		rate.sleep()

		



		
	
