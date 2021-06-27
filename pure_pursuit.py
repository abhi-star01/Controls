#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


import math

delta = 0
vel = 0
kd = 0.1
index = 0
n = 0
path = list()
pub = rospy.Publisher("action", Twist, queue_size=1)

def main():
    
    rospy.init_node("p_p", anonymous=True)


    sub1=rospy.Subscriber("astroid_path", Path, path_callback)
    sub2=rospy.Subscriber("base_pose_ground_truth", Odometry, odo_callback)


    rospy.spin()


def path_callback(path_data):
    global path, n
   # rospy.loginfo(rospy.get_caller_id() + "Path data recieved")
    path = path_data.poses
    n = 1


def odo_callback(odo_data):
    global vel
    global yaw
    global ox, oy,n,pub
    #rospy.loginfo(rospy.get_caller_id() + "Odometry data recieved")
    vel = math.hypot(odo_data.twist.twist.linear.x , odo_data.twist.twist.linear.y)
    yaw = odo_data.twist.twist.angular.z
    ox = odo_data.pose.pose.position.x
    oy = odo_data.pose.pose.position.y
    act_twst = Twist()
    if(n == 1):
        pure_pursuit(act_twst)
        pub.publish(act_twst)
        print("action published", act_twst)
    

def pure_pursuit(act_twst):
    global index,ox,oy,kd,vel,yaw,pub
    i = index
    if i<(len(path)-2) :
         np=math.hypot(path[i].pose.position.x-ox, path[i].pose.position.y-oy) # first point on the path
         dis=math.hypot(path[i+1].pose.position.x-ox, path[i+1].pose.position.y-oy) 
        
         while (dis<np and i<len(path)-1): # checking other points if we find a nearer point on the path than first one 
            i=i+1
            np=dis
            dis=math.hypot(path[i+1].pose.position.x-ox, path[i+1].pose.position.y-oy)  
                 
         index = i+1
         while(np<math.fabs(kd*vel)): # finding goal point(at look-ahead of car) iterating from nearest point of car on the path 
            i=i+1
            np=math.hypot(path[i].pose.position.x-ox, path[i].pose.position.y-oy)

         g_x=path[i].pose.position.x-ox
         g_y=path[i].pose.position.y-oy
         g_xtrf=g_x*math.cos(yaw)+g_y*math.sin(yaw)
         g_ytrf=-g_x*math.sin(yaw)+g_y*math.cos(yaw) 
         alpha=math.atan(g_ytrf/g_xtrf)
    
    
         delta=math.atan(2*2.5*math.sin(alpha)/(0.0001+kd*vel)) # 0.0001 added to avoid atan(1/0) case
         MAX_STEER=math.pi/4
         if delta >= MAX_STEER:
            delta = MAX_STEER
         elif delta <= -MAX_STEER:
            delta = -MAX_STEER
         omega = math.tan(delta)*vel/2.5 #2.5m is wheelbase
        
         act_twst.linear.x=5*math.cos(yaw)   # maintaining a speed of 5 
         act_twst.linear.y=5*math.sin(yaw)   # maintaining a speed of 5
         act_twst.angular.z=omega

    else:
         act_twst.linear.x=0     # path end reached and zero velocity commands
         act_twst.linear.y=0
         act_twst.angular.z=0  
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
 



