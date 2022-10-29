#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import time
import math
import random
from numpy import load
from std_msgs.msg import Empty
from std_srvs.srv import Empty as emt


def callback_odom(data):
    global pose
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]

def cal_distance(x1, y1, x2, y2):
    return math.sqrt((x2-x1)**2+(y2-y1)**2)

def motion_go_straight(in_dirn):
    dict_1 = {0:[1, 0], 1:[0, 1], 2:[-1, 0], 3:[0, -1]}
    tolerance = 0.01
    current_x = micromouse_location[0] + dict_1[in_dirn][0]
    current_y = micromouse_location[1] + dict_1[in_dirn][1]
    micromouse_location[0] = current_x
    micromouse_location[1] = current_y
    if current_x>=0:
        current_x += 1
    if current_y<=0:
        current_y -= 1
    x_goal = (2*abs(current_x)-1)*0.09
    y_goal = (2*abs(current_y)-1)*0.09
    if current_x<0:
        x_goal = -x_goal
    if current_y<0:
        y_goal = -y_goal
    cmd_vel = Twist()
    cmd_vel.linear.y = 0
    cmd_vel.linear.z = 0
    cmd_vel.angular.x = 0
    cmd_vel.angular.y = 0
    rate = rospy.Rate(50) # 50
    angle1 = norm(math.atan2(y_goal-pose[1], x_goal-pose[0])-(pose[2]-math.pi/2))
    angle2 = norm(math.atan2(pose[1]-y_goal, pose[0]-x_goal)-(pose[2]-math.pi/2))
    while(cal_distance(pose[0], pose[1], x_goal, y_goal)>tolerance):
        angle1 = norm(math.atan2(y_goal-pose[1], x_goal-pose[0])-(pose[2]-math.pi/2))
        angle2 = norm(math.atan2(pose[1]-y_goal, pose[0]-x_goal)-(pose[2]-math.pi/2))
        if abs(angle1) <abs(angle2):
            angle = angle1
        else:
            angle = angle2
        cmd_vel.angular.z = 3.5*angle
        if abs(angle1)>math.pi/2:
            cmd_vel.linear.x = -0.85
        else:
            cmd_vel.linear.x =0.85 
        pub.publish(cmd_vel)
        rate.sleep()
    cmd_vel.linear.x = 0
    cmd_vel.angular.z = 0
    pub.publish(cmd_vel)

def perform_rotation(in_dirn):
    tolerance = 0.1
    cmd_vel = Twist()
    cmd_vel.linear.x = 0
    cmd_vel.linear.y = 0
    cmd_vel.linear.z = 0
    cmd_vel.angular.x = 0
    cmd_vel.angular.y = 0
    rate = rospy.Rate(50)
    goal_angle = norm(in_dirn*math.pi/2)

    last_time = 0
    ref_time = 0.1   
    last_yaw = pose[2]
    max_ang = 5

    Kp = 6    
    Ki = 0.003   
    Kd =  3.5     
    errSum = 0
    while(abs(norm(goal_angle - pose[2]+math.pi/2))>tolerance):
        current_time = int(round(time.time()*1000))
        change_in_time = current_time - last_time
        if change_in_time>=ref_time :
            error = norm(goal_angle - pose[2]+math.pi/2)
            if abs(error)>tolerance:
                if change_in_time<100000:
                    errSum += error*change_in_time*0.001
                    
                dErr = (pose[2] - last_yaw)/(change_in_time*0.01)
                
                out_error= Kp*error + Ki*errSum - Kd*dErr
                last_yaw = pose[2]
                if out_error>max_ang:
                    out_error = max_ang
                elif out_error<-1*max_ang:
                    out_error = -1*max_ang
                cmd_vel.angular.z = out_error
            pub.publish(cmd_vel)
        last_time = current_time
        rate.sleep()
    cmd_vel.angular.z = 0
    pub.publish(cmd_vel)
    return in_dirn

def norm(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


class goToGoal:
    def __init__(self, start_pt):
        self.start_point = start_pt
        self.path_found = []
        self.in_dirn = 3      #E:0, N:1, W:2, S:3
        self.dict_1 = {(1, 0): 0, (0, 1):1, (-1, 0):2, (0, -1):3}
        
    def reach_goal(self):
        while len(self.path_found):
            newCell = self.path_found.pop(0)
            last_dirn = self.in_dirn
            self.in_dirn = self.dict_1[(newCell[1]-self.start_point[1], self.start_point[0]-newCell[0])]
            if last_dirn != self.in_dirn:
                perform_rotation(self.in_dirn)
            motion_go_straight(self.in_dirn)
            self.start_point = newCell


def runMicromouse():
    path_found = load('shortest_path.npy')
    path_found = path_found.tolist()
    start_pt = [0, 0] 
    findShortestPath = goToGoal(start_pt)

    findShortestPath.path_found = path_found

    print("______________going toward goal using shortest path_______________")
    micr_start_time = time.time()
    findShortestPath.reach_goal()
    micro_runTime = (time.time()) - micr_start_time
    print("Run Time_sec_:",micro_runTime)
    print("Run Time_minutes_",(micro_runTime)/60)

def callback_function(msg):
    resetWorld()
    global pub, micromouse_location
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub_odom = rospy.Subscriber('/odom', Odometry, callback_odom)
    
    micr_dirn = 3
    micromouse_location = [-8, 8]
    while (True):
        try:
            pose
            break
        except:
            continue
    runMicromouse()
        
if __name__ == '__main__':
    rospy.init_node('gotodestination', anonymous=True)
    global resetWorld
    resetWorld = rospy.ServiceProxy('/gazebo/reset_world', emt)
    sub1 = rospy.Subscriber('/gotodestination', Empty, callback_function)
    rospy.spin()
