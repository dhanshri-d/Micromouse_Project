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
from numpy import asarray
from numpy import save
from std_msgs.msg import Empty
from std_srvs.srv import Empty as emt




MAX_SIZE = 9223372036854775807

micromouse_location = [-8, 8]  #[7, -7] #0 : x, 1: y




def callback_odom(data):
    global pose
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]



def callback_laser(msg):
    global regions          #360 lines
    regions = {
        'right': min(min(msg.ranges[0:10]), 10),
        'front': min(min(msg.ranges[174:185]), 10),
        'left':min(min(msg.ranges[349:360]), 10)
    }
    
 

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
        cmd_vel.angular.z = 5*angle
        if abs(angle1)>math.pi/2:
            cmd_vel.linear.x = -0.75
        else:
            cmd_vel.linear.x =0.75
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


    last_time= 0
    ref_time = 0.1 
    last_yaw = pose[2]
    max_ang = 5
    
    
    Kp =  8
    Ki =  0.003
    Kd = 4

   
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




class gridCell:
    def __init__(self, cord_x, cord_y):
        self.x = cord_x
        self.y = cord_y
        self.is_wallPresent = [True, True, True, True] #E, N, W, S -> 0, 1, 2, 3 -> Right, Up, Left, Down




class FloodFillAlgo:
    def __init__(self, start_loc, goal_loc, algo):
        self.n = 16
        self.m = 16
        self.start_point = start_loc
        self.goal_points = goal_loc
        self.floodFill = [[-1]*self.m for _ in range(self.n)]
        self.resetMaze = None
        self.setWallsForMaze()
        self.path_found = []
        self.currentGoal = None
        self.in_dirn = 3 
        self.dict_1 = {(1, 0): 0, (0, 1):1, (-1, 0):2, (0, -1):3}
        self.dict_2 = {0:(0, 1), 1:(-1, 0), 2:(0, -1), 3:(1, 0)}
        self.algo = algo
    
    def setWallsForMaze(self):
        self.resetMaze = [[0]*self.m for _ in range(self.n)]
        for i in range(self.n):
            for j in range(self.m):
                new_cell = gridCell(i, j)
                if i == 0:
                    new_cell.is_wallPresent[1] = False
                if i == self.n-1:
                    new_cell.is_wallPresent[3] = False
                if j == 0:
                    new_cell.is_wallPresent[2] = False
                if j == self.m-1:
                    new_cell.is_wallPresent[0] = False
                self.resetMaze[i][j] = new_cell
    
    def setWalls_currentCell(self, cell):
        wallPresent = False
        #dist_ = 0.18

        if self.resetMaze[cell[0]][cell[1]].is_wallPresent[self.in_dirn] and regions['front']<0.18:
            wallPresent = True
            self.resetMaze[cell[0]][cell[1]].is_wallPresent[self.in_dirn] = False
            next_row = cell[0] + self.dict_2[self.in_dirn][0]
            next_clm = cell[1] + self.dict_2[self.in_dirn][1]
            if 0<=next_row<=16 and 0<=next_clm<=16:
                self.resetMaze[next_row][next_clm].is_wallPresent[(self.in_dirn+2)%4] = False
        
        if self.resetMaze[cell[0]][cell[1]].is_wallPresent[(self.in_dirn-1)%4] and regions['right']<0.18:
            wallPresent = True
            self.resetMaze[cell[0]][cell[1]].is_wallPresent[(self.in_dirn-1)%4] = False
            next_row = cell[0] + self.dict_2[(self.in_dirn-1)%4][0]
            next_clm = cell[1] + self.dict_2[(self.in_dirn-1)%4][1]
            if 0<=next_row<=16 and 0<=next_clm<=16:
                self.resetMaze[next_row][next_clm].is_wallPresent[(self.in_dirn+1)%4] = False
        
        if self.resetMaze[cell[0]][cell[1]].is_wallPresent[(self.in_dirn+1)%4] and regions['left']<0.18:
            wallPresent = True
            self.resetMaze[cell[0]][cell[1]].is_wallPresent[(self.in_dirn+1)%4] = False
            next_row = cell[0] + self.dict_2[(self.in_dirn+1)%4][0]
            next_clm = cell[1] + self.dict_2[(self.in_dirn+1)%4][1]
            if 0<=next_row<=16 and 0<=next_clm<=16:
                self.resetMaze[next_row][next_clm].is_wallPresent[(self.in_dirn-1)%4] = False
        
        return wallPresent

    def get_grid(self):
        return self.resetMaze
    
    def fillGrid(self):
        start_pt = self.start_point
        stack = []
        for g in self.goal_points:
            self.floodFill[g[0]][g[1]] = 0
            stack.append(g)
        
        while(len(stack)>0):
            current_cell = stack.pop(0)
            up = self.resetMaze[current_cell[0]][current_cell[1]].is_wallPresent[1]
            down = self.resetMaze[current_cell[0]][current_cell[1]].is_wallPresent[3]
            left = self.resetMaze[current_cell[0]][current_cell[1]].is_wallPresent[2]
            right = self.resetMaze[current_cell[0]][current_cell[1]].is_wallPresent[0]

    
            if up == True and self.floodFill[current_cell[0]-1][current_cell[1]] == -1:
                self.floodFill[current_cell[0]-1][current_cell[1]] = 1+self.floodFill[current_cell[0]][current_cell[1]]
                stack.append([current_cell[0]-1, current_cell[1]])
            if down == True and self.floodFill[current_cell[0]+1][current_cell[1]] == -1:
                self.floodFill[current_cell[0]+1][current_cell[1]] = 1+self.floodFill[current_cell[0]][current_cell[1]]
                stack.append([current_cell[0]+1, current_cell[1]])
            if left == True and self.floodFill[current_cell[0]][current_cell[1]-1] == -1:
                self.floodFill[current_cell[0]][current_cell[1]-1] = 1+self.floodFill[current_cell[0]][current_cell[1]]
                stack.append([current_cell[0], current_cell[1]-1])
            if right == True and self.floodFill[current_cell[0]][current_cell[1]+1] == -1:
                self.floodFill[current_cell[0]][current_cell[1]+1] = 1+self.floodFill[current_cell[0]][current_cell[1]]
                stack.append([current_cell[0], current_cell[1]+1])

    def get_valid_path(self):
        path_found = []
        isGoalFound = False
        current_cell = self.start_point
        nextCellWithMinDist = current_cell
        ref_min_distance = self.floodFill[current_cell[0]][current_cell[1]]
        while not isGoalFound:
            current_cell = nextCellWithMinDist
            ref_min_distance = self.floodFill[current_cell[0]][current_cell[1]]
            up = self.resetMaze[current_cell[0]][current_cell[1]].is_wallPresent[1]
            down = self.resetMaze[current_cell[0]][current_cell[1]].is_wallPresent[3]
            left = self.resetMaze[current_cell[0]][current_cell[1]].is_wallPresent[2]
            right = self.resetMaze[current_cell[0]][current_cell[1]].is_wallPresent[0]
            
            valid_path = []

            if up and self.floodFill[current_cell[0]-1][current_cell[1]]<ref_min_distance:
                valid_path.append([current_cell[0]-1, current_cell[1]])
            
            if down and self.floodFill[current_cell[0]+1][current_cell[1]]<ref_min_distance:
                valid_path.append([current_cell[0]+1, current_cell[1]])
            
            if left and self.floodFill[current_cell[0]][current_cell[1]-1]<ref_min_distance:
                valid_path.append([current_cell[0], current_cell[1]-1])
            
            if right and self.floodFill[current_cell[0]][current_cell[1]+1]<ref_min_distance:
                valid_path.append([current_cell[0], current_cell[1]+1])
            
            if self.algo == 1:
                nextCellWithMinDist = valid_path[0]
            elif self.algo == 2:
                nextCellWithMinDist = valid_path[-1]
            else:
                i = random.randint(0, len(valid_path)-1)
                nextCellWithMinDist = valid_path[i]
            
            ref_min_distance = self.floodFill[nextCellWithMinDist[0]][nextCellWithMinDist[1]]

            for g in self.goal_points:
                if nextCellWithMinDist[0]==g[0] and nextCellWithMinDist[1]==g[1]:
                    isGoalFound = True
            
            path_found.append(nextCellWithMinDist)
        return path_found

    def findValidPath(self):
        self.floodFill = [[-1]*self.m for _ in range(self.n)]
        self.fillGrid()
        self.path_found = self.get_valid_path()

    def DefineMaze(self):
        actual_path = [self.start_point]
        isGoalFound = False
        for g in self.goal_points:
            if self.start_point[0] == g[0] and self.start_point[1] == g[1]:
                isGoalFound = True
         
        while not isGoalFound:
            wallPresent = self.setWalls_currentCell(self.start_point)
            if wallPresent or len(self.path_found)==0:
                self.findValidPath()

            newCell = self.path_found.pop(0)
            last_dirn = self.in_dirn
            self.in_dirn = self.dict_1[(newCell[1]-self.start_point[1], self.start_point[0]-newCell[0])]
            if last_dirn != self.in_dirn:
                perform_rotation(self.in_dirn)
            motion_go_straight(self.in_dirn)

            if len(actual_path)>1 and actual_path[-2][0] == newCell[0] and actual_path[-2][1] == newCell[1]:
                actual_path.pop()
            else:
                actual_path.append(newCell)

            self.start_point = newCell

            for g in self.goal_points:
                if self.start_point[0] == g[0] and self.start_point[1] == g[1]:
                    isGoalFound = True
                    self.currentGoal = g
            
        return actual_path
        
    def reach_goal(self):
        while len(self.path_found):
            newCell = self.path_found.pop(0)
            last_dirn = self.in_dirn
            self.in_dirn = self.dict_1[(newCell[1]-self.start_point[1], self.start_point[0]-newCell[0])]
            if last_dirn != self.in_dirn:
                perform_rotation(self.in_dirn)
            motion_go_straight(self.in_dirn)
            self.start_point = newCell
#______________________________________ALGO END_____________________________________________________________________________#



def runMicromouse():
    run = True
    start_pt = [0,0]
    goal_pts = [[7, 7], [7, 8], [8, 7], [8, 8]] 
    findShortestPath = FloodFillAlgo(start_pt, goal_pts, 1)
    micr_search_time_start = time.time()
    findShortestPath.DefineMaze()
    #print("_________________________At goal point________________________")
    
    currentGoal = findShortestPath.currentGoal
    findShortestPath.goal_points = [start_pt]
    #print("___________Going to start position using shortest path________________")
    findShortestPath.DefineMaze()
    micr_search_time_end = time.time()
    search_time = micr_search_time_end-micr_search_time_start
    print("Search Time_Sec_ : ",search_time)
    print("Search Time_Minutes_: ",(search_time)/60)
    print("____________Shortest path found, Going to destination using shortest path_____________________")
    
    micr_run_time_start = time.time()

    findShortestPath.goal_points = [currentGoal]
    
    print(findShortestPath.DefineMaze())
    micr_run_time= (time.time()) - micr_run_time_start
    print("Run Time_sec_ : ",micr_run_time)
    print("Run Time_Minutes_ : ",(micr_run_time)/60)
    
    findShortestPath.start_point = start_pt
    findShortestPath.findValidPath()
    save('shortest_path.npy', asarray(findShortestPath.path_found))
    
    

    
def main():
    global pub
    micr_dirn = 3
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub_odom = rospy.Subscriber('/odom', Odometry, callback_odom)
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, callback_laser)  

    while (True):
        try:
            pose
            regions
            break
        except:
            continue
    
    runMicromouse()
      

if __name__ == '__main__':
    rospy.init_node('findshortestpath', anonymous=True)
    resetWorld = rospy.ServiceProxy('/gazebo/reset_world', emt)
    while (True):
        input_ = int(input("Enter 0 to find shortest path or enter 1 for going to goal using shortest path: "))
        if input_:
            resetWorld()
            pub1 = rospy.Publisher('gotodestination', Empty, queue_size=1)
            pub1.publish()
        else:
            print("Finding Shortest path")
            main()

