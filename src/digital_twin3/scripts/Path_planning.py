import rospy

import sys, os, math, csv
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from decimal import Decimal
import matplotlib.patches as mpatches
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from AStarPlanner import AStarPlanner
from DWA import DWA
from digital_twin3.srv import get_robot_env

class DWAConfig:
    robot_radius = 0.25

    def __init__(self, obs_radius):
        self.obs_radius = obs_radius
        self.dt = 0.1  # [s] Time tick for motion prediction

        self.max_speed = 1  # [m/s] 最大线速度
        self.min_speed = -1  # [m/s] 最小线速度
        self.max_accel = 0.5 # [m/ss] 加速度
        self.v_reso = self.max_accel*self.dt/2  # [m/s] 速度增加的步长

        self.min_yawrate = -90.0 * math.pi / 180.0  # [rad/s] 最小角速度
        self.max_yawrate = 90.0 * math.pi / 180.0  # [rad/s] 最大角速度
        self.max_dyawrate = 90.0 * math.pi / 180.0  # [rad/ss] 角加速度
        self.yawrate_reso = self.max_dyawrate*self.dt/2  # [rad/s] 角速度增加的步长

        # 模拟轨迹的持续时间
        self.predict_time = 3  # [s]

        # 三个比例系数
        self.to_goal_cost_gain = 1.0  # 距离目标点的评价函数的权重系数
        self.speed_cost_gain = 1.0  # 速度评价函数的权重系数
        self.obstacle_cost_gain = 0.3  # 距离障碍物距离的评价函数的权重系数

        self.tracking_dist = self.predict_time * self.max_speed
        self.arrive_dist = 0.1


class DT_planning:
    def __init__(self):
        self.EXIT = False

        # map related
        self.mapRange = []
        self.startPoint = []
        self.endPoint = []
        self.obsList = np.empty(shape=(0, 2))
        self.globalPathList = np.empty(shape=(0, 2))
        self.planning_obs_radius = 0.15
        
        # DWA local planning related
        self.dwa = DWA()
        self.dwaMidposIndex = None
        self.dwaConfig = DWAConfig(self.planning_obs_radius)
        self.timestep = self.dwaConfig.dt

        # robot related
        self.robot_radius = 0.25
        self.robotPos = [0, 0, 0]
        self.robotVel = [0, 0, 0]
        self.robotTraj = [[0, 0]]
        
        # figure related
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.canvas.mpl_connect('key_press_event', self.key_press)

    def process(self):
        rospy.init_node('DT_path_planning', anonymous=True)
        
        # get webots env information
        rospy.wait_for_service('get_robot_env_infor')
        getWebotsEnvClient = rospy.ServiceProxy('get_robot_env_infor',get_robot_env)
        RES = getWebotsEnvClient(1)
        self.startPoint = [RES.robot_pos.x, RES.robot_pos.y]
        self.endPoint = [RES.end_pos.x, RES.end_pos.y]
        self.mapRange = [-RES.x_size/2, RES.x_size/2, -RES.y_size/2, RES.y_size/2]
        for obsPoint in RES.static_obs:
            self.obsList = np.append(self.obsList, [[obsPoint.x, obsPoint.y]], axis=0)

        # A* based on the webots env information
        self.globalPathList = self.global_plan()
        # print(self.globalPathList)
        rospy.loginfo("Planning | A* plans global path done! Path points number : %d",self.globalPathList.shape[0])
        with open(os.path.dirname(os.path.abspath(__file__))+'/globalPath.csv', "w",newline='') as csv_file:
            writer = csv.writer(csv_file)
            for line in self.globalPathList:
                # 四舍五入保留两位小数
                a = Decimal(line[0]).quantize(Decimal("0.1"), rounding = "ROUND_HALF_UP")
                b = Decimal(line[1]).quantize(Decimal("0.1"), rounding = "ROUND_HALF_UP")
                writer.writerow([a, b])


        rospy.Subscriber("/dt_robot/odom", Odometry, self.get_robot_states_callback)
        robotVelCtrlPub = rospy.Publisher('/controller2', Point, queue_size=10)

        rate = rospy.Rate(1/self.timestep)
        while not rospy.is_shutdown():
            if self.EXIT:
                plt.close("all")
                break
            
            # preview point index
            self.dwaMidposIndex = self.check_path()
            robotCurrStates = [self.robotPos[0],self.robotPos[1],self.robotPos[2],self.robotVel[0],self.robotVel[1],self.robotVel[2]]

            # get best velocity
            if self.dwaMidposIndex >= 0:
                midpos = self.globalPathList[self.dwaMidposIndex]
                [self.robotVel[0],self.robotVel[1],self.robotVel[2]], best_traj, all_traj, all_u = self.dwa.plan(
                    robotCurrStates,self.dwaConfig, midpos, self.obsList)
            else:
                self.robotVel = [0.0, 0.0, 0.0]
            
            msg = Point()
            msg.x, msg.y, msg.z = self.robotVel[0], self.robotVel[1], self.robotVel[2]
            robotVelCtrlPub.publish(msg)
            
            self.robotTraj.append([self.robotPos[0],self.robotPos[1]])

            plt.cla()
            self.plot(all_traj, all_u, best_traj)
            rate.sleep()

    def plot(self, all_traj, all_value, best_traj):
        # 车身绘制，包含位置和角度
        p1_i = np.array([0.3, 0, 1]).T
        p2_i = np.array([-0.3, 0.15, 1]).T
        p3_i = np.array([-0.3, -0.15, 1]).T
        T = self.transformation_matrix(self.robotPos[0], self.robotPos[1], self.robotPos[2])
        p1 = np.matmul(T, p1_i)
        p2 = np.matmul(T, p2_i)
        p3 = np.matmul(T, p3_i)
        pp1 = plt.plot([p1[0], p2[0], p3[0], p1[0]], [p1[1], p2[1], p3[1], p1[1]], 
                 'c-', linewidth =2.0)

        # 边界绘制
        pp2 = plt.plot([self.mapRange[0], self.mapRange[0], self.mapRange[1], self.mapRange[1], self.mapRange[0]], 
                 [self.mapRange[2], self.mapRange[3], self.mapRange[3], self.mapRange[2], self.mapRange[2]],
                 'k--', linewidth = 3.5)
        pp3 = None
        # self.ax.grid()
        # 目标点绘制
        if self.endPoint is not None:
            pp3 = self.ax.plot(
                self.endPoint[0], self.endPoint[1], "m*", markersize=20)
        
        # 所有路径绘制
        if len(all_traj) > 0:
            all_value = np.array(all_value,dtype=float)
            all_value = (all_value-all_value.min())/(all_value.max()-all_value.min())
            for i,traj in enumerate(all_traj):
                color = plt.cm.jet(all_value[i])
                self.ax.plot(traj[:,0],traj[:,1],".",color=color,markersize=1)
                self.ax.plot(traj[-1,0],traj[-1,1],"+",color=color,markersize=3)

        # 当前最优路径绘制
        if best_traj is not None:
            self.ax.plot(best_traj[:,0],best_traj[:,1],color="red",linewidth=3)
        
        pp4 = None
        pp5 = None
        pp6 = None
        if self.globalPathList is not None:
            # 全局路径绘制
            pp4 = self.ax.plot(self.globalPathList[:, 0],
                         self.globalPathList[:, 1], 'b:', linewidth = 3.0)
            # 参考点绘制
            if self.dwaMidposIndex is not None and self.dwaMidposIndex >= 0:
                midpos = self.globalPathList[self.dwaMidposIndex]
                pp5 = self.ax.plot(midpos[0], midpos[1], "g+", markersize=20)

        # 车走过的路径绘制
        if len(self.robotTraj) > 0:
            plt.plot(self.robotTraj[0], self.robotTraj[1], 'g-')
        
        # 障碍物绘制
        for obs in self.obsList:
            pp6 = self.ax.add_artist(plt.Circle(
                (obs[0], obs[1]), self.planning_obs_radius, fill=True, color="red"))
       
        # 坐标轴
        self.ax.set_xlim(self.mapRange[0]-1, self.mapRange[1]+1)
        self.ax.set_ylim(self.mapRange[2]-1, self.mapRange[3]+1)
        self.ax.set_xlabel("X-axis (m)",fontsize = 24)
        self.ax.set_ylabel("Y-axis (m)",fontsize = 24)
        self.ax.tick_params(labelsize=20)
        m = 3
        self.ax.spines['bottom'].set_linewidth(m)
        self.ax.spines['left'].set_linewidth(m)
        self.ax.spines['right'].set_linewidth(m)
        self.ax.spines['top'].set_linewidth(m)

        # color = ['lightskyblue', 'lime', 'red', 'gold']
        # labels = ['winter', 'spring', 'summer', 'autumn']
        # patches = [ mpatches.Patch(color=color[i], label="{:s}".format(labels[i]) ) for i in range(len(color))]
        # self.ax.legend(handles=patches, bbox_to_anchor=(0.95,1.12), ncol=4)
        
        #l1 = self.ax.legend((pp2), ("boundary"), loc=0, frameon=False)
        # l1 = self.ax.legend((pp2,pp3,pp6), 
        #                ("boundary","endpoint","obstacle"), 
        #                loc=0)
        
        # l2 = self.ax.legend([pp1,pp2,pp3,pp4,pp5,pp6], 
        #                ["robot","boundary","endpoint","global path","preview point","obstacle"], 
        #                loc='upper left')
        
        # l2 = self.ax.legend([pp1,pp2,pp3,pp4,pp5,pp6], 
        #                ["robot","boundary","endpoint","global path","preview point","obstacle"], 
        #                loc='upper left')

        plt.pause(self.timestep)

    def check_path(self):
        if self.globalPathList is None or self.globalPathList.shape[0] == 0:
            return -1
        if self.dwaMidposIndex is not None and self.dwaMidposIndex >= 0:
            midindex = self.dwaMidposIndex
            
            # 定义自适应预瞄点
            vv = np.hypot(self.robotVel[0],self.robotVel[1])
            myDist = vv*self.dwaConfig.predict_time
            myDist = myDist if myDist >= 1.5 else 1
            self.dwaConfig.tracking_dist = myDist

            while True:
                midpos = self.globalPathList[midindex]
                dist = np.hypot(self.robotPos[0]-midpos[0], self.robotPos[1]-midpos[1])
                if dist > self.dwaConfig.tracking_dist:
                    break
                if midindex + 1 == self.globalPathList.shape[0]:
                    return midindex
                midindex += 1
            return midindex
        else:
            return 0
        
    def get_robot_states_callback(self, data):
        self.robotPos[0] = data.pose.pose.position.x
        self.robotPos[1] = data.pose.pose.position.y

        Vx = data.pose.pose.orientation.x
        Vy = data.pose.pose.orientation.y
        Vz = data.pose.pose.orientation.z
        Vw = data.pose.pose.orientation.w
        self.robotPos[2] =self.orientation_to_yaw(Vw, Vx, Vy, Vz)

        self.robotVel[0] = data.twist.twist.linear.x
        self.robotVel[1] = data.twist.twist.linear.y
        self.robotVel[2] = data.twist.twist.angular.z
        # print(self.robotPos)

    def key_press(self,event):
        if(event.key == 'escape'):
            self.EXIT = True

    def transformation_matrix(self, x, y, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta), x],
            [np.sin(theta), np.cos(theta), y],
            [0, 0, 1]
        ])
    
    def global_plan(self):
        globalPlanner = AStarPlanner(0.1)
        gPathX, gPathY = globalPlanner.planning(
            self.obsList[:, 0], self.obsList[:, 1], self.robot_radius+0.3, 
            self.startPoint[0], self.startPoint[1], 
            self.endPoint[0], self.endPoint[1],
            self.mapRange[0], self.mapRange[2], self.mapRange[1], self.mapRange[3]
        )
        globalPath = np.vstack([gPathX, gPathY]).T
        # print(globalPath)
        return globalPath

    def add_obs(self, x, y):
        self.obsList = np.append(self.obsList, [[x, y]], axis=0)

    def publisher(self):
        pass

    def orientation_to_yaw(self, w, x , y, z):
            return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))




if __name__ == '__main__':
    try:
        DT_path_planning = DT_planning()
        DT_path_planning.process()

    except rospy.ROSInterruptException:
        pass
    