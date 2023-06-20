import rospy

import sys, os, math
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from AStarPlanner import AStarPlanner
from DWA import DWA
from digital_twin3.srv import get_robot_env



class DWAConfig:
    robot_radius = 0.25

    def __init__(self, obs_radius):
        self.obs_radius = obs_radius
        self.dt = 0.1  # [s] Time tick for motion prediction

        self.max_speed = 1.5  # [m/s] 最大线速度
        self.min_speed = -0.5  # [m/s] 最小线速度
        self.max_accel = 0.5 # [m/ss] 加速度
        self.v_reso = self.max_accel*self.dt/10.0  # [m/s] 速度增加的步长

        self.min_yawrate = -100.0 * math.pi / 180.0  # [rad/s] 最小角速度
        self.max_yawrate = 100.0 * math.pi / 180.0  # [rad/s] 最大角速度
        self.max_dyawrate = 500.0 * math.pi / 180.0  # [rad/ss] 角加速度
        self.yawrate_reso = self.max_dyawrate*self.dt/10.0  # [rad/s] 角速度增加的步长

        # 模拟轨迹的持续时间
        self.predict_time = 2  # [s]

        # 三个比例系数
        self.to_goal_cost_gain = 1.0  # 距离目标点的评价函数的权重系数
        self.speed_cost_gain = 0.7  # 速度评价函数的权重系数
        self.obstacle_cost_gain = 1.0  # 距离障碍物距离的评价函数的权重系数

        self.tracking_dist = self.predict_time*self.max_speed
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
        self.planning_obs_radius = 0.25
        
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
        self.fig, self.ax = plt.subplots()
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
        rospy.loginfo("Planning | A* plans global path done! Path points number : %d",self.globalPathList.shape[0])
        
        rospy.Subscriber("/dt_robot/odom", Odometry, self.get_robot_states_callback)
        robotVelCtrlPub = rospy.Publisher('/controller2', Point, queue_size=10)

        rate = rospy.Rate(1/self.timestep)
        while not rospy.is_shutdown():
            if self.EXIT:
                plt.close("all")
                break
            
            # preview point index
            self.dwaMidposIndex = self.check_path()
            curRbtStates = [self.robotPos[0],self.robotPos[1],self.robotPos[2],self.robotVel[0],self.robotVel[2]]
            
            # get best velocity
            if self.dwaMidposIndex >= 0:
                midpos = self.globalPathList[self.dwaMidposIndex]
                [self.robotVel[0],self.robotVel[2]], best_traj, all_traj, all_u = self.dwa.plan(
                    curRbtStates,self.dwaConfig, midpos, self.obsList)
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
        p1_i = np.array([0.5, 0, 1]).T
        p2_i = np.array([-0.5, 0.25, 1]).T
        p3_i = np.array([-0.5, -0.25, 1]).T
        T = self.transformation_matrix(self.robotPos[0], self.robotPos[1], self.robotPos[2])
        p1 = np.matmul(T, p1_i)
        p2 = np.matmul(T, p2_i)
        p3 = np.matmul(T, p3_i)
        plt.plot([p1[0], p2[0], p3[0], p1[0]], [
                 p1[1], p2[1], p3[1], p1[1]], 'c-')

        # 边界绘制
        plt.plot([self.mapRange[0], self.mapRange[0], self.mapRange[1], self.mapRange[1], self.mapRange[0]], 
                 [self.mapRange[2], self.mapRange[3], self.mapRange[3], self.mapRange[2], self.mapRange[2]],
                 'k-', markersize=10)
        
        # 目标点绘制
        if self.endPoint is not None:
            self.ax.plot(
                self.endPoint[0], self.endPoint[1], "r*", markersize=10)
        
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

        if self.globalPathList is not None:
            # 全局路径绘制
            self.ax.plot(self.globalPathList[:, 0],
                         self.globalPathList[:, 1], 'b--')
            # 参考点绘制
            if self.dwaMidposIndex is not None and self.dwaMidposIndex >= 0:
                midpos = self.globalPathList[self.dwaMidposIndex]
                self.ax.plot(midpos[0], midpos[1], "g+", markersize=20)

        # 车走过的路径绘制
        if len(self.robotTraj) > 0:
            plt.plot(self.robotTraj[0], self.robotTraj[1], 'g-')
        
        # 障碍物绘制
        # for obs in self.obsList:
        #     self.ax.add_artist(plt.Circle(
        #         (obs[0], obs[1]), self.planning_obs_radius, fill=True))
       
        # 坐标轴
        self.ax.set_xlim(self.mapRange[0]-2, self.mapRange[1]+2)
        self.ax.set_ylim(self.mapRange[2]-2, self.mapRange[3]+2)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")

        plt.pause(self.timestep)

    def check_path(self):
        if self.globalPathList is None or self.globalPathList.shape[0] == 0:
            return -1
        if self.dwaMidposIndex is not None and self.dwaMidposIndex >= 0:
            midindex = self.dwaMidposIndex
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
    