#!/usr/bin/env python

"""
ApplicationService is AS of the digital twin based on the WMR.
The AS and the DP are in this file.
Function of this file is as follows:
* Subscribe states topics from physical WMR and virtual WMR.
* Store states of the DT in database.
* Publish twist topics for controlling physical WMR and virtual WMR.
Note that PWMR = P, VWMR = V in understanding.
"""
    
import rospy
import sqlite3
import numpy as np
from math import *
import os
import sys
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from digital_twin3.msg import DT_WMR
import control

sys.path.append(os.path.dirname(os.path.abspath(__file__))+"/../..")

class ApplicationService:
    '''ApplicationSerivce class of the digital twin based on wheeled mobile robot.

    Functions of this class contain communication of PWMR and VWMR by ROS, data storing, 
    outer loop controller for PWMR and VWMR, inner loop controller for PWMR.
    '''

    def __init__(self):
        self.R = 0.05
        self.Lx = 0.156
        self.Ly = 0.176
        self.m = 6.1                     # kg
        self.Iz = 0.08                   # Moment of inertia
        self.Iw = 0.0005                 # Moment of inertia
        self.timestep = 0.1              # timestep
        self.time = 0                    # current time
        self.torque2pwm = 4000
        self.Ref_Q = [0, 0, 0]
        self.Ref_V = [0, 0, 0]           # Ref_vx, Ref_vy, Ref_vth
        self.keyCtrl_V = [0, 0, 0] 
        self.P_initPos = [0, 0, 0]       # P_x0, P_y0, P_th0
        self.P_name = "PWMRrobot"
        self.P_SOC = 0
        self.P_Q = [0, 0, 0]             # P_x, P_y, P_th
        self.P_V = [0, 0, 0]             # P_vx, P_vy, P_vth
        self.P_Vw = [0, 0, 0, 0]         # P_vw1, P_vw2, P_vw3, P_vw4,
        self.P_U = [0, 0, 0]             # P_ux, P_uy, P_uth                 : desired body velocities
        self.P_Uw = [0, 0, 0, 0]         # P_uw1, P_uw2, P_uw3, P_uw4        : desired wheel angular velocities.
        self.P_tau = [0, 0, 0, 0]        # P_tau1, P_tau2, P_tau3, P_tau4    : desired wheel torques.
        self.P_lamda = [0, 0, 0, 0]
        self.P_TD1 = [0, 0, 0, 0]
        self.P_TD2 = [0, 0, 0, 0]
        self.P_ESO1 = [0, 0, 0, 0]
        self.P_ESO2 = [0, 0, 0, 0]
        self.P_ISMC_inte = [0, 0, 0, 0]
        self.V_Q = [0, 0, 0]             # V_x, V_y, V_th
        self.V_V = [0, 0, 0]             # V_vx, V_vy, V_vth
        self.V_U = [0, 0, 0]             # V_ux, V_uy, V_uth

        self.pose_flag = 1
        self.angular_flag = 1
        self.Start_flag = 0

        self._pub = rospy.Publisher('/dt_monitor', DT_WMR, queue_size=10)
        self._P_VelPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self._V_VelPub = rospy.Publisher('/controller2', Point, queue_size=10)

        rospy.init_node('ApplicationService', anonymous=True)
        
        # clear the historical database eVry time the AS is started
        db_file = os.path.dirname(os.path.abspath(__file__))+'/DT_data.db'
        # print(db_file)
        
        if os.path.isfile(db_file):
            os.remove(db_file)
            print("Initialize datebase sucessfully.")
        
        # connect the database
        self.conn = sqlite3.connect(db_file)
        self.cursor = self.conn.cursor()

        # create data table
        self.cursor.execute('''
            create table if not exists DT_states(
                Ref_x real,
                Ref_y real,
                Ref_th real,
                Ref_vx real,
                Ref_vy real,
                Ref_vth real,

                V_x real,
                V_y real,
                V_th real,
                V_vx real,
                V_vy real,
                V_vth real,
                V_ux real,
                V_uy real,
                V_uth real,

                P_SOC real,
                P_x real,
                P_y real,
                P_th real,

                P_vx real,
                P_vy real,
                P_vth real,
                P_vw1 real,
                P_vw2 real,
                P_vw3 real,
                P_vw4 real,

                P_ux real,
                P_uy real,
                P_uth real,
                P_uw1 real,
                P_uw2 real,
                P_uw3 real,
                P_uw4 real,

                ESO_z1_1 real,
                ESO_z1_2 real,
                ESO_z1_3 real,
                ESO_z1_4 real,
                ESO_z2_1 real,
                ESO_z2_2 real,
                ESO_z2_3 real,
                ESO_z2_4 real,

                TD1_1 real,
                TD1_2 real,
                TD1_3 real,
                TD1_4 real,
                TD2_1 real,
                TD2_2 real,
                TD2_3 real,
                TD2_4 real,
                
                P_tau1 real,
                P_tau2 real,
                P_tau3 real,
                P_tau4 real) 
        ''')


    def __del__( self ):
        self.cursor.close()
        self.conn.close()
        

    def dataSave(self):
        sql = 'insert into DT_states values (?, ?, ?,   ?, ?, ?,\
                                                ?, ?, ?,   ?, ?, ?,   ?, ?, ?,\
                                                ?, ?, ?, ?,\
                                                ?, ?, ?,   ?, ?, ?, ?,\
                                                ?, ?, ?,   ?, ?, ?, ?,\
                                                ?, ?, ?, ?,   ?, ?, ?, ?,\
                                                ?, ?, ?, ?,   ?, ?, ?, ?,\
                                                ?, ?, ?, ?)'
        dtStates = (self.Ref_Q[0], self.Ref_Q[1], self.Ref_Q[2],
                    self.Ref_V[0], self.Ref_V[1], self.Ref_V[2],
                    self.V_Q[0], self.V_Q[1], self.V_Q[2], 
                    self.V_V[0], self.V_V[1], self.V_V[2], 
                    self.V_U[0], self.V_U[1], self.V_U[2],
                    self.P_SOC, self.P_Q[0], self.P_Q[1], self.P_Q[2],
                    self.P_V[0], self.P_V[1], self.P_V[2], 
                    self.P_Vw[0], self.P_Vw[1], self.P_Vw[2], self.P_Vw[3],
                    self.P_U[0], self.P_U[1], self.P_U[2],
                    self.P_Uw[0], self.P_Uw[1], self.P_Uw[2], self.P_Uw[3],
                    self.P_ESO1[0], self.P_ESO1[1], self.P_ESO1[2], self.P_ESO1[3],
                    self.P_ESO2[0], self.P_ESO2[1], self.P_ESO2[2], self.P_ESO2[3],
                    self.P_TD1[0], self.P_TD1[1], self.P_TD1[2], self.P_TD1[3],
                    self.P_TD2[0], self.P_TD2[1], self.P_TD2[2], self.P_TD2[3],
                    self.P_tau[0], self.P_tau[1], self.P_tau[2], self.P_tau[3])
       
        self.cursor.execute(sql, dtStates)
        self.conn.commit()

    
    def dt_monitor_pub(self):
        msg = DT_WMR()
        msg.header.stamp = rospy.Time.now()
        msg.PWMR_name = self.P_name
        
        msg.Ref_pos.x = self.Ref_Q[0]
        msg.Ref_pos.y = self.Ref_Q[1]
        msg.Ref_pos.z = self.Ref_Q[2]
        msg.Ref_vel.x = self.Ref_V[0]
        msg.Ref_vel.y = self.Ref_V[1]
        msg.Ref_vel.z = self.Ref_V[2]

        msg.VWMR_pos.x = self.V_Q[0]
        msg.VWMR_pos.y = self.V_Q[1]
        msg.VWMR_pos.z = self.V_Q[2]
        msg.VWMR_vel.x = self.V_V[0]
        msg.VWMR_vel.y = self.V_V[1]
        msg.VWMR_vel.z = self.V_V[2]
        msg.VWMR_desired_vel.x = self.V_U[0]
        msg.VWMR_desired_vel.y = self.V_U[1]
        msg.VWMR_desired_vel.z = self.V_U[2]

        msg.PWMR_SOC = self.P_SOC
        msg.PWMR_pos.x = self.P_Q[0]
        msg.PWMR_pos.y = self.P_Q[1]
        msg.PWMR_pos.z = self.P_Q[2]

        msg.PWMR_vel.x = self.P_V[0]
        msg.PWMR_vel.y = self.P_V[1]
        msg.PWMR_vel.z = self.P_V[2]
        msg.PWMR_wheel_ang_vel.w1 = self.P_Vw[0]
        msg.PWMR_wheel_ang_vel.w2 = self.P_Vw[1]
        msg.PWMR_wheel_ang_vel.w3 = self.P_Vw[2]
        msg.PWMR_wheel_ang_vel.w4 = self.P_Vw[3]

        msg.PWMR_desired_vel.x = self.P_U[0]
        msg.PWMR_desired_vel.y = self.P_U[1]
        msg.PWMR_desired_vel.z = self.P_U[2]
        msg.PWMR_desired_wheel_ang_vel.w1 = self.P_Uw[0]
        msg.PWMR_desired_wheel_ang_vel.w2 = self.P_Uw[1]
        msg.PWMR_desired_wheel_ang_vel.w3 = self.P_Uw[2]
        msg.PWMR_desired_wheel_ang_vel.w4 = self.P_Uw[3]

        msg.PWMR_wheel_torque.w1 = self.P_tau[0]
        msg.PWMR_wheel_torque.w2 = self.P_tau[1]
        msg.PWMR_wheel_torque.w3 = self.P_tau[2]
        msg.PWMR_wheel_torque.w4 = self.P_tau[3]

        msg.TD1.w1 = self.P_TD1[0]
        msg.TD1.w2 = self.P_TD1[1]
        msg.TD1.w3 = self.P_TD1[2]
        msg.TD1.w4 = self.P_TD1[3]
        msg.TD2.w1 = self.P_TD2[0]
        msg.TD2.w2 = self.P_TD2[1]
        msg.TD2.w3 = self.P_TD2[2]
        msg.TD2.w4 = self.P_TD2[3]

        msg.ESO1.w1 = self.P_ESO1[0]
        msg.ESO1.w2 = self.P_ESO1[1]
        msg.ESO1.w3 = self.P_ESO1[2]
        msg.ESO1.w4 = self.P_ESO1[3]
        msg.ESO2.w1 = self.P_ESO2[0]
        msg.ESO2.w2 = self.P_ESO2[1]
        msg.ESO2.w3 = self.P_ESO2[2]
        msg.ESO2.w4 = self.P_ESO2[3]

        self._pub.publish(msg)


    def trajectory_planing(self,num):
        if num == 1:  # circular, vy = 0
            self.Ref_V[0] = 0.3
            self.Ref_V[2] = 0.2
            self.Ref_Q[2] += self.timestep * self.Ref_V[2]
            self.Ref_Q[2] = self.Limit_angular(self.Ref_Q[2])
            self.Ref_Q[0] += self.timestep * self.Ref_V[0] * cos(self.Ref_Q[2])
            self.Ref_Q[1] += self.timestep * self.Ref_V[0] * sin(self.Ref_Q[2])
            
        if num == 2:  # circular, vth = 0
            self.Ref_V[0] = 0.2
            self.Ref_V[1] = 0.3
            self.Ref_Q[0] = 2 * sin(0.25 * self.time)
            self.Ref_Q[1] = 2 * sin(0.5  * self.time)

        if num == 3:  # lemniscates, vth = 0
            # temp_x = self.P_Q[0] + 0.2 + 1.5 * sin(0.15 * self.time)
            # temp_y = self.P_Q[1] + 0.2 + 1.5 * sin(0.3 * self.time)
            temp_x = 1.5 * sin(0.15 * self.time)
            temp_y = 1.5 * sin(0.3 * self.time)

            self.Ref_V[0] = (temp_x - self.Ref_Q[0]) / self.timestep
            self.Ref_V[1] = (temp_y - self.Ref_Q[1]) / self.timestep

            self.Ref_Q[0] = temp_x
            self.Ref_Q[1] = temp_y

        if num == 0:  # keyboard control
            if self.time < 2:
                self.Ref_V[0] = 0.4
            elif self.time < 2.5:
                self.Ref_V = [0,0,0]

            elif self.time < 4.5:
                self.Ref_V[1] = 0.4
            elif self.time < 5:
                self.Ref_V = [0,0,0]

            elif self.time < 5.5:
                self.Ref_V[2] = 0.5


    def process(self):   
        rospy.Subscriber("/PowerVoltage", Float32, self.P_SOC_callback)
        rospy.Subscriber("/imu", Imu, self.P_imu_callback)
        rospy.Subscriber("/UWB_pose", Point, self.P_uwb_callback)
        rospy.Subscriber("/odom", Odometry, self.P_odem_callback)
        rospy.Subscriber("/dt_robot/odom", Odometry, self.V_odem_callback)
        rospy.Subscriber("/Cmd_vel", Twist, self.V_keyboard_callback)

        self.Start_flag = 1 # int(input("Start control?  1 or 0 \n"))
        
        rate = rospy.Rate(1/self.timestep)  # 10hz
        while not rospy.is_shutdown():
            if self.Start_flag:
                self.trajectory_planing(1)
                
                self.VWMR_Control(IsOpenLoop= 0, # bool. keyboard control webots robot
                                  kineController= 1, # int. trajectory tracking control algorithm
                                  desired_pos= self.Ref_Q, 
                                  desired_vel= self.Ref_V)
                
                # self.PWMR_Control(kineController=2, desired_pos=self.V_Q, desired_vel=self.V_V)
                
                self.dataSave()
                self.time += self.timestep
            
            self.dt_monitor_pub()
            rate.sleep()


    def PWMR_Control(self, kineController, desired_pos=[], desired_vel=[]):
        '''Publish wheel velocity control topic to the PWMR.
        Function: error convertion, kinematic controller, ESO, TD, ISMC.
        '''
        if kineController == 1:
            ## Backstepping controller
            # pos error in body coordinate system
            P_ex =  (desired_pos[0] - self.P_Q[0]) * cos(self.P_Q[2]) + (desired_pos[1] - self.P_Q[1]) * sin(self.P_Q[2])
            P_ey = -(desired_pos[0] - self.P_Q[0]) * sin(self.P_Q[2]) + (desired_pos[1] - self.P_Q[1]) * cos(self.P_Q[2])
            P_eth = desired_pos[2] - self.P_Q[2]

            K1,K2,K3 = 0.98,1.9,0.8
            # desired body velocities obtained by Kinematic controller
            self.P_U[0] = desired_vel[0] * cos(P_eth) - desired_vel[1] * sin(P_eth) + K1 * P_ex
            self.P_U[1] = desired_vel[0] * sin(P_eth) + desired_vel[1] * cos(P_eth) + K2 * P_ey
            self.P_U[2] = desired_vel[2] + K3 * sin(P_eth)
        
        elif kineController == 2:
            ## LQR controller
            Ax = -desired_vel[0]*sin(desired_pos[2])-desired_vel[1]*cos(desired_pos[2])
            Ay =  desired_vel[0]*cos(desired_pos[2])-desired_vel[1]*sin(desired_pos[2])
            A = np.array([[0,0,Ax], [0,0,Ay], [0,0,0]])
            B = np.array([[cos(desired_pos[2]),-sin(desired_pos[2]),0], [sin(desired_pos[2]),cos(desired_pos[2]),0], [0,0,1]])
            Q = np.diag([1, 1, 20])
            R = np.diag([1, 1, 5])

            K, X, E = control.lqr(A, B, Q, R)
            
            EQ = np.zeros((3,1))
            
            for i in range(3):
                EQ[i,0] = self.P_Q[i] - desired_pos[i]
            
            U = -np.dot(K, EQ)

            for i in range(3):
                self.P_U[i] = U[i, 0] + desired_vel[i]

        else:
            self.P_U = [0, 0, 0]


        # test
        self.P_U = [0.0, 0.4, 0.0]

        # desired wheel angular velocities
        self.P_Uw[0] =  (self.P_U[0] + self.P_U[1] - (self.Lx + self.Ly) * self.P_U[2]) / self.R
        self.P_Uw[1] =  (self.P_U[0] - self.P_U[1] - (self.Lx + self.Ly) * self.P_U[2]) / self.R
        self.P_Uw[2] =  (self.P_U[0] + self.P_U[1] + (self.Lx + self.Ly) * self.P_U[2]) / self.R
        self.P_Uw[3] =  (self.P_U[0] - self.P_U[1] + (self.Lx + self.Ly) * self.P_U[2]) / self.R

        self.wheel_torque_control(if_M= 0, isISMC= 1 , isPID= 0)


    def wheel_torque_control(self,isISMC, isPID, if_M):
        for i in range(4):  
            # TD: best R = 12.
            R = 11.5
            self.P_TD1[i] += self.timestep*self.P_TD2[i]
            self.P_TD2[i] += self.timestep*(-R*R*(asinh(self.P_TD1[i]-self.P_Uw[i]) + asinh(self.P_TD2[i]/R)))

            # ESO:  10，25
            b1,b2 = 10, 25
            ESOe = self.P_ESO1[i] - self.P_Vw[i]
            self.P_ESO1[i] += self.timestep * (self.P_ESO2[i] - b1*ESOe + self.P_Vw[i])
            self.P_ESO2[i] += self.timestep * (- b2*self.fal(ESOe, 0.8, 1.5))
            # self.P_ESO2[i] = 0

            # innerController:  17, 20, 3
            n1,n2,n3 = 17, 21, 3
            ISMCe = self.P_Uw[i] - self.P_Vw[i]
            self.P_ISMC_inte[i] += self.timestep*self.fal(ISMCe, 0.8, 1.3)
            ISMCs = ISMCe + n1*self.P_ISMC_inte[i]
            
            # with M converting control laws
            if if_M:
                # ISMC controller
                if isISMC:
                    # self.P_lamda[i] = self.P_TD2[i] - self.P_ESO2[i] + n1* \
                    #     self.fal(ISMCe, 0.5, 0.5) + n2*tanh(ISMCs) + n3*ISMCs
                    
                    self.P_lamda[i] =n1*self.fal(ISMCe, 0.5, 1.5)+ n2*tanh(ISMCs)+\
                        n3*ISMCs- self.P_ESO2[i]+ self.P_TD2[i] * 0.01
                
                # PID controller
                if isPID:
                    self.P_lamda[i] = ISMCe * 20 + self.P_ISMC_inte[i] * 60 \
                        + self.P_TD2[i] * 2 - self.P_ESO2[i]
                
                # M convert
                A = self.m*self.R*self.R / 8
                B = self.Iz*self.R*self.R / (16*(self.Lx + self.Ly)*(self.Lx + self.Ly))
                C = A + B + self.Iw
                Matrix = np.array([[  C, -B,   B, A-B],
                                [ -B,  C, A-B,   B],
                                [  B,A-B,   C,  -B],
                                [A-B,  B,  -B,   C]])
                UI = np.array([[self.P_lamda[0]],[self.P_lamda[1]],[self.P_lamda[2]],[self.P_lamda[3]]])
                TAU = np.dot(Matrix, UI)
                (self.P_tau[0],self.P_tau[1],self.P_tau[2],self.P_tau[3])=(TAU[0,0],TAU[1,0],TAU[2,0],TAU[3,0])

            # without M converting control laws
            if not if_M:
                self.P_tau[i] = ISMCe * 0.04 + self.P_ISMC_inte[i] * 0.4

        # publish desired wheel angular velocities by Twist msg type
        Tau = Twist()
        pwmLimit = 4000
        Tau.linear.x  = self.Limit(self.torque2pwm*self.P_tau[0], pwmLimit)      # STM32: Move_A
        Tau.linear.y  = self.Limit(self.torque2pwm*self.P_tau[1], pwmLimit)      # STM32: Move_B
        Tau.angular.x = self.Limit(self.torque2pwm*self.P_tau[2], pwmLimit)      # STM32: Move_C
        Tau.angular.y = self.Limit(self.torque2pwm*self.P_tau[3], pwmLimit)      # STM32: Move_D
        
        if self.time > 5:
            Tau.linear.x  = 0
            Tau.linear.y  = 0
            Tau.angular.x = 0
            Tau.angular.y = 0
        
        self._P_VelPub.publish(Tau)


    def VWMR_Control(self,IsOpenLoop, kineController, desired_pos=[], desired_vel=[]):
        '''Publish body velocity control topic to the VWMR.
        Function: error convertion, kinematic controller.
        '''
        if kineController == 1:
            V_ex =  (desired_pos[0] - self.V_Q[0])*cos(self.V_Q[2]) + (desired_pos[1] - self.V_Q[1])*sin(self.V_Q[2])
            V_ey = -(desired_pos[0] - self.V_Q[0])*sin(self.V_Q[2]) + (desired_pos[1] - self.V_Q[1])*cos(self.V_Q[2])
            V_eth = desired_pos[2] - self.V_Q[2]

            k1,k2,k3 = 2.2, 2.3, 1.5  # 室外双纽线(0.15,0.3) vth = 0 最佳参数 1.9, 2.3, 1.2
            # K4,K5,K6 = 2.2, 2.5, 1.5  # 室外圆轨迹(0.15,0.3) vth = 0 最佳参数 2.2, 2.5, 1.5
            
            self.V_U[0] = desired_vel[0]*cos(V_eth) - desired_vel[1]*sin(V_eth) + k1*V_ex
            self.V_U[1] = desired_vel[0]*sin(V_eth) + desired_vel[1]*cos(V_eth) + k2*V_ey
            self.V_U[2] = desired_vel[2] + k3*sin(V_eth)

        ## LQR controller
        elif kineController == 2:
            Ax = -desired_vel[0]*sin(desired_pos[2])-desired_vel[1]*cos(desired_pos[2])
            Ay =  desired_vel[0]*cos(desired_pos[2])-desired_vel[1]*sin(desired_pos[2])
            A = np.array([[0,0,Ax], [0,0,Ay], [0,0,0]])
            B = np.array([[cos(desired_pos[2]),-sin(desired_pos[2]),0], [sin(desired_pos[2]),cos(desired_pos[2]),0], [0,0,1]])
            Q = np.diag([1 , 1 , 20 ])
            R = np.diag([1 , 1 , 5 ])

            K, X, E = control.lqr(A, B, Q, R)
            
            EQ = np.zeros((3,1))
            
            for i in range(3):
                EQ[i][0] = self.V_Q[i] - desired_pos[i]
            
            U = -np.dot(K, EQ)

            for i in range(3):
                self.V_U[i] =  U[i, 0] + desired_vel[i]

        if IsOpenLoop:
            for i in range(3):
                self.V_U[i] = self.keyCtrl_V[i]

        V_msg = Point()
        V_msg.x = self.Limit(self.V_U[0], 20)
        V_msg.y = self.Limit(self.V_U[1], 20)
        V_msg.z = self.Limit(self.V_U[2], 20)
        self._V_VelPub.publish(V_msg)


    def fal(self, e, alpha, delta):
        if abs(e) <= delta:
            return e/pow(delta, 1 - alpha)
        else:
            return pow(abs(e), alpha)*np.sign(e)


    def Limit(self,val,lim):
        if val >lim:
            val = lim
            # print("Reached upper limit!")
        if val < -lim:
            val = -lim
            # print("Reached lower limit!")
        return val


    def Limit_angular(self,angular):
        while angular >= pi:
            angular -= 2*pi
        while angular <= -pi:
            angular -= 2*pi
        return angular


    def orientation_to_yaw(self, w, x , y, z):
        return atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


    def P_SOC_callback(self, data):
        self.P_SOC = data.data

    def P_imu_callback(self, data):
        w = data.orientation.w
        x = data.orientation.x
        y = data.orientation.y
        z = data.orientation.z
        th = self.orientation_to_yaw(w, x, y, z)

        if self.angular_flag == 1 and th != 0 :
            self.P_initPos[2] = th
            print("PWMR initial angular: {}".format(self.P_initPos[2]))
            self.angular_flag = 0

        self.P_Q[2] = th

    def P_odem_callback(self, data):
        self.P_V[0] = data.twist.twist.linear.x
        self.P_V[1] = data.twist.twist.linear.y
        self.P_V[2] = data.twist.twist.angular.z
        print(1)
        self.P_Vw[0] = (self.P_V[0] + self.P_V[1] - (self.Lx + self.Ly) * self.P_V[2]) / self.R
        self.P_Vw[1] = (self.P_V[0] - self.P_V[1] - (self.Lx + self.Ly) * self.P_V[2]) / self.R
        self.P_Vw[2] = (self.P_V[0] + self.P_V[1] + (self.Lx + self.Ly) * self.P_V[2]) / self.R
        self.P_Vw[3] = (self.P_V[0] - self.P_V[1] + (self.Lx + self.Ly) * self.P_V[2]) / self.R

    def P_uwb_callback(self,data):
        if self.pose_flag == 1 and data.x != 0:
            self.P_initPos[0] = data.x
            self.P_initPos[1] = data.y
            print("PWMR initial position: ({},{})".format(self.P_initPos[0], self.P_initPos[1]))
            self.pose_flag = 0
        self.P_Q[0] = data.x
        self.P_Q[1] = data.y
        print(1)

    def V_odem_callback(self, data):
        self.V_Q[0] = data.pose.pose.position.x #+ self.P_initPos[0]
        self.V_Q[1] = data.pose.pose.position.y #+ self.P_initPos[1]
        self.V_V[0] = data.twist.twist.linear.x
        self.V_V[1] = data.twist.twist.linear.y
        self.V_V[2] = data.twist.twist.angular.z
        
        Vx = data.pose.pose.orientation.x
        Vy = data.pose.pose.orientation.y
        Vz = data.pose.pose.orientation.z
        Vw = data.pose.pose.orientation.w
        self.V_Q[2] =self.orientation_to_yaw(Vw, Vx, Vy, Vz) #+ self.P_initPos[2]

    def V_keyboard_callback(self,data):
        self.keyCtrl_V[0] = data.linear.x
        self.keyCtrl_V[1] = data.linear.y
        self.keyCtrl_V[2] = data.angular.z



if __name__ == '__main__':
    try:
        AS = ApplicationService()
        AS.process()

    except rospy.ROSInterruptException:
        pass
