#!/usr/bin/env python3


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
from laser_line_extraction.msg import LineSegmentList
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from scipy import linalg


'''
Project: extended kalman filter homework AMR
Authors: Ruzic Nikola, Rodic Filip
Date June, 2023
File kalman_node.py
Contains:
    kalmanNODE
Description:
    Ros implementation of the kalman filter for position estimation

Use:
    Launch the line fitting map extractor node with:
    roslaunch laser_line_extraction example.launch
    This script depends on the extracted wall features
'''

class kalmanNODE():

    def __init__(self):
        
        rospy.init_node("kalman_node")
        
        self.sub_current = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.sub_measure = rospy.Subscriber("/line_segments", LineSegmentList, self.laser_cb)
        
        self.process_rate = 100

        self.Z = []
        self.R = []
        self.M_loaded = 0
        self.M = any

        self.X_odom = np.empty((3,1))
        self.P_odom = np.empty((3,3))
        self.U = np.zeros((2,1))
        self.b = 0.16
        self.g = 0.11
        self.v = 0
        self.w = 0

        self.X = np.array([[0],[0],[0]])
        self.P = np.eye(3)/100

    def transition_function(self, x1, P1, U, b):

        delta_sl = U[0]
        delta_sr = U[1]

        phase_const = (delta_sl - delta_sr)/2/b
        scale_const = (delta_sl + delta_sr)/2
        teta = x1[2]
        
        # numpy fuckery
        teta = float(teta)

        x_pred = x1 + np.array([scale_const*np.cos(teta + phase_const), 
                                scale_const*np.sin(teta + phase_const),
                                phase_const], dtype=object).reshape(3,1)
        
        teta_novo = x_pred[2]

        # numpy fuckery
        teta_novo = float(teta_novo)

        
        Fx = np.array([[1, 0, - scale_const*np.sin(teta_novo + phase_const)],
                       [0, 1,   scale_const*np.cos(teta_novo + phase_const)],
                       [0, 0,                         1                    ]],dtype=object)
        
        Fu = np.zeros([3,2])
        Fu[0,0] = np.cos(teta_novo + phase_const)/2 + np.sin(teta_novo + phase_const)*scale_const/2/b
        Fu[0,1] = np.cos(teta_novo + phase_const)/2 - np.sin(teta_novo + phase_const)*scale_const/2/b

        Fu[1,0] = np.sin(teta_novo + phase_const)/2 - np.cos(teta_novo + phase_const)*scale_const/2/b
        Fu[1,1] = np.sin(teta_novo + phase_const)/2 + np.cos(teta_novo + phase_const)*scale_const/2/b

        Fu[2,0] = -1/b
        Fu[2,1] =  1/b

        k = 0.03
        Q = np.array([[k*np.abs(delta_sl), 0], [0, k*np.abs(delta_sr)]], dtype=object)

        P_pred = np.dot(np.dot(Fx,P1),np.transpose(Fx)) + np.dot(np.dot(Fu,Q),np.transpose(Fu))

        return x_pred, P_pred
    
    def measurement_function(self,x_pred, mi):
        [x, y, teta] = x_pred
        [rho, alpha] = mi

        z_pred = np.array([alpha - teta, rho - (x*np.cos(alpha) + y*np.sin(alpha))])
        H_pred = np.array([[0, 0, -1],[-np.cos(alpha), -np.sin(alpha), 0]])

        return  z_pred, H_pred
    
    def associate_measurements(self,x_pred, P_pred, Z, R, M, g):
        
        H_pred = np.empty((np.shape(M)[1],2,3)) # N x 2 x 3, x3 jer se mnozi sa P_pred,
                                                # N merenja, 2 dva parametra
        V = np.empty((np.shape(M)[1],np.shape(Z)[1],2)) # N merenja, 2 parametra X2
        Sgm = np.empty((np.shape(M)[1],np.shape(Z)[1],2,2))

        for i in range(np.shape(M)[1]):
            [z_pred, H_predi] = self.measurement_function(x_pred,M[:,i])
            H_pred[i, :, :] = H_predi
            for j in range(np.shape(Z)[1]):
                z = Z[:,j].reshape(2,1) # u format z_pred [2,1]
                V[i,j,:] = np.ravel(z-z_pred)
                Sgm[i,j,:] = np.dot(np.dot(H_predi,P_pred),np.transpose(H_predi)) + R[j,:,:]

        V_out = []
        H_out = []
        R_out = []

        for i in range(np.shape(V)[0]):
            for j in range(np.shape(V)[1]):
                dtij = np.dot(np.dot(np.transpose(V[i,j,:]),np.linalg.inv(Sgm[i,j,:,:])), V[i,j,:])
                if (dtij < g*g):
                    V_out.append(V[i,j,:])
                    H_out.append(H_pred[i,:,:])
                    R_out.append(R[j,:,:])
        
        V_out, H_out, R_out = self.as_np_array(V_out,H_out,R_out)

    def as_np_array(self,V,H,R):
        return np.array(V,dtype=float), np.array(H,dtype=float), np.array(R,dtype=float)

    def filter_step(self, x_pred, P_pred, V_pred, H_pred, R_pred):
        P_pred = P_pred.astype(float)
        R = linalg.block_diag(*R_pred)      # 2k x 2k
        H = np.reshape(H_pred, (-1,3))      # 2k x 3
        V = np.reshape(V_pred, (-1,1))      # 2k x 1

        S = np.dot(np.dot(H,P_pred),np.transpose(H)) + R
        K = np.dot(np.dot(P_pred,np.transpose(H)),np.linalg.inv(S))
        
        x_t = x_pred + np.dot(K,V)
        P_t = np.dot((np.eye(3) - np.dot(K,H)),P_pred)

        return x_t, P_t

    def odom_cb(self,data):

        ids = [0,1,5,6,7,11,30,31,35]
        cov = data.pose.covariance

        _P_odom = np.array(cov)
        self.P_odom = _P_odom[ids].reshape((3,3))

        pose = data.pose.pose.position
        orient = data.pose.pose.orientation

        _,_,theta = euler_from_quaternion([orient.x,orient.y,orient.z,orient.w])
        self.X_odom = np.transpose(np.array([pose.x,pose.y,theta]))

        self.v = data.twist.twist.linear.x
        self.w = data.twist.twist.angular.z

        self.calc_U_odom()

    def calc_U_odom(self):
        T = self.process_rate/1000

        vl = self.v - self.w*self.b
        vr = self.v + self.w*self.b

        self.U[0] = self.U[0] + T * vl
        self.U[1] = self.U[1] + T * vr

    
    def laser_cb(self,data):

        lines = data.line_segments
        
        _Z = []
        _R = []

        for i, line in enumerate(lines):
            _Z.append(np.array([line.angle,line.radius]))
            cov = np.asarray(line.covariance)
            _R.append(cov.reshape((2,2)))

        self.Z = np.transpose(np.array(_Z))
        self.R = np.array(_R)
        if(self.M_loaded == 0):
            self.M_loaded = 1


    def angle_correction(self,ang):
        
        if ang > np.pi:
            _ang = np.round(ang/(np.pi*2))
            print(ang -np.pi*2*_ang)
            return ang -np.pi*2*_ang
        if ang <(-np.pi):
            _ang = np.round(-ang/(np.pi*2))
            print(ang + np.pi*2*_ang)
            return ang + np.pi*2*_ang
        
        return ang

    def run(self):
        rospy.loginfo("Starting kalmanNODE")

        self.main()

    def main(self):

        while self.M_loaded == 0:
            self.M = []

        self.M = self.Z


        r = rospy.Rate(self.process_rate)
        while not rospy.is_shutdown():
            
            X_prev = self.X
            P_prev = self.P

            [xt,Pt] = self.transition_function(X_prev,P_prev, self.U, self.b)
            a = self.associate_measurements(xt, Pt, self.Z,self.R,self.M,self.g)
            if a == None:
                V_out = []
                H_out = []
                R_out = []
            else:
                [V_out,H_out,R_out] = a
            self.X,self.P = self.filter_step(xt, Pt, V_out, H_out, R_out)

            self.X[2] = self.angle_correction(self.X[2])

            print("Pos odom")
            print(self.X_odom)
            print("Pos kalman")
            print(self.X)

            print("Kov odom")
            print(self.P_odom)
            print("Kov kalman")
            print(self.P)

            r.sleep()

if __name__ == '__main__':
    kN = kalmanNODE()
    kN.run()
   
