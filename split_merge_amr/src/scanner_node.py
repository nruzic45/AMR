#!/usr/bin/env python3


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import numpy as np
import math as m
import time

'''
Project: split_merge homework AMR
Authors: Ruzic Nikola, Rodic Filip
Date May, 2023
File scanner_node.py
Contains:
    scanNODE
Description:
    Ros implementation of the split&merge algorithm
'''

class scanNODE():

    def __init__(self):
        
        rospy.init_node("scanNODE")
        
        # self.pub_command        = rospy.Publisher("/yolo_pos", String, queue_size = 10)
        self.sub_laser_scan     = rospy.Subscriber("/scan", LaserScan, self.getInfo)

        # iterative mode - 0
        # recursive mode - 1
        self.mode = 1

        self.threshold = 0.07

        self.info = LaserScan()
        self.process_rate = 10


    ###################################################
    # Polar coordinate calculations
    ###################################################
    
    def calc_rho_alpha(self,X,Y):
        X = X-np.mean(X)
        Y = Y-np.mean(Y)
        
        if (X.any()) or (Y.any()):
            k,n = np.polyfit(X,Y,1)
            alpha = m.atan(-1/k)
            ro = n/(m.sin(alpha)-k*m.cos(alpha))
            return ro,alpha
        else:
            return 0,0
    
    def rho_alpha_correction(self,ro,alpha):
        if ro<0:
            alpha = alpha + m.pi
            if alpha > m.pi:
                alpha = alpha - 2*m.pi
            ro = -ro
        return ro,alpha
    
    def Pol2C(self,r,alpha):
        if (alpha.any()) or (r.any()):
            return np.transpose(np.array([np.cos(alpha)*r, np.sin(alpha)*r]))
        else:
            pass

    def Car2P(self,x,y):
        r = np.sqrt(x**2 + y**2)
        phi = np.arctan2(y, x)
        return r, phi

    ###################################################
    # Distance calculations
    ###################################################
    def dist(self,P,Ps,Pe):
        if np.all(np.equal(Ps,Pe)):
            return np.linalg.norm(P-Ps)
        else:
            return np.divide(np.abs(np.linalg.norm(np.cross(Pe-Ps,Ps-P))),
                             np.linalg.norm(Pe-Ps))
    
    def max_dist(self,P):
        dmax = 0
        ind = -1
        for i in range(1, P.shape[0]):
            d = self.dist(P[i,:],P[0,:],P[-1,:])
            if (d>dmax):
                ind = i
                dmax = d
        return dmax,ind

    ###################################################
    # S&M algorithm implementation
    ###################################################

    # Iterative #######################################
    
    def SM_I(self,P,threshold):
        stack = []
        stack.append((P, True))  # True indicates splitting, False indicates merging

        points = []
        
        while stack:
            P, is_splitting = stack.pop()
            
            if is_splitting:
                d, ind = self.max_dist(P)
                if d > threshold:
                    stack.append((P[ind:], True))
                    stack.append((P[:ind + 1, :], True))
                else:
                    stack.append((P, False))
            else:
                if len(P) > 1:
                    points.append(P[0])
                    points.append(P[-1])
                elif len(P) == 1:
                    points.append(P[0])
        
        return np.vstack(points)
    
    # Recursive ##########################################
    
    def SM(self,P,th):
        d,ind = self.max_dist(P)
        if (d>th):
            P_left = self.SM(P[:ind+1,:],th)
            P_right = self.SM(P[ind:,:],th)

            points = np.vstack((P_left[:-1,:],P_right))
        else:
            points = np.vstack((P[0,:],P[-1,:]))


        return points
    
    # Function caller #####################################
    
    def split_merge(self):
        time.sleep(1)
        vr1 = time.time()
      
        ranges = np.array(self.info.ranges)
        alpha = np.linspace(self.info.angle_min, self.info.angle_max,360)
   
        ranges[ranges==np.inf] = 3
        P = self.Pol2C(ranges,alpha)

        if self.mode == 1:
            points = self.SM(P,self.threshold)
        elif self.mode == 0:
            points = self.SM_I(P,self.threshold)
        else:
            print("incorrect mode")

        for i in range(points.shape[0]-1):
            ro,alpha = self.calc_rho_alpha(points[i:i+2,0],points[i:i+2,1])
            ro,alpha = self.rho_alpha_correction(ro,alpha)
            print("ro,alpha={},{}".format(ro,alpha))


        vr2 = time.time()
        print("time:" + str(vr2 - vr1))
    


    def getInfo(self, data):
        self.info = data
        


    def run(self):
        rospy.loginfo("Starting Scan_Node")

        self.main()

    def main(self):

        r = rospy.Rate(self.process_rate)
        while not rospy.is_shutdown():
            if self.mode == 0:
                print("iterative split_merge")
                self.split_merge()
            elif self.mode == 1:
                print("recursive split_merge")
                
                self.split_merge()

            r.sleep()

if __name__ == '__main__':
    sN = scanNODE()
    # try:
    sN.run()
    # except Exception as e:
    #     print("Error: ")
    #     print(e)
        