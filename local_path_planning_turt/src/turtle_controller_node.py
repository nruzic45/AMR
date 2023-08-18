#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

    
class controlNode():

    def __init__(self):
        
        rospy.init_node("control_node")
        
        self.sub_current = rospy.Subscriber("/odom", Odometry, self.odometryCallback)
        self.pub_command = rospy.Publisher("/cmd_vel", Twist, queue_size = 2)
        self.sub_auto = rospy.Subscriber("/turtle_commands/command", String, self.getInfo)
       
        self.process_rate = 100
        self.control_mode = ''
        self.manual_mode = ''
        self.theta_curr = 0
        self.x_curr = 0
        self.y_curr = 0
        self.x_goal = 0
        self.y_goal = 0


    def getInfo(self, data):
        s = data.data.split(",")
        print(s)
        self.control_mode = s[0]

        if self.control_mode == 'a':
            self.x_goal = float(s[1])
            self.y_goal = float(s[2])
        elif self.control_mode == 'm':
            self.manual_mode = s[1]
        else:
            print("Invalid command")
        
    
    def odometryCallback(self, dataOdom): 
        self.x_curr = dataOdom.pose.pose.position.x
        self.y_curr = dataOdom.pose.pose.position.y
        orientation_q = dataOdom.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.theta_curr = yaw

    def auto(self):

        k_rho = 0.3
        k_alpha = 0.5
        k_beta = -0.1

        d_x = self.x_goal - self.x_curr
        d_y = self.y_goal - self.y_curr

        rho = math.sqrt(math.pow(d_x,2) + math.pow(d_y,2))
        alpha = -self.theta_curr + math.atan2(d_y,d_x)

        beta = -self.theta_curr - alpha

        alpha = self.to_fir_for_quadrant(alpha)
        beta = self.to_fir_for_quadrant(beta)

        v = k_rho * rho
        w = k_alpha * alpha + k_beta * beta 

        if alpha < -math.pi/2 or alpha > math.pi/2:
            v *= -1
            w = k_alpha * self.to_fir_for_quadrant(alpha + math.pi) + k_beta * self.to_fir_for_quadrant(beta + math.pi)

            
        if rho < 0.5:
            constant = v/w
            if v != 0:
                v = 0.3 * v/abs(v)
            if w != 0:
                w = 0.3/constant * w/abs(w)
        

        if rho < 0.1:
            v = 0
            w = 0

        pub_cmd = Twist()
        pub_cmd.linear.x = v 
        pub_cmd.linear.y = 0
        pub_cmd.linear.z = 0
        pub_cmd.angular.x = 0
        pub_cmd.angular.y = 0
        pub_cmd.angular.z = w


        #Publish data
        self.pub_command.publish(pub_cmd)  
            
    def man(self):
        pub_cmd = Twist()
        speed = 0.5
        angular_speed = 1

        if self.manual_mode == 'f':
            pub_cmd.linear.x = speed 
            pub_cmd.linear.y = 0
            pub_cmd.linear.z = 0
            pub_cmd.angular.x = 0
            pub_cmd.angular.y = 0
            pub_cmd.angular.z = 0
        elif self.manual_mode == 'b':
            pub_cmd.linear.x = -speed 
            pub_cmd.linear.y = 0
            pub_cmd.linear.z = 0
            pub_cmd.angular.x = 0
            pub_cmd.angular.y = 0
            pub_cmd.angular.z = 0
        elif self.manual_mode == 's':
            pub_cmd.linear.x = 0
            pub_cmd.linear.y = 0
            pub_cmd.linear.z = 0
            pub_cmd.angular.x = 0
            pub_cmd.angular.y = 0
            pub_cmd.angular.z = 0
        elif self.manual_mode == 'cw':
            pub_cmd.linear.x = 0
            pub_cmd.linear.y = 0
            pub_cmd.linear.z = 0
            pub_cmd.angular.x = 0
            pub_cmd.angular.y = 0
            pub_cmd.angular.z = angular_speed
        elif self.manual_mode == 'ccw':
            pub_cmd.linear.x = 0
            pub_cmd.linear.y = 0
            pub_cmd.linear.z = 0
            pub_cmd.angular.x = 0
            pub_cmd.angular.y = 0
            pub_cmd.angular.z = -angular_speed
        
        
        self.pub_command.publish(pub_cmd)

    def to_fir_for_quadrant(self,angle):
        while(angle > math.pi):
            angle -= 2*math.pi
        while(angle < -math.pi):
            angle += 2*math.pi

        return angle


    def run(self):
        rospy.loginfo("Starting controlNode")

        self.send_info()

    def send_info(self):

        r = rospy.Rate(self.process_rate)
        while not rospy.is_shutdown():
            if self.control_mode == "a":
                self.auto()
            if self.control_mode == "m":
                self.man()
                    
            r.sleep()

if __name__ == '__main__':
    cN = controlNode()
    try:
        cN.run()
    except ROSInterruptException:
        pass

