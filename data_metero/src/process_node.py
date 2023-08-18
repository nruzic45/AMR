#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import Float32



class processNODE():

    def __init__(self):
        
        rospy.init_node("process_node")
        
        self.sub_temp = rospy.Subscriber("/measurements/temp",String, self.get_temp)
        self.pub_action = rospy.Publisher("triggers/save_C", Bool, queue_size = 10)
        self.pub_UI = rospy.Publisher("UI/graph_data", Float32, queue_size = 10)

        self.process_rate = 10
        self.tempC = 0.0
        self.trig = False

    def get_temp(self,data):
        temperature = data.data
        temperature = temperature[2:len(temperature)-2]
        temperature = temperature.split(',')
        self.tempC = self.farrenheit_2_cel(float(temperature[3])) 
        self.trig = self.check_max(temperature)


    def farrenheit_2_cel(self, tempF):
        celsius = (tempF - 32) * 5 / 9
        return celsius

    def check_max(self, temp_row):
        
        return True if abs(self.farrenheit_2_cel(float(temp_row[1]) - float(temp_row[2]))) > 15 else False


    def run(self):
        rospy.loginfo("Starting processNODE")

        self.send_info()

    def send_info(self):

        r = rospy.Rate(self.process_rate)
        while not rospy.is_shutdown():
            if self.trig:
                self.pub_action.publish(self.trig)
            self.pub_UI.publish(self.tempC)
            r.sleep()

if __name__ == '__main__':
    pN = processNODE()

    try:
        pN.run()
    except ROSInterruptException:
        pass

