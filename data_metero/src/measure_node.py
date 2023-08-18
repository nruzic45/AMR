#!/usr/bin/env python3


#!/usr/bin/env python3

import rospy
import time

import csv

from std_msgs.msg import String


class measureNODE():
    def __init__(self):

        rospy.init_node("measure_node")
        self.pub_temp= rospy.Publisher("/measurements/temp",String,queue_size=5)
        
        self.measurement = []

        with open('src/data_metero/src/weather_data_nyc_centralpark_2016.csv', newline='') as csvfile:
                temp_reader = csv.reader(csvfile, delimiter=' ', quotechar='|')
                for row in temp_reader:
                    self.measurement.append(row)

        self.mCount = 0
        self.measure_rate=10
        
    def run(self):

        rospy.loginfo("Starting measure Node")

        self.send_temperature()
        
    def load_data(self):
        self.measurement = []
        with open('src/data_metero/src/weather_data_nyc_centralpark_2016.csv', newline='') as csvfile:
                temp_reader = csv.reader(csvfile, delimiter=' ', quotechar='|')
                for row in temp_reader:
                    self.measurement.append(row)

    def send_temperature(self):
        r = rospy.Rate(self.measure_rate)
        while not rospy.is_shutdown():

            rospy.loginfo("measurment Num:" + str(self.mCount))
            if self.mCount < len(self.measurement):
                self.pub_temp.publish(str(self.measurement[self.mCount]))
                self.mCount += 1
            else:
                print("no more measurements")
                print("Reloading Data")
                self.mCount = 0
                self.load_data()

            r.sleep()    

        
if __name__ == '__main__':
    mN = measureNODE()

    try:
        mN.run()
    except rospy.ROSInterruptException:
        pass