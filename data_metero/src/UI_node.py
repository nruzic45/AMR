#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float32

class uiNODE():

    def __init__(self):

        rospy.init_node("ui_node")

        self.sub_avg_temp = rospy.Subscriber("UI/graph_data",Float32, self.get_avgtemp)
        self.UIrate = 10


    def get_avgtemp(self,data):
        print(data.data)

    def run(self):
        rospy.loginfo("Starting uiNODE")

        self.main()

    def main(self):
        r = rospy.Rate(self.UIrate)
        while not rospy.is_shutdown():
            r.sleep

if __name__ == '__main__':
    uiN = uiNODE()

    try:
        uiN.run()
    except ROSInterruptException:
        pass