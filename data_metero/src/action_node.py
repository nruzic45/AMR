#!/usr/bin/env python3

import rospy

from std_msgs.msg import Bool


class actionNODE():

    def __init__(self):

        rospy.init_node("action_node")

        self.sub_trigger = rospy.Subscriber("triggers/save_C", Bool, self.get_flag)
        self.UIrate = 10
        self.action = False
        self.counter = 0


    def get_flag(self,data):
        self.action = data.data
        if self.action:
            self.counter += 1
            print(self.counter)

    def run(self):
        rospy.loginfo("Starting actionNODE")

        self.main()

    def main(self):
        r = rospy.Rate(self.UIrate)
        while not rospy.is_shutdown():
            r.sleep

if __name__ == '__main__':
    actN = actionNODE()

    try:
        actN.run()
    except ROSInterruptException:
        pass