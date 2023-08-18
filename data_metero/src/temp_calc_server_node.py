#!/usr/bin/env python3

import rospy
import csv

from data_metero.srv import cel_to_far, cel_to_farResponse

#rosservice call /<ime_servisa> + 2 puta tab, da ti isformatira poruku
#rosservice call /cel_to_far + 2 puta tab

def celsius_to_fahrenheit(celsius):
    fahrenheit = (celsius * 1.8) + 32
    return fahrenheit


def callback(req):
    
    maxTempF = celsius_to_fahrenheit(req.a)
    minTempF = celsius_to_fahrenheit(req.b)
    avgTempF = celsius_to_fahrenheit(req.c) 

    # data_to_write  = "newdata,"+str(maxTempF)+","+str(minTempF)+","+str(avgTempF)+","+"0.0,"+"0.0,"+"0.0"+'\n'
    data_to_write = ["newdata", str(maxTempF),str(minTempF),str(avgTempF),'0.0','0.0','0.0']

    with open('src/data_metero/src/weather_data_nyc_centralpark_2016.csv', 'a+') as f:
        print(data_to_write)
        writer = csv.writer(f)
        writer.writerow(data_to_write)

    return cel_to_farResponse(True)

def calculation():
    rospy.init_node("temperature_calculation_service")
    service = rospy.Service("cel_to_far", cel_to_far, callback)
    rospy.loginfo("Service is ready!")
    rospy.spin()

if __name__ == '__main__':
    calculation()