#!/usr/bin/env python
import os
import rospy

class Command(object):
    def __init__(self,debug=False):
        os.system("sudo python /home/ubuntu/duckietown/catkin_ws/src/sensor/src/DHT11_node.py")
    
if __name__ == '__main__':
    rospy.init_node('command',anonymous=False)
    command = Command()
    rospy.spin()