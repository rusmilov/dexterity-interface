#!/usr/bin/env python

"""
Vision system that looks for objects
"""

import rospy
import copy
from assistive_robotics_thesis.src.florence_2_L.main import warmup, command

from std_msgs.msg import Header
from interface.msg import Object, ObjectArray



class VisionInterface:
    def __init__(self):
        rospy.init_node('vision_interface')
    
        self.object_pub = rospy.Publisher("/vision/objects", ObjectArray, queue_size=10)

        rospy.Timer(rospy.Duration(0.1), self.object_monitor_pub)


        rospy.spin()



    def object_monitor_pub(self, msg):
        # TODO Connect to VLM


        self.object_pub.publish(msg)

        


if __name__=="__main__":
    scene_handler = VisionInterface()
