#!/usr/bin/env python

"""
Test objects that mimic what the vision system returns by 
randomly moving 3 objects around.
"""

import rospy
import copy
import random 

from std_msgs.msg import Header
from interface.msg import Object, ObjectArray



class VisionInterface:
    def __init__(self):
        rospy.init_node('vision_interface')

    
        self.object_pub = rospy.Publisher("/scene/vision/objects", Object, queue_size=10)

        rospy.Timer(rospy.Duration(3), self.object_monitor)


        rospy.spin()


    def object_monitor(self, event):
        objects = ['apple', 'bread', 'peanut_butter']
        dim = 0.05

        for label in objects:
            msg = Object()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.id = label
            msg.description = label
            msg.x = random.uniform(0.1, 0.3) 
            msg.y = random.uniform(-0.3, 0.3) 
            msg.z = 0.025  
            msg.width = dim
            msg.length = dim
            msg.height = dim


            self.object_pub.publish(msg)

            


if __name__=="__main__":
    scene_handler = VisionInterface()
