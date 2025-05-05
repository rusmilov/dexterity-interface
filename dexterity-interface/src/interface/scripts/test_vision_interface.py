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

    
        self.object_pub = rospy.Publisher("/scene/vision/objects", ObjectArray, queue_size=10)

        rospy.Timer(rospy.Duration(10), self.object_monitor)


        rospy.spin()


    def object_monitor(self, event):
        objects = ['apple', 'bread', 'peanut_butter']
        dim = 0.05

        msg = ObjectArray()
        for label in objects:
            obj = Object()
            obj.header = Header()
            obj.header.stamp = rospy.Time.now()
            obj.id = label
            obj.description = label
            obj.x = random.uniform(0.3, 0.5) 
            obj.y = random.uniform(-0.2, 0.2) 
            obj.z = 0.025  
            obj.width = dim
            obj.length = dim
            obj.height = dim

            msg.objects.append(obj)

        self.object_pub.publish(msg)

            


if __name__=="__main__":
    scene_handler = VisionInterface()
