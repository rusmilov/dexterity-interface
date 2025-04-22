#!/usr/bin/env python

"""
Vision system that looks for objects
"""

import rospy
import copy
from assistive_robotics_thesis.src.florence_2_L.main import warmup, command

from std_msgs.msg import Header
from interface.msg import Object, ObjectArray

import random



class VisionInterface:
    def __init__(self):
        rospy.init_node('vision_interface')

        self.model, self.processor = warmup()
    
        self.object_pub = rospy.Publisher("/scene/vision/objects", ObjectArray, queue_size=10)

        rospy.Timer(rospy.Duration(1), self.object_monitor)
        print("STARTING DETECTION NOW.")

        rospy.spin()


    def mmToM(self, mm:float) -> float:
        """
        Convert millimeters to meters
        """
        M = float(mm) / 1000
        return M
    

    def object_monitor(self, event):

        objects = command(self.model, self.processor, "")

        msg = ObjectArray()
        for objDict in objects:
            
            # TODO: Make different width, len, height
            # dim = self.mmToM(objDict['depth_max']) - self.mmToM(objDict['depth_min'])
            dim = 0.05

            obj = Object()
            obj.header = Header()
            obj.header.stamp = rospy.Time.now()
            obj.id = objDict['label']
            obj.description = objDict['label']
            obj.x = self.mmToM(objDict['center_coord'][0])
            obj.y = self.mmToM(objDict['center_coord'][1])
            obj.z = self.mmToM(objDict['center_coord'][2])
            obj.width = dim
            obj.length = dim
            obj.height = dim

            msg.objects.append(obj)

        #     print(obj)

        # objects = ['apple', 'bread', 'peanut_butter']
        # dim = 0.05

        # msg = ObjectArray()
        # for label in objects:
        #     obj = Object()
        #     obj.header = Header()
        #     obj.header.stamp = rospy.Time.now()
        #     obj.id = label
        #     obj.description = label
        #     obj.x = random.uniform(0.1, 0.3) 
        #     obj.y = random.uniform(-0.3, 0.3) 
        #     obj.z = 0.025  
        #     obj.width = dim
        #     obj.length = dim
        #     obj.height = dim

        #     msg.objects.append(obj)
        
        # TODO convert to robot frame of reference

        

        self.object_pub.publish(msg)

        


if __name__=="__main__":
    scene_handler = VisionInterface()
