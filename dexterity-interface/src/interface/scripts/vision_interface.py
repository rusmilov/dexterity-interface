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

        self.model, self.processor = warmup()
    
        self.object_pub = rospy.Publisher("/scene/vision/objects", ObjectArray, queue_size=10)

        rospy.Timer(rospy.Duration(0.1), self.object_monitor)


        rospy.spin()


    def mmToM(self, mm:float) -> float:
        """
        Convert millimeters to meters
        """
        M = float(mm) / 1000
        return M
    

    def object_monitor(self, event):
        print("MADE IT HERE!")

        objects = command(self.model, self.processor, "")
        print("OBJECTS", objects)

        msg = ObjectArray()
        for objDict in objects:
            
            # TODO: Make different width, len, height
            dim = self.mmToM(objDict['depth_max']) - self.mmToM(objDict['depth_min'])

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
            print("HERE", obj)
        
        # TODO convert to robot frame of reference

        

        self.object_pub.publish(msg)

        


if __name__=="__main__":
    scene_handler = VisionInterface()
