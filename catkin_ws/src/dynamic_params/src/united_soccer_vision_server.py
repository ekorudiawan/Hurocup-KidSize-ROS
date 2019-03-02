#!/usr/bin/env python
import rospy
import os
from dynamic_reconfigure.server import Server
import dynamic_reconfigure
from dynamic_params.cfg import UnitedSoccerVisionReconfConfig

def callback(config, level):
    return config
    

if __name__ == "__main__":
    rospy.init_node("united_soccer_vision_params")
    srv = Server(UnitedSoccerVisionReconfConfig, callback)
    rospy.spin()
