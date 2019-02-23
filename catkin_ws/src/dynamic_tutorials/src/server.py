#!/usr/bin/env python

import rospy
# import yaml
import os

from dynamic_reconfigure.server import Server
from dynamic_reconfigure.load_file import load_file
from dynamic_tutorials.cfg import TutorialsConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {Pan_KP}, {Pan_KI},
          {Pan_KD}, {Pan_Step}, {Tilt_KP}, {Tilt_KI},
          {Tilt_KD}, {Tilt_Step}, {Scan_Rate}""".format(**config))

    # rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
    #       {str_param}, {bool_param}, {size}""".format(**config))
    
    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_tutorials")
    # rosrun dynamic_reconfigure dynparam load /dynamic_tutorials ~/catkin_ws/dataset/sprint_param.yaml
    # data = load_file("~/catkin_ws/dataset/sprint_param.yaml")
    srv = Server(TutorialsConfig, callback)
    ld = load_file(~/catkin_ws/dataset/sprint_param.yaml)
    os.system("rosrun dynamic_reconfigure dynparam load /dynamic_tutorials ~/catkin_ws/dataset/sprint_param.yaml")
    rospy.spin()