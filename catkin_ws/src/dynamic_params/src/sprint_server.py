#!/usr/bin/env python
import rospy
import os
from dynamic_reconfigure.server import Server
import dynamic_reconfigure
from dynamic_params.cfg import SprintReconfConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {Pan_KP}, {Pan_KI},
          {Pan_KD}, {Pan_Step}, {Tilt_KP}, {Tilt_KI},
          {Tilt_KD}, {Tilt_Step}, {Scan_Rate}, {Body_Forward_KP}, {Body_Backward_KP}""".format(**config))
    return config

if __name__ == "__main__":
#    rospy.init_node("dynamic_params")
    rospy.init_node("sprint_params")
    srv = Server(SprintReconfConfig, callback)
    # os.system("rosrun dynamic_reconfigure dynparam load /dynamic_params ~/catkin_ws/dataset/sprint_param.yaml")
    rospy.spin()
