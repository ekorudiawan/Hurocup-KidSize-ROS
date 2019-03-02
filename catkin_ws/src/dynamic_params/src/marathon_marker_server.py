#!/usr/bin/env python
import rospy
import os
from dynamic_reconfigure.server import Server
import dynamic_reconfigure
from dynamic_params.cfg import MarathonMarkerReconfConfig

def callback(config, level):
    # rospy.loginfo("""Reconfigure Request: {H_Max}, {H_Min},
    #       {S_Max}, {S_Min}, {V_Max}, {V_Min}, {Min_Size}, {Pan_KP}, {Pan_KI},
    #       {Pan_KD}, {Pan_Step}, {Tilt_KP}, {Tilt_KI},
    #       {Tilt_KD}, {Tilt_Step}, {Tilt_Angle}, {Scan_Rate}, {Body_KP},""".format(**config))
    return config
    

if __name__ == "__main__":
    rospy.init_node("marathon_marker_params")
    srv = Server(MarathonMarkerReconfConfig, callback)
    # os.system("rosrun dynamic_reconfigure dynparam load /marathon_params ~/catkin_ws/dataset/marathon_param.yaml")
    rospy.spin()
