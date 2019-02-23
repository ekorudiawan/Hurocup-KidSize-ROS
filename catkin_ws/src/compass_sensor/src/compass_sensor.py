#! /usr/bin/env python
import rospy
import serial
from std_msgs.msg import Int32

def kill_node():
	rospy.signal_shutdown("shutdown time.") 

def main():
    ser = serial.Serial('/dev/arduino', 9600)

    rospy.loginfo("Read Compass - Running")
    rospy.init_node("compass_sensor")
    compass_pub = rospy.Publisher("/compass/value", Int32, queue_size=1)
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        serial_line = ser.readline().rstrip()
        # print(serial_line)
        try:	
            magneto = int(serial_line)
            # print(magneto)
            compass_pub.publish(magneto)	
        except:
            pass
        rate.sleep()
    rospy.on_shutdown(kill_node)
    ser.close()

if __name__ == "__main__":
	main()