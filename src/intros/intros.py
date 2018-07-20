#!/usr/bin/python3
import rospy
import boardDevices
import time

def start():

	ms = boardDevices.magneticScaner()

    rospy.init_node('intros')

    cm_teak = "/teak;".encode()
    time_start = time.time()

    ms.initialize()
    ms.startScan()

    rospy.loginfo("Scan started")

    rate = rospy.Rate(4)
    while not rospy.is_shutdown():
    	rate.sleep()
    
	rospy.rosinfo("Terminating...")
    ms.stopScan()
    rospy.rosinfo("Ok")
