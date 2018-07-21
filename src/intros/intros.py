#!/usr/bin/python3
import rospy
import boardDevices
import time

def start():

    rospy.init_node('intros')

    boardDevices.intros.init()
    rospy.loginfo(boardDevices.intros.readData())
    boardDevices.intros.initialize()
    rospy.loginfo(boardDevices.intros.readData())
    if boardDevices.intros.startScan() == 1:
        rospy.loginfo("Scan started")
        rospy.loginfo(boardDevices.intros.readData())

        rate = rospy.Rate(4)
        while not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo("Terminating...")
        boardDevices.intros.stopScan()
        rospy.loginfo("Scan stopped")
        rospy.loginfo(boardDevices.intros.readData())
    else:
        rospy.loginfo("Unable to start scan")
