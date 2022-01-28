#!/usr/bin/env python
import rospy
import rosunit
import roslaunch
import unittest
import dynamic_reconfigure.client
import sys
import time
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from miradar_node.msg import PPIData

PKG = "miradar_node"
NODE = "miradar_node"
LAUNCH = ""
TESTCASE = ""

class DynConfPubSubTestCase(unittest.TestCase):
    def __init__(self):
        self.ppisub = rospy.Subscriber("/miradar/ppidata", PPIData)
        self.scansub = rospy.Subscriber("/miradar/scan", LaserScan, self.parseScan)
        self.pcsub = rospy.Subscriber("/miradar/points", PointCloud2)

        self.scannum = 0
        self.scanmindist = 0
        self.scanmaxdist = 0

        self.isPPIPublished = False
        self.isScanPublished = False
        self.isPCPublished = False


    def parsePPI(self, data):
        self.isPPIPublished = True

    def parseScan(self, data):
        self.isScanPublished = True
        self.scannum = len(data.ranges)
        self.scanmindist = min(data.ranges)
        self.scanmaxdist = max(data.ranges)

    def parsePC(self, data):
        self.isPCPublished = True

    def test_sensormode_switch(self):
        cli = dynamic_reconfigure.client.Client("miradar_node")
        testcase = [0, 1, 2]
        cli.update_configuration({"sensor_mode" : testcase[0]})
            

    def test_hpf(self):
        cli = dynamic_reconfigure.client.Client("miradar_node")
        testcase = [-1, 0, 1, 2, 3]
        test_result = [False if (i < 0 or i > 2) else True for i in testcase]
        for i in range(len(testcase)):
            cli.update_configuration({"hpf_gain" : testcase[i]})
            time.sleep(0.1)
            dynparam = cli.get_configuration()
            self.assertEquals(dynparam["hpf_gain"] == testcase[i], test_result[i])

    def test_pga(self):
        cli = dynamic_reconfigure.client.Client("miradar_node")
        testcase = [-1, 0, 1, 2, 3, 4]
        test_result = [False if (i < 0 or i > 3) else True for i in testcase]
        for i in range(len(testcase)):
            cli.update_configuration({"pga_gain" : testcase[i]})
            time.sleep(0.1)
            dynparam = cli.get_configuration()
            self.assertEquals(dynparam["pga_gain"] == testcase[i], test_result[i], msg="case {0}, realdata : {1}, expected : {2}, actual {3}".format(i, dynparam["pga_gain"], test_result[i], dynparam["pga_gain"] == testcase[i]))

    def test_tx(self):
        cli = dynamic_reconfigure.client.Client("miradar_node")
        testcase = [-11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1]
        test_result = [False if i < -10 or i > 0 else True for i in testcase]
        for i in range(len(testcase)):
            cli.update_configuration({"tx_power" : testcase[i]})
            time.sleep(0.1)
            dynparam = cli.get_configuration()
            self.assertEquals(dynparam["tx_power"] == testcase[i], test_result[i])



def main():
    rospy.init_node("test_dynconf_pubsub")
    #print(PKG, NODE)
    #node = roslaunch.core.Node(PKG, NODE, NODE)

    #launch = roslaunch.scriptapi.ROSLaunch()
    #launch.start()
    #process = launch.launch(node)

    rosunit.unitrun(PKG, NODE, DynConfPubSubTestCase) 
    rospy.spin()

main()





if __name__ == "__main__":
    main()
    #rospy.init_node("test_dynconf")
    #print(PKG, NODE)
    #node = roslaunch.core.Node(PKG, NODE, NODE)

    #launch = roslaunch.scriptapi.ROSLaunch()
    #launch.start()
    #process = launch.launch(node)

    #rosunit.unitrun(PKG, NODE, DynConfTestCase) 
    #rostest.rosrun(PKG, NODE, DynConfTestCase)
    #process.stop()