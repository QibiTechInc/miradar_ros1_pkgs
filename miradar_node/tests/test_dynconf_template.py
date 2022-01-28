#!/usr/bin/env python
import rospy
import rosunit
import roslaunch
import unittest
import dynamic_reconfigure.client
import sys
import time

PKG = "miradar_node"
NODE = "miradar_node"
LAUNCH = ""
TESTCASE = ""

class DynConfTestCase(unittest.TestCase):
    def test_sensormode(self):
        cli = dynamic_reconfigure.client.Client("miradar_node")
        testcase = [-1, 0, 1, 2, 3]
        test_result = [False, True, True, True, False]
        for i in range(len(testcase)):
            cli.update_configuration({"sensor_mode" : testcase[i]})
            time.sleep(0.1)
            dynparam = cli.get_configuration()
            self.assertEquals(dynparam["sensor_mode"] == testcase[i], test_result[i])

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
    rospy.init_node("test_dynconf")
    print(PKG, NODE)
    node = roslaunch.core.Node(PKG, NODE, NODE)

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)

    rosunit.unitrun(PKG, NODE, DynConfTestCase) 
    #rostest.rosrun(PKG, NODE, DynConfTestCase)
    process.stop()

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