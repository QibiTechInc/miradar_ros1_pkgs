#!/usr/bin/env python3

import rospy
from miradar_node.msg import PPI, PPIData
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import dynamic_reconfigure.client

class PPIVisualizer:
    def __init__(self):
        self.pub = rospy.Publisher("/miradar/markers", MarkerArray, queue_size=20)
        self.sub = rospy.Subscriber("/miradar/ppidata", PPIData, self.visualizePPI)
        
    def visualizePPI(self, data):

        markerArraydel = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "miradar"
        marker.action = marker.DELETEALL
        markerArraydel.markers.append(marker)
        self.pub.publish(markerArraydel)

        #---------------- QibiTech san original  color by dB
        # cli = dynamic_reconfigure.client.Client("miradar_node")
        # dynparam = cli.get_configuration()
        markerArray = MarkerArray()
        # 
        # mindb = dynparam["min_dB"]
        # maxdb = dynparam["max_dB"]
        #-----------------------------------------------------
        
        for i in range(len(data.data)):
            marker = Marker()
            marker.header.frame_id = "miradar"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0

            #-------------------- ST 2022_0704  color by speed
            if 1 < data.data[i].speed:
                marker.color.r = 0.0
                marker.color.b = 0.0
                marker.color.g = 1.0
            elif data.data[i].speed <-1:
                marker.color.r = 1.0
                marker.color.b = 0.0
                marker.color.g = 0.0
            else:
                marker.color.r = 0.0
                marker.color.b = 1.0
                marker.color.g = 0.0
            #------------------------------------------------

            #---------------- QibiTech san original  color by dB
            # a = 1.0/(float(maxdb) - float(mindb))
            # b = - (float(mindb)/(float(maxdb) - float(mindb)))
            # print("a : {0}, b : {1}".format(a, b))
            # marker.color.r = data.data[i].db * a + b
            # marker.color.b = 1.0 - marker.color.r
            # marker.color.g = 0.0
            #-----------------------------------------------------

            marker.pose.orientation.w = 1.0
            marker.pose.position = data.data[i].position
            marker.id = i
            markerArray.markers.append(marker)

        self.pub.publish(markerArray)
            
if __name__ == "__main__":
    rospy.init_node("ppi_visualizer")
    ppiVisualizer = PPIVisualizer()
    rospy.spin()
