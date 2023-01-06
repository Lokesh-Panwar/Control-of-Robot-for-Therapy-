#!/usr/bin/env python3
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from scipy.integrate import quad
import rospy
import time
import numpy as np

class extract_points():
    def __init__(self):

        self.des_vol = input('What is the desired volume:    ')
        self.x = 0
        self.y = 0
        self.z = 0
        self.volume = 0
        self.xpoint = []
        self.ypoint = []
        self.zpoint = []
        self.cross_section = 0

    def callback_pointcloud(self,data):
            #global volume
            #global x
            #global y
            #global z
        assert isinstance(data, PointCloud2)
        gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True) 

        for p in gen:
            self.xpoint.append(p[0])
            self.ypoint.append(p[1])
            self.zpoint.append(p[2]) 
        self.x = max(self.xpoint) - min(self.xpoint)
        self.y = max(self.ypoint) - min(self.ypoint)
        self.cross_section = self.x*self.y

        def f(z):
            return self.cross_section

        i = quad(f,min(self.zpoint),max(self.zpoint))
        self.volume = i

    def main(self):
        rospy.init_node('pcl_listener', anonymous=True)
        rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.callback_pointcloud)
        rospy.sleep(10)
        #rospy.spin()
        print ("value of x : ", self.x)
        print ("value of y : ", self.y)
        #print ("value of z : ", z)
        #print ("Total initial volume is : ", self.volume)
        #remaining_volume = self.volume - float(self.des_vol)
        #quotient = remaining_volume/self.volume
        #percentage = quotient * 100
        #print("Remaining Volume needs to be completed : ",remaining_volume)
        #print("Remaining Percentage needs to be completed : ",percentage)

if __name__ == "__main__":
    p = extract_points()
    p.main()