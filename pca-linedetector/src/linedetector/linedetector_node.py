import glob
import os
import sys
import cv2
import numpy as np
import rospy
import cv_bridge
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image

def nothing(x):
    pass

def image_callback(data):
    frame = data

pub = rospy.Publisher('/line', Vector3, queue_size=100)
rospy.init_node('linedetector_node', anonymous=True)
rate = rospy.Rate(200)

cv2.namedWindow('Color Range')
cv2.createTrackbar('Blue Low',  'Color Range', 12, 255,  nothing)
cv2.createTrackbar('Green Low', 'Color Range', 45, 255,  nothing)
cv2.createTrackbar('Red Low',   'Color Range', 100, 255, nothing)
cv2.createTrackbar('Blue High', 'Color Range', 41, 255,  nothing)
cv2.createTrackbar('Green High','Color Range', 202, 255, nothing)
cv2.createTrackbar('Red High',  'Color Range', 255, 255, nothing)

