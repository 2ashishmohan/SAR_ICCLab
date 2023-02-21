#!/usr/bin/env python
from ultralytics import YOLO
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import numpy as np


global model1
global bridge
model1 = YOLO('yolov8n.pt')
bridge = CvBridge()
'''
results = model('image2.jpg', show = True)
#cv2.imshow("Image", cv_image)
for r in results:
    for c in r.boxes.cls:
        print(model.names[int(c)])

cv2.waitKey(1)

''' 
def image_callback(image):

    #print(image)
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

    results = model1(cv_image, show = True)

    #cv2.imshow("Image", cv_image)
    for r in results:
        for c in r.boxes.cls:
            print(model1.names[int(c)])

    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('object_detector')
    image_sub = rospy.Subscriber('/summit_xl/front_rgbd_camera/rgb/image_raw', Image, image_callback)
    rospy.spin()

