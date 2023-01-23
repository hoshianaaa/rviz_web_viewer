#!/usr/bin/env python
import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import pyfakewebcam

FCAM_W = 640
FCAM_H = 480

class cvBridgeDemo:
    def __init__(self):

        self.camera = pyfakewebcam.FakeWebcam("/dev/video10", FCAM_W, FCAM_H)

        self.node_name = "cv_bridge_demo_compressed"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/rviz/image/compressed", CompressedImage, self.image_callback, queue_size=1)

    def image_callback(self, ros_image_compressed):
        global FCAM_W, FCAM_H
        print("image callback")
        try:
            np_arr = np.fromstring(ros_image_compressed.data, np.uint8)
            input_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        except(CvBridgeError, e):
            print(e)


        out_img = cv2.resize(input_image, (FCAM_W, FCAM_H))

        plt = cv2.cvtColor(out_img, cv2.COLOR_BGR2RGB)

        self.camera.schedule_frame(plt)

    #    cv2.imshow(self.node_name, input_image)   
    #    cv2.waitKey(1)

    def cleanup(self):
        cv2.destroyAllWindows()   

if __name__ == '__main__':
    cvBridgeDemo()
    rospy.spin()
