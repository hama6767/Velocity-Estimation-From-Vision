#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Vision-based position estimater
4 LEDs
@author: Yuya Hamamatsu
2017/7/16
'''
import math
import cv2
import rospy
from cv_bridge import CvBridge
from auv.msg import AUVStatus
from mavros_msgs.msg import OverrideRCIn
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt


class VisionEstimation(object):
    def __init__(self):
        rospy.init_node("vision_estimation")
        self._hz = rospy.get_param("~hz", 10.)
        self._rate = rospy.Rate(self._hz)
        self._img = Image()
        self._buff_img = Image()
        self._mask_img = Image()
        self._buff_mask_img = Image()

        self._relative_position = Vector3()
        self._pub_relative_position = rospy.Publisher(
            "relative_position", Vector3, queue_size=1)

        self._pub_mask_img = rospy.Publisher(
            "mask_image", Image, queue_size=10)

        rospy.Subscriber("usb_cam/image_raw", Image, self._set_buff_img)

        self._bridge = CvBridge()

    def _set_buff_img(self, msg):
        self._buff_img = msg

    def _update_subscribers(self):
        self._img = self._buff_img

    def _mask(self)
        if self._img.height == 0: 
            return 

        camera_matrix = np.array([[269.350336, 0.000000, 308.643701], [0.000000, 270.577786, 253.152731], [0., 0., 1.]]) 
        distortion_params = np.array([-0.220488, 0.036101, -0.002395, 0.005621, 0.000000]) 
        img_raw = self._bridge.imgmsg_to_cv2(self._img, "bgr8")

        newcameramatrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_params, (w, h), 1.0)
        map_func = cv2.initUndistortRectifyMap(camera_matrix, distortion_params, np.eye(3), newcameramatrix, (w, h), cv2.CV_32FC1) 
        img_undistort = cv2.remap(img_raw, map_func[0], map_func[1], cv2.INTER_AREA)

         # マスク処理パラメータ
        led_min = np.array([0, 150, 0], np.uint8)
        led_max = np.array([255, 255, 255], np.uint8)

        #　読み込み　
        
        img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        mask_led = cv2.inRange(img_undistort, led_min, led_max)

        # LEDを検出
        # led_contours, _ = cv2.findContours(mask_led, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        led_image, led_contours, led_hierarchy = cv2.findContours(mask_led, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # make copy of image
        led_and_contours = np.copy(img)
    
        min_led_area = 5 # LEDエリアとして認識する最低領域面積　要パラメータ調整
        large_contours = [cnt for cnt in led_contours if cv2.contourArea(cnt) > min_led_area]

    
        # draw contours
        cv2.drawContours(led_and_contours, large_contours, -1, (255,0,0))
    
        # LEDの数を判定　4以外なら無視
        print('number of led: %d' % len(large_contours))
        if not len(large_contours) == 4:
        error_count = error_count + 1
        continue    

        # 重心計算
        cx = []
        cy = []
        area = []
        for i in range(len(large_contours)):
        cnt = large_contours[i]
        M = cv2.moments(cnt)
        # print M['m00']
        cx.append(int(M['m10']/M['m00']))
        cy.append(int(M['m01']/M['m00']))
        area.append(int(M['m00']))
        # print(cx[i], cy[i])

        # 最大面積のLEDのインデックスを取得
        largest_area_number = np.argmax(area)
        top_area_number = np.argmin(cy)
        right_area_number = np.argmax(cx)
        left_area_number = np.argmin(cx)

        L1 =  3.762 # 0.26978 # ヨコ
        L2 = 2.975 # 0.22189 # タテ
        h = 0.075
        W = 43.00 #58
        W2 = 30.00 # 720.000

        oy2 = cx[top_area_number] 
        oz2 = cy[left_area_number] + (cy[right_area_number] - cy[left_area_number]) * (oy2 - cx[left_area_number]) / (cx[right_area_number] - cx[left_area_number])
        ox =  h / (oz2 - cy[top_area_number]) 
        theta = np.arctan(oy2)
        r = ox * np.sqrt(1 + oy2 * oy2)
        oz = ox * oz2
        oy = np.sin(theta) * r
        theta2 = np.arctan(cx[largest_area_number]) - theta

        # 例外処理
        if theta2 == 0.0:
        error_count = error_count + 1
        print mcn #"Zero division error!"
        continue    

        if ox < 1.0:
        #print ox,oy
        error_count = error_count + 1
        print mcn #"Calculate error!"
        continue   

        sin_alpha_1 = ((2*r / (h * np.tan(theta2))) - np.sqrt((2*r / (h * np.tan(theta2)))**2 - 4 * (1 + 1/((np.tan(theta2))**2)) * (r*r/h/h - 1)))/ (2 *  (1 + 1/((np.tan(theta2))**2)))
        sin_alpha_2 = ((2*r / (h * np.tan(theta2))) + np.sqrt((2*r / (h * np.tan(theta2)))**2 - 4 * (1 + 1/((np.tan(theta2))**2)) * (r*r/h/h - 1)))/ (2 *  (1 + 1/((np.tan(theta2))**2)))


        alpha2 = np.arcsin(sin_alpha_1)
        alpha = theta + 3.14 - alpha2

        self._relative_position.x = ox
        self._relative_position.y = oy
        self._relative_position.z = oz

        # publish
        self._relative_position.publish(self._relative_position)
        self._mask_image.publish(mask_led)

    def run(self):
        while not rospy.is_shutdown():
            self._update_subscribers()
            self._mask()
            self._rate.sleep()


if __name__ == "__main__":
    controller = VisionEstimation()
    controller.run()

        
