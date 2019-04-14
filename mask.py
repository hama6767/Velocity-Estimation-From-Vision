#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import csv
import matplotlib
import numpy as np
import matplotlib.pyplot as plt

distance_of_array = 200 # mm
sensor_size = 19.2 # y方向




def mask():
  # LEDの写真からLEDのxy相対座標を出す
  f = open('output.csv', 'w')
  writer = csv.writer(f, lineterminator='\n')
  csvlist = []
  before_x = 0
  before_y = 0
  error_count = 1
  mcn = -1
  pixel_height = 720
  pixel_width = 1280
  angle_of_view = 62.2 * 3.1415 / 180
  sensor_size = 37.7 * 3.1415 / 180
  #カメラ補正
  camera_matrix = np.array([[1.06105005e+003, 0.000000, 6.48830933e+002], [0.000000, 1.06279529e+003, 3.83095215e+002], [0., 0., 1.]])
  distortion_params = np.array([-2.45146647e-001, 1.50185645e-001, 2.24484666e-003, -2.79057189e-003, 0.000000])

  newcameramatrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_params, (1280, 720), 0.0)
  map_func = cv2.initUndistortRectifyMap(camera_matrix, distortion_params, np.eye(3), newcameramatrix, (1280, 720), cv2.CV_32FC1)

  # img = cv2.imread('c:\Users\makilab\Desktop\Docking-master\Docking-master\sample.jpg') # Errorのため絶対パスで指定　要改善


  for i in range(55):
    img = cv2.imread('c:\Users\makilab\Desktop\Docking-master\Docking-master\images\img{0:05d}.jpg'.format(i),)
    
    mcn = mcn + 1
    img_undistort = cv2.remap(img, map_func[0], map_func[1], cv2.INTER_AREA)


    # マスク処理パラメータ
    led_min = np.array([0, 150, 0], np.uint8)
    led_max = np.array([255, 255, 255], np.uint8)

    #　読み込み　
    
    img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    mask_led = cv2.inRange(img_undistort, led_min, led_max)

    #plt.imshow(cv2.cvtColor(mask_led, cv2.COLOR_GRAY2RGB))
    #plt.show()

    # LEDを検出
    # led_contours, _ = cv2.findContours(mask_led, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    led_image, led_contours, led_hierarchy = cv2.findContours(mask_led, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # make copy of image
    led_and_contours = np.copy(img)
  
    min_led_area = 5 # LEDエリアとして認識する最低領域面積　要パラメータ調整
    large_contours = [cnt for cnt in led_contours if cv2.contourArea(cnt) > min_led_area]

  
    # draw contours
    cv2.drawContours(led_and_contours, large_contours, -1, (255,0,0))
  
    # print number of contours
    print('number of led: %d' % len(large_contours))
    if not len(large_contours) == 4:
      error_count = error_count + 1
      continue    

    # 重心計算
    cx = []
    cy = []
    area = []
    #Tw = 0.0036736 / 640
    #Th = 0.0027384 / 360
    Tw = ( 2 * np.tan(angle_of_view/2)) / pixel_width * (-1)
    Th = ( 2 * np.tan(sensor_size/2)) / pixel_height
    for i in range(len(large_contours)):
      cnt = large_contours[i]
      M = cv2.moments(cnt)
      # print M['m00']
      cx.append((int(M['m10']/M['m00']) - pixel_width/2) * Tw )
      cy.append((int(M['m01']/M['m00']) - pixel_height/2) * Th)
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
    sin_alpha_1 = ((2*r / (h * np.tan(theta2))) - np.sqrt((2*r / (h * np.tan(theta2)))**2 - 4 * (1 + 1/((np.tan(theta2))**2)) * (r*r/h/h - 1)))/ (2 *  (1 + 1/((np.tan(theta2))**2)))
    sin_alpha_2 = ((2*r / (h * np.tan(theta2))) + np.sqrt((2*r / (h * np.tan(theta2)))**2 - 4 * (1 + 1/((np.tan(theta2))**2)) * (r*r/h/h - 1)))/ (2 *  (1 + 1/((np.tan(theta2))**2)))
    D = (2*r / (h * np.tan(theta2)))**2 - 4 * (1 + 1/((np.tan(theta2))**2)) * (r*r/h/h - 1)

    alpha2 = np.arcsin(sin_alpha_1)
    alpha = theta + 3.14 - alpha2
    print ox,r


   

    # 向き計算
    
    

    relative_angle = ( (cx[largest_area_number] * angle_of_view / pixel_width) - angle_of_view / 2 ) * 3.1415 / 180 # 相対角を計算

    #print relative_angle

    # 座標を出力



    Hz = 1.25 * error_count # logより逆算　ROSでは実際の稼働Hzを入れる

    error_count = 1
    

  # 出力
  writer.writerow(csvlist)

  # ファイルクローズ
  f.close()

mask() 
