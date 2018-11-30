import cv2, matplotlib
import numpy as np
import matplotlib.pyplot as plt

distance_of_array = 200 # mm
sensor_size = 17.3 # y方向

def mask():

  # マスク処理
  led_min = np.array([0, 100, 0], np.uint8)
  led_max = np.array([255, 255, 255], np.uint8)

  img = cv2.imread('c:\Users\Roomba\Desktop\dock\sample.jpg') # Errorのため絶対パスで指定　要改善
  img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

  mask_led = cv2.inRange(img, led_min, led_max)
  plt.imshow(cv2.cvtColor(mask_led, cv2.COLOR_GRAY2RGB))

  plt.show()

  # LEDを検出
  # led_contours, _ = cv2.findContours(mask_led, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  led_image, led_contours, led_hierarchy = cv2.findContours(mask_led, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  # make copy of image
  led_and_contours = np.copy(img)
 
  min_led_area = 5 # 領域面積　要差パラメータ調整
  large_contours = [cnt for cnt in led_contours if cv2.contourArea(cnt) > min_led_area]
 
  # draw contours
  cv2.drawContours(led_and_contours, large_contours, -1, (255,0,0))
 
  # print number of contours
  print('number of led: %d' % len(large_contours))
  

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
  
  L1 = 0.02688 # ヨコ
  L2 = 0.02217 # タテ
  a = 0.015
  W = 1280
  W2 = 720
  h1 = cx[right_area_number] - cx [left_area_number] # 画像上の左右長さ px
  h2 = cy[largest_area_number] - cy[top_area_number]

  D1 = L1 * (W / float(h1))
  D2 = L2 * (W2 / float(h2))
  print D1,D2


mask() 
