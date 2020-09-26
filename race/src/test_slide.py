#!/usr/bin/env python

import cv2, time
import rospy, rospkg
from xycar_motor.msg import xycar_motor

import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

bridge = CvBridge()
pub = None
cv_image = np.empty(shape = [0])
gstart_time = None
g_stop = False

###
#out2 = cv2.VideoWriter('/home/nvidia/Desktop/oripy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (640,480)) # my_code
####

def img_callback(data):
    global cv_image
    try:
      tmp = bridge.imgmsg_to_cv2(data, "bgr8")
      cv_image = cv2.resize(tmp, (640,480))
    except CvBridgeError as e:
      rospy.loginfo(e)

def auto_drive(pid,speed):
	global pub
	#global car_run_speed

	msg = xycar_motor()
	msg.angle = pid
	msg.speed = speed
	pub.publish(msg)


def detect_line(edge_img, hsv_mask):
    global gstart_time
    global g_stop

    nonzero = 0
    col = 640

    rpos = 0
    lpos = 640

    thresh_pixel_cnt = 20 #30
    height_offset = 15
    r_width = 20
    r_height = 10

    i = height_offset

    for rcol in range(col/2 + 100, col):
        if edge_img[i, (rcol-1)] == 255:
            detect_area = hsv_mask[i - r_height : i, (rcol-1) : (rcol-1) + r_width]
            nonzero = cv2.countNonZero(detect_area)

        if nonzero > thresh_pixel_cnt:
            rpos = (rcol - 1)
            break

    for lcol in range(col/2 - 100, 0, -1):
        if edge_img[i, (lcol-1)] == 255:
            detect_area = hsv_mask[i - r_height : i, (lcol-1) : (lcol-1) + r_width]
            nonzero = cv2.countNonZero(detect_area)

            if nonzero > thresh_pixel_cnt:
                lpos = (lcol - 1)
                break

    '''
    cnt_block = 0
    len_contours = 0

    for ii in range(-120,120,10):
       detect_area = hsv_mask[2:15,320+ii:320+ii+10]
       cntzero = cv2.countNonZero(detect_area)
       if cntzero >= 100:
          cnt_block = cnt_block + 1
          continue
  
    if cnt_block > 5:
       _,contours,_ = cv2.findContours(hsv_mask[2:15,320-120-50:320+120+50], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
       len_contours = len(contours)

    chk_time = 0

    if cnt_block >= 6 and len_contours <= 3:   
       rospy.loginfo("STOP LINE!!!")
       gstart_time = time.time()

       g_stop = True

       rospy.sleep(0.3)
       auto_drive(0, 90)

    if gstart_time != None:
       chk_time = (time.time() - gstart_time) 
       if chk_time >= 3:
          g_stop = False
          gstart_time = None
    '''
    return lpos, rpos

def process_image(frame, brightness):
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    width = 640

    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

    low_threshold = 60
    high_threshold = 70
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

    offset_roi = 70 #120
    mask = frame[430-offset_roi:450-offset_roi, 0:width]

    hsv = cv2.cvtColor(mask, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0,0,brightness], dtype=np.uint8)
    upper_white = np.array([131,255,255], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_white, upper_white)

    edge2_img = edge_img[430-offset_roi:450-offset_roi, 0:width]
    hsv_mask = mask

    lpos, rpos = detect_line(edge2_img, hsv_mask)

    cv2.rectangle(frame, (0, 430 - offset_roi), (width, 450 - offset_roi), (0, 255, 0))
    cv2.imshow('origin', frame)
    cv2.imshow('HSV', hsv_mask)
    cv2.imshow('edge', edge2_img)

    return lpos, rpos

def start(brightness):
    global pub
    global g_stop

    ###    
    lpos_old = 136
    rpos_old = 549
    ####

    rospy.init_node('xycar_A2')
    pub = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)

    image_sub = rospy.Subscriber('/usb_cam/image_raw/', Image, img_callback)
    rospy.sleep(2)
    speed = 3
    print("fuck")
    print(cv_image.size)

    while cv_image.size == (640*480*3):
      lpos, rpos = process_image(cv_image, brightness)
      print("LPOS : {}  RPOS : {}".format(lpos, rpos))

      if np.abs(lpos_old - lpos) <  np.abs(rpos_old - rpos):
        lpos_old = lpos
        rpos_old = lpos + 413
        x_location = 136 - lpos
        x_location = (x_location / -100.0)
      else:
        rpos_old = rpos
        lpos_old = rpos - 413
        x_location = 549 - rpos # 168
        x_location = (x_location / -100.0)

      auto_drive(x_location, speed)

      if cv2.waitKey(1) & 255 == ord('q'):
        break
    rospy.spin()

def main():
    global Speed 
    global Angle

    time.sleep(3)

    Speed = 3
    Angle = 0

    start(50)

if __name__ == '__main__':
    main()

