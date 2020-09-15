import cv2
import numpy as np
import hj_linedetect

class Pix_Stop:
    def findStop(self, roi, img):
	stopFlag = 0
	nonzero = roi.nonzero()

	lower_yellow = np.array([-20, 80, 80])
	upper_yellow = np.array([50. 255, 255])
	
	img = img[240:, :]
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
	res = cv2.bitwise_and(img, img, mask = mask_yellow)

    	resnon = res.nonzero()
	resx = np.array(resnon[1])
	goodres = ((resx > 200) &(resx < 440)).nonzero()[0]

	if (len(nonzero[0] > 1100):
		stopFlag = 1
		if (len(goodres)):
			stopFlag = 2
	return stopFlag
