import cv2
import numpy as np


class Warper:
	def __init__(self):
		h = 480
		w = 640
		# distort scr to dst
		src = np.float32([
		    [100,320],
		    [-80,400],
		    [715,400],
		    [544,320],
		])
		dst = np.float32([
		    [0,0],
		    [50,h - 20],
		    [w-70 , h - 20],
		    [w , 0],
		])
	    

		self.M = cv2.getPerspectiveTransform(src, dst)
		self.Minv = cv2.getPerspectiveTransform(dst, src)

	def warp(self, img):
		h = img.shape[0]
		w = img.shape[1]
		#z = img.shape[2]
		return cv2.warpPerspective(
		    img,
		    self.M,
		    (w, h),
		    flags=cv2.INTER_LINEAR
		)

