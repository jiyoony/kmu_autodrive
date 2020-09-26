import cv2
import numpy as np


class Warper:
	def __init__(self):
		h = 480
		w = 640
		# distort scr to dst
		src = np.float32([
		    [0, 320],
		    [0, 410],
		    [w, 320],
		    [w, 410],
		])
		dst = np.float32([
		    [-10,0],
		    [150, h], #200
		    [w, 0],
		    [w-150, h], #-200
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

