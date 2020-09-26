import numpy as np
import cv2
import math
class SlideWindow:

	def __init__(self):
		self.left_fit = None
		self.right_fit = None
		self.leftx = None
		self.rightx = None
		self.center_old = 320
		self.left_old = 180
		self.right_old = 500
		self.flag = None
		
	def slidewindow(self, img):
	
		x_location = 320
		# init out_img, height, width        
		out_img = np.dstack((img, img, img))
		height = img.shape[0]
		width = img.shape[1]

		# num of windows and init the height
		window_height = 10
		nwindows = 10
		center_x = np.zeros((nwindows),dtype = 'f')
		center_y = np.zeros((nwindows),dtype = 'f')
		# find nonzero location in img, nonzerox, nonzeroy is the array flatted one dimension by x,y 
		nonzero = img.nonzero()
		#print nonzero 
		nonzeroy = np.array(nonzero[0])
		nonzerox = np.array(nonzero[1])
		#print nonzerox
		# init data need to sliding windows
		margin = 20
		minpix = 10
		left_lane_inds = []
		right_lane_inds = []

		# first location and segmenation location finder
		# draw line
		# 130 -> 150 -> 180
		pts_left = np.array([[width/2 - 90, height], [width/2 - 90, height - 60], [width/2 - 200, height - 80], [width/2 - 200, height]], np.int32)
		cv2.polylines(out_img, [pts_left], False, (0,255,0), 1)
		pts_right = np.array([[width/2 + 60, height], [width/2 + 60, height - 80], [width/2 + 190, height - 110], [width/2 + 190, height]], np.int32)
		cv2.polylines(out_img, [pts_right], False, (255,0,0), 1)
		#pts_center = np.array([[width/2 + 90, height], [width/2 + 90, height - 150], [width/2 - 60, height - 231], [width/2 - 60, height]], np.int32)
		#cv2.polylines(out_img, [pts_center], False, (0,0,255), 1)
		#pts_catch = np.array([[0, 340], [width, 340]], np.int32)
		#cv2.polylines(out_img, [pts_catch], False, (0,120,120), 1)

		# indicies before start line(the region of pts_left)
		# 337 -> 310
		good_left_inds = ((nonzerox >= width/2 - 130) & (nonzeroy >= nonzerox * 0.33 + 260) & (nonzerox <= width/2 - 95)).nonzero()[0]
		good_right_inds = ((nonzerox >= width/2 + 20) & (nonzeroy >= nonzerox * (-0.48) + 460) & (nonzerox <= width/2 + 120)).nonzero()[0]
		#################################################################################
		# left line exist, lefty current init
		line_exist_flag = None 
		y_current = None
		x_current = None
		good_center_inds = None
		p_cut = None

		# check the minpix before left start line
		# if minpix is enough on left, draw left, then draw right depends on left
		# else draw right, then draw left depends on right
		if len(good_left_inds) > minpix:
			line_flag = 1
			x_current = np.int(np.mean(nonzerox[good_left_inds]))
			y_current = np.int(np.mean(nonzeroy[good_left_inds]))
			max_y = y_current
		elif len(good_right_inds) > minpix:
			line_flag = 2
			x_current = nonzerox[good_right_inds[np.argmax(nonzeroy[good_right_inds])]]
			y_current = np.int(np.max(nonzeroy[good_right_inds]))
		else:
			line_flag = 3
			# indicies before start line(the region of pts_center)
			# good_center_inds = ((nonzeroy >= nonzerox * 0.45 + 132) & (nonzerox >= width/2 - 60) & (nonzerox <= width/2 + 90)).nonzero()[0] 
			# p_cut is for the multi-demensional function
			# but curve is mostly always quadratic function so i used polyfit( , ,2)
		#    if nonzeroy[good_center_inds] != [] and nonzerox[good_center_inds] != []:
		#        p_cut = np.polyfit(nonzeroy[good_center_inds], nonzerox[good_center_inds], 2)

		if line_flag != 3:
			# it's just for visualization of the valid inds in the region
			for i in range(len(good_left_inds)):
				img = cv2.circle(out_img, (nonzerox[good_left_inds[i]], nonzeroy[good_left_inds[i]]), 1, (0,255,0), -1)
			# window sliding and draw
			for window in range(0, nwindows):
				if line_flag == 1: 
					# rectangle x,y range init
					win_y_low = y_current - (window + 1) * window_height
					win_y_high = y_current - (window) * window_height
					win_x_low = x_current - margin
					win_x_high = x_current + margin
					# draw rectangle
					# 0.33 is for width of the road
					cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 1)
					cv2.rectangle(out_img, (win_x_low + int(width * 0.33), win_y_low), (win_x_high + int(width * 0.33), win_y_high), (255, 0, 0), 1)
					# indicies of dots in nonzerox in one square
					good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
					# check num of indicies in square and put next location to current 
					if len(good_left_inds) > minpix:
						x_current = np.int(np.mean(nonzerox[good_left_inds]))
						print('minpix mode')
					elif nonzeroy[left_lane_inds] != [] and nonzerox[left_lane_inds] != []:
						print('not minpix mode')
						p_left = np.polyfit(nonzeroy[left_lane_inds], nonzerox[left_lane_inds], 1) 
						x_current = np.int(np.polyval(p_left, win_y_high))
					# 338~344 is for recognize line which is yellow line in processed image(you can check in imshow)
					#if win_y_low >= 338 and win_y_low < 344:
						# 0.165 is the half of the road(0.33)
					x_location = x_current + int(width * 0.15) 
					center_x[window] = x_location
					center_y[window] =int((win_y_low + win_y_high)/2)
				elif line_flag ==2: # change line from left to right above(if)
					win_y_low = y_current - (window + 1) * window_height
					win_y_high = y_current - (window) * window_height
					win_x_low = x_current - margin
					win_x_high = x_current + margin
					cv2.rectangle(out_img, (win_x_low - int(width * 0.33), win_y_low), (win_x_high - int(width * 0.33), win_y_high), (0, 255, 0), 1)
					cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0, 0), 1)
					good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
					if len(good_right_inds) > minpix:
						x_current = np.int(np.mean(nonzerox[good_right_inds]))
					elif nonzeroy[right_lane_inds] != [] and nonzerox[right_lane_inds] != []:
						p_right = np.polyfit(nonzeroy[right_lane_inds], nonzerox[right_lane_inds], 2) 
						x_current = np.int(np.polyval(p_right, win_y_high))
					if win_y_low >= 338 and win_y_low < 344:
						# 0.165 is the half of the road(0.33)
						x_location = np.int(x_current - int(width * 0.15))
					center_x[window] = x_location
					center_y[window] = int((win_y_low + win_y_high)/2)
		
				left_lane_inds.extend(good_left_inds)
		else:
			print('aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')
			for window in range(nwindows):
				center_x[window] = np.int(320)
				center_y[window] = np.int(240)

	
				
		
		#print('center_x :',center_x)
		#print('center_y :',center_y)
		fp1 = np.polyfit(center_y, center_x,2)
		f1 = np.poly1d(fp1)
		returns_x = f1(center_y)
		#print("returns_x : ",returns_x)
		#print("center_y : ",center_y)
		cv2.line(out_img,(int(returns_x[0]),int(center_y[0])),(int(returns_x[nwindows-1]),int(center_y[nwindows-1])),(0,0,255))
		for i in range(nwindows):
			print(returns_x[i], center_y[i])
			cv2.circle(out_img,(int(returns_x[i]),int(center_y[i])),3,(0,0,255),-1)
		steer_theta=math.atan((returns_x[0] - returns_x[-1])/(center_y[0] - center_y[nwindows-1]))
		print("steer_theta:", steer_theta)
		#cv2.line(out_img,(320,460 ),(returns_x[nwindows-1],center_y[nwindows-1]),(0,255,255))
		#print(line_flag)
		
		#print(x_location)
		return out_img, x_location,line_flag
