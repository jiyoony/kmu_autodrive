import numpy as np
import cv2
import math

#Now Using Line Detect File


class SlideWindow:
	def __init__(self):
		self.xL_current = 0
		self.xL_Old_current = 640
		self.xR_current = 0
		self.xR_Old_current = 0
		self.y_current = 0
		self.x_location = 0
		self.x_location_old = 320
		
	def slidewindow(self,img):
		line_Width = 160
		left_flag = False
		right_flag = False
		
		out_img = np.dstack((img,img,img))
		height = img.shape[0]
		width = img.shape[1]
		
		window_height = 5
		nwindows = 30 

		nonzero = img.nonzero()
		nonzeroy = np.array(nonzero[0])
		nonzerox = np.array(nonzero[1])
		margin = 20
		minpix = 10
		decide_x_location_idx = [338, 344]	

		left_lane_inds = []
		right_lane_inds = []

		# left visualize
		pts_left = np.array([[width/2 - 85, height], [width/2 -85,420], [width/2-210,420],[width/2-210,height]],np.int32)
		cv2.polylines(out_img,[pts_left],False, (0,255,0),1)
		
		# right visualize
		pts_right = np.array([[width/2 + 85, height], [width/2+85, 420],[width/2 + 210,420],[width/2+ 210, height]],np.int32)
		cv2.polylines(out_img,[pts_right],False,(255,0,0),1)

		# start inds
		good_left_inds = ((nonzerox >= width/2 - 210) & (nonzeroy >= 420) & (nonzerox <= width/2 - 85)).nonzero()[0]
		good_right_inds = ((nonzerox >= width/2 + 85) & (nonzeroy >= 420) & (nonzerox <= width/2 + 210)).nonzero()[0]

 
		# check start inds
		if len(good_left_inds) > minpix:
			self.xL_current = np.int(np.mean(nonzerox[good_left_inds]))
			self.y_current = np.int(np.mean(nonzeroy[good_left_inds]))
			left_flag = True
		if len(good_right_inds) > minpix:
			self.xR_current = np.int(np.mean(nonzerox[good_right_inds]))
			self.y_current = np.int(np.mean(nonzeroy[good_right_inds]))
			right_flag = True
			
		# when left, right start inds is good
		if left_flag == True and right_flag == True:			
			for window in range(nwindows):
				y_low = self.y_current - (window + 1) * window_height
				y_high = self.y_current - window*window_height
				
				xL_left = self.xL_current - margin
				xL_right = self.xL_current + margin
				
				xR_left = self.xR_current - margin
				xR_right = self.xR_current + margin
				
				cv2.rectangle(out_img, (xL_left,y_low),(xL_right,y_high),(0,255,0),1)
				cv2.rectangle(out_img, (xR_left,y_low),(xR_right,y_high),(0, 0, 255),1)
				
				good_left_inds = ((nonzeroy >= y_low) & (nonzeroy <y_high) & (nonzerox >=xL_left) & (nonzerox < xL_right)).nonzero()[0]
				good_right_inds = ((nonzeroy >= y_low) & (nonzeroy <y_high) & (nonzerox >=xR_left) & (nonzerox < xR_right)).nonzero()[0]

				if len(good_left_inds) > minpix:
					self.xL_current = np.int(np.mean(nonzerox[good_left_inds]))
				elif nonzeroy[left_lane_inds] != [] and nonzerox[left_lane_inds] != []:
					p_left = np.polyfit(nonzeroy[left_lane_inds], nonzerox[left_lane_inds], 2) 
					self.xL_current = np.int(np.polyval(p_left, y_high))

				if len(good_right_inds) > minpix:
					self.xR_current = np.int(np.mean(nonzerox[good_right_inds]))
				elif nonzeroy[right_lane_inds] != [] and nonzerox[right_lane_inds] != []:
					p_right = np.polyfit(nonzeroy[right_lane_inds], nonzerox[right_lane_inds], 2) 
					self.xR_current = np.int(np.polyval(p_right, y_high))

				left_lane_inds.extend(good_left_inds)
				right_lane_inds.extend(good_right_inds)

				# decide x_location
				if y_low >= decide_x_location_idx[0] and y_low < decide_x_location_idx[1]:
					if (abs(self.xL_Old_current - self.xL_current) < abs(self.xR_Old_current - self.xR_current)):
						self.xL_Old_current = self.xL_current
						self.xR_Old_current = self.xL_current + (2 * line_Width)
						self.x_location = self.xL_current + line_Width
						cv2.circle(out_img, (self.x_location,int(np.mean(decide_x_location_idx))),2,(0,255,0),2)
					else:
						self.xL_Old_current = self.xR_current - (2 * line_Width)
						self.xR_Old_current = self.xR_current
						self.x_location = self.xR_current - line_Width
						cv2.circle(out_img, (self.x_location,int(np.mean(decide_x_location_idx))),2,(0,0,255),2)
						
		
		# when only left start inds is good					
		elif left_flag == True:
			for window in range(nwindows):
				y_low = self.y_current - (window + 1) * window_height
				y_high = self.y_current - window*window_height
				
				xL_left = self.xL_current - margin
				xL_right = self.xL_current + margin
				
				cv2.rectangle(out_img, (xL_left,y_low),(xL_right,y_high),(0,255,0),1)
				
				good_left_inds = ((nonzeroy >= y_low) & (nonzeroy <y_high) & (nonzerox >=xL_left) & (nonzerox < xL_right)).nonzero()[0]

				if len(good_left_inds) > minpix:
					self.xL_current = np.int(np.mean(nonzerox[good_left_inds]))
				elif nonzeroy[left_lane_inds] != [] and nonzerox[left_lane_inds] != []:
					p_left = np.polyfit(nonzeroy[left_lane_inds], nonzerox[left_lane_inds], 2) 
					self.xL_current = np.int(np.polyval(p_left, y_high))
				
				left_lane_inds.extend(good_left_inds)

				# decide x_location
				if y_low >= decide_x_location_idx[0] and y_low < decide_x_location_idx[1]:	
					self.xL_Old_current = self.xL_current
					self.xR_Old_current = self.xL_current + (2 * line_Width)
					self.x_location = self.xL_current + line_Width
					cv2.circle(out_img, (self.x_location,int(np.mean(decide_x_location_idx))),2,(0,255,0),2)
	
		# when only right start inds is good		
		elif right_flag == True:
			for window in range(nwindows):
				y_low = self.y_current - (window + 1) * window_height
				y_high = self.y_current - window * window_height
				
				xR_left = self.xR_current - margin
				xR_right = self.xR_current + margin
				
				cv2.rectangle(out_img, (xR_left,y_low),(xR_right,y_high),(0, 0, 255),1)
				
				good_right_inds = ((nonzeroy >= y_low) & (nonzeroy <y_high) & (nonzerox >=xR_left) & (nonzerox < xR_right)).nonzero()[0]
				
				if len(good_right_inds) > minpix:
					self.xR_current = np.int(np.mean(nonzerox[good_right_inds]))
				elif nonzeroy[left_lane_inds] != [] and nonzerox[left_lane_inds] != []:
					p_left = np.polyfit(nonzeroy[left_lane_inds], nonzerox[left_lane_inds], 2) 
					self.xL_current = np.int(np.polyval(p_left, y_high))

				left_lane_inds.extend(good_left_inds)
				
				# decide x_location
				if y_low >= decide_x_location_idx[0] and y_low < decide_x_location_idx[1]:	
					self.xL_Old_current = self.xR_current - (2 * line_Width)
					self.xR_Old_current = self.xR_current
					self.x_location = self.xR_current - line_Width
					cv2.circle(out_img, (self.x_location,int(np.mean(decide_x_location_idx))),2,(0,0,255),2)
		
		# left, right all bad 
		else:
			self.x_location = self.x_location_old


		#print("XL : ",self.xL_current,"         XR : " ,self.xR_current,'  X Location : ',self.x_location)

		#if old and current diff is large too much use x_location_old
		if abs(self.x_location_old - self.x_location) > 100:
			self.x_location = self.x_location_old
			#need for error
		else:
			self.x_location_old = self.x_location
		self.x_location_old = self.x_location
		
		#cv2.imshow('a',out_img)
		#cv2.waitKey(1)
		
		return out_img,self.x_location
