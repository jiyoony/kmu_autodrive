import numpy as np
import cv2
import math


class SlideWindow:
	def __init__(self):
		self.x_current = 0
		self.y_current = 0
		self.steer = 0

	def slidewindow(self,img):
		#value
		x_location = 320
		current_Margin = 110

		x_CurrentLst = []
		
		#img process--
		out_img = np.dstack((img,img,img))
		height = img.shape[0]
		width = img.shape[1]

		window_height = 10
		nwindows = 10

		center_x = list()
		center_y = list()
		
		nonzero = img.nonzero()

		nonzeroy = np.array(nonzero[0])
		nonzerox = np.array(nonzero[1])
		left_list =[]
		right_list =[]
		center_list = []
		margin = 20
		
		minpix = 5
		left_lane_inds = []
       		right_lane_inds = []
		######################################set parameter


		#for visualize

		#left
		pts_left = np.array([[width/2 - 50, height], [width/2 -50,height - 60], [width/2-140,height-80],[width/2-140,height]],np.int32)

		cv2.polylines(out_img,[pts_left],False, (0,255,0),1)
		
		#right
		pts_right = np.array([[width/2 + 50, height], [width/2+50, height - 80],[width/2 + 185,height - 110],[width/2+ 185, height]],np.int32)

		cv2.polylines(out_img,[pts_right],False,(255,0,0),1)




		######################################detect line


		good_left_inds = ((nonzerox >= width/2 - 140) & (nonzeroy >= 400) & (nonzerox <= width/2 - 50)).nonzero()[0]

		good_right_inds = ((nonzerox >= width/2 + 50) & (nonzeroy >= nonzerox * (-0.48) + 460) & (nonzerox <= width/2 + 185)).nonzero()[0]



		#####################################



		if len(good_left_inds) > minpix:# and len(good_left_inds)> len(good_right_inds):
			line_flag = 1
			self.x_current = np.int(np.mean(nonzerox[good_left_inds]))
			self.y_current = np.int(np.mean(nonzeroy[good_left_inds]))
		elif len(good_right_inds) > minpix:# and len(good_left_inds) < len(good_right_inds):
			line_flag = 2
			self.x_current = nonzerox[good_right_inds[np.argmax(nonzeroy[good_right_inds])]]
			self.y_current = np.int(np.max(nonzeroy[good_right_inds]))
		else: line_flag = 3
			
		################################### detect logic

	
		if line_flag != 3:
			# find line least 1
			for window in range(nwindows):
				if line_flag == 1:   #left lane found
					y_low = self.y_current - (window + 1) * window_height
					y_high = self.y_current - window*window_height
					
					x_left = self.x_current - margin
					x_right = self.x_current + margin

					cv2.rectangle(out_img, (x_left,y_low),(x_right,y_high),(0,255,0),1)
					good_left_inds = ((nonzeroy >= y_low) & (nonzeroy <y_high) & (nonzerox >=x_left) & (nonzerox < x_right)).nonzero()[0]
					

					if len(good_left_inds) > minpix:
						self.x_current = np.int(np.mean(nonzerox[good_left_inds]))
						
						#print('findleft and minpix mode')
					elif nonzeroy[left_lane_inds] != [] and nonzerox[left_lane_inds] != []:
						p_left = np.polyfit(nonzeroy[left_lane_inds], nonzerox[left_lane_inds], 2) 
                       				self.x_current = np.int(np.polyval(p_left, y_high))
					#left_list.append([nonzerox[left_lane_inds],nonzeroy[left_lane_inds]])
				
					#if y_low >= 338 and y_low < 344:
                    # 0.165 is the half of the road(0.33)
                        		#	x_location = self.x_current + int(width * 0.175)
						#center_list.append([x_location,self.y_current]) 

				else:
					

					y_low = self.y_current - (window+1) * window_height
					y_high = self.y_current - window* window_height

					x_left = self.x_current -margin
					x_right = self.x_current + margin


					cv2.rectangle(out_img, (x_left, y_low),(x_right + margin,y_high),(255,0,0),1)

					good_right_inds = ((nonzeroy >=y_low) & (nonzeroy < y_high) & (nonzerox > x_left) & (nonzerox < x_right)).nonzero()[0]


					if len(good_right_inds) > minpix:
						self.x_current = np.int(np.mean(nonzerox[good_right_inds]))
						#print('find Right and minpix mode')
					elif nonzeroy[right_lane_inds] != [] and nonzerox[right_lane_inds] != []:
			                        p_right = np.polyfit(nonzeroy[right_lane_inds], nonzerox[right_lane_inds], 2)
						self.x_current = np.int(np.polyval(p_right, y_high))
					
					#if y_low >= 338 and y_low < 344:
                    # 0.165 is the half of the road(0.33)
                        		#	x_location = self.x_current - int(width * 0.175) 
						#center_list.append([x_location,self.y_current]) 
				left_lane_inds.extend(good_left_inds)
        		        right_lane_inds.extend(good_right_inds)  
				#center_list.append([x_location,self.y_current])
            		#left_lane_inds = np.concatenate(left_lane_inds)
            		#right_lane_inds = np.concatenate(right_lane_inds)


			

		
				if line_flag == 1:
					x_location = self.x_current + 105
					self.x_Old_Loca = x_location

				elif line_flag == 2:
					x_location = self.x_current - 105
					self.x_Old_Loca = x_location

				else:
					x_location = self.x_Old_Loca

				center_list.append([x_location,(y_low+y_high)//2])
			#x_CurrentLst.append(self.x_current)	
		
		center_list_x = []
		center_list_y = []
		for i in range(len(center_list)):
			center_list_x.append(center_list[i][0])
			center_list_y.append(center_list[i][1])
		#print(center_list2)
		
		if (center_list_x and center_list_y):
			#fp1 = np.polyfit(center_list_y,center_list_x,1)
			#tmp = math.atan(fp1[0])

			fp1 = np.polyfit(center_list_y,center_list_x,1)
			f1 = np.poly1d(fp1)
			x_location = int(f1(380))
			#print("steer_slide : ", tmp)
			#f1 = np.poly1d(fp1)
			# result_x = f1(center_list[1])
			#result_x = np.polyval(fp1, center_list_y)
			#tmp = np.polyval(fp1, 390)
			#if (tmp < 0.05): tmp = 0
			self.steer = x_location
		else:
			x_location = self.steer
		print(x_location)
		'''
		for i in range(len(center_list_x)):
			#print('i : ', int(result_x[i]))
			#print('y : ',center_list[1][i])
			cv2.circle(out_img,(int(center_list_x[i]),center_list_y[i]),3, (0,0,255),-1)
		'''
		cv2.circle(out_img, (x_location,370),2,(0,255,255),2)
		"""
		if line_flag == 1 : 
			x_left = int(np.mean(x_CurrentLst))
			x_right = x_left + 240
		else:
			x_right = int(np.mean(x_CurrentLst))
			x_left = x_right - 240
		"""
		cv2.imshow('aaa',out_img)
		return out_img, x_location#, center_list_y[-1], x_left, x_right
