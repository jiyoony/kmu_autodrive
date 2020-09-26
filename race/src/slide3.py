import numpy as np
import cv2



class SlideWindow:
	def __init__(self):
		self.x_current = 0
		self.y_current = 0
		self.x_Old_Loca = 0
		self.steer = 0

	def slidewindow(self,img):
		#value
		x_location = 320
		current_Margin = 110
		tmp = 0
		#img process--
		out_img = np.dstack((img,img,img))
		height = img.shape[0]
		width = img.shape[1]
		window_height = 10
		nwindows = 10
		margin = 15
		minpix = 5
		
		nonzero = img.nonzero()

		nonzeroy = np.array(nonzero[0])
		nonzerox = np.array(nonzero[1])
		center_list = []

		left_lane_inds = []
       		right_lane_inds = []
		######################################set parameter


		#for visualize

		#left
		pts_left = np.array([[width/2 - 90, height], [width/2 -90,height - 60], [width/2-200,height-80],[width/2-200,height]],np.int32)

		cv2.polylines(out_img,[pts_left],False, (0,255,0),1)
		
		#right
		pts_right = np.array([[width/2 + 60, height], [width/2+60, height - 80],[width/2 + 190,height - 110],[width/2+ 190, height]],np.int32)

		cv2.polylines(out_img,[pts_right],False,(255,0,0),1)




		######################################detect line


		good_left_inds = ((nonzerox >= width/2 - 160) & (nonzeroy >= nonzerox * 0.33+280) & (nonzerox <= width/2 - 100)).nonzero()[0]

		good_right_inds = ((nonzerox >= width/2 + 50) & (nonzeroy >= nonzerox * (-0.48) + 460) & (nonzerox <= width/2 + 180)).nonzero()[0]



		#####################################





		if len(good_left_inds) > minpix and len(good_left_inds)> len(good_right_inds):
			line_flag = 1
			self.x_current = np.int(np.mean(nonzerox[good_left_inds]))
			self.y_current = np.int(np.mean(nonzeroy[good_left_inds]))
		elif len(good_right_inds) > minpix and len(good_left_inds) < len(good_right_inds):
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
						
					elif nonzeroy[left_lane_inds] != [] and nonzerox[left_lane_inds] != []:
						p_left = np.polyfit(nonzeroy[left_lane_inds], nonzerox[left_lane_inds], 2) 
                       				x_current = np.int(np.polyval(p_left, y_high))
				else:
					

					y_low = self.y_current - (window+1) * window_height
					y_high = self.y_current - window* window_height

					x_left = self.x_current -margin
					x_right = self.x_current + margin


					cv2.rectangle(out_img, (x_left, y_low),(x_right + margin,y_high),(255,0,0),1)

					good_right_inds = ((nonzeroy >=y_low) & (nonzeroy < y_high) & (nonzerox > x_left) & (nonzerox < x_right)).nonzero()[0]


					if len(good_right_inds) > minpix:
						self.x_current = np.int(np.mean(nonzerox[good_right_inds]))

					elif nonzeroy[right_lane_inds] != [] and nonzerox[right_lane_inds] != []:
			                        p_right = np.polyfit(nonzeroy[right_lane_inds], nonzerox[right_lane_inds], 2)
					
				left_lane_inds.extend(good_left_inds)



		

		
				if line_flag == 1:
					x_location = self.x_current + 120
					self.x_Old_Loca = x_location

				elif line_flag == 2:
					x_location = self.x_current - 120
					self.x_Old_Loca = x_location

				else:
					x_location = self.x_Old_Loca

				center_list.append([x_location,(y_low+y_high)//2])
				
			
		center_list_x = []
		center_list_y = []
		for i in range(len(center_list)):
			center_list_x.append(center_list[i][0])
			center_list_y.append(center_list[i][1])
		
		if (center_list_x != [] and center_list_y != []):
			print("Here????")
			change_Y = np.mean(center_list_y) - center_list_y[0]
			change_X = center_list_x[-1] - center_list_x[0]
			if change_X == 0: tmp = 0
			else: tmp = np.degrees(np.arctan(change_Y / change_X))
			self.steer = tmp

		else:
			print("Here~!!!!!!!!!!!!!!!!")
			tmp = self.steer

		print("Tmp : ", tmp * 0.33)

		for i in range(len(center_list_x)):
			#print('i : ', int(result_x[i]))
			#print('y : ',center_list[1][i])
			cv2.circle(out_img,(int(center_list_x[i]),center_list_y[i]),3, (0,0,255),-1)
		cv2.circle(out_img, (x_location,380),2,(0,255,255),2)
		
		return out_img, tmp
