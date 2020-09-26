#Find Stop Line
def findStop(roi, img):#plus : mark_Cnt
	global car_run_speed
	global old_time
	#Using HSV
	stopFlag = 0
	nonzero = roi.nonzero()

	lower_yellow = np.array([-20, 80, 80])
	upper_yellow = np.array([50, 255, 255])

	img = img[240:, :]
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
	res = cv2.bitwise_and(img, img, mask = mask_yellow)
	#cv2.imshow('res',res)
	cv2.imshow('roi', roi)
	resnon = res.nonzero()
	resx = np.array(resnon[1])
	goodres = ((resx > 200) &(resx < 440)).nonzero()[0]
	print("NONZERO : ", len(nonzero[0]))

	if len(nonzero[0]) > 800:
		stopFlag = 1
		if (len(goodres)):
			stopFlag = 2

	if stopFlag == 1:
		if car_run_speed - car_run_speed * 0.5 > 0: 
			if car_run_speed < 0.0005: car_run_speed = 0
			else: 
				car_run_speed = 0
				time.sleep(0.2)
		else:
			print("Stop!")
			time.sleep(4)
			car_run_speed = 6
	#if stopFlag == 2:
		#if (mark_Cnt > 15): car_run_spped /= 0.5

'''
#Not Using HSV
	self.stopCnt = 0
	nonzero = roi.nonzero()

	cv2.imshow('roi', roi)
	goodres = ((resx > 200) &(resx < 440)).nonzero()[0]
	print("NONZERO : ", len(nonzero[0]))

	if len(nonzero[0]) > 800:
		if self.stopCnt % 3 == 2:
			if car_run_speed - car_run_speed * 0.5 > 0: 
				if car_run_speed < 0.0005: car_run_speed = 0
				else: 
					car_run_speed = 0
					time.sleep(0.2)
			else:
				print("Stop!")
				time.sleep(4)
				car_run_speed = 6
'''

	return stopFlag
