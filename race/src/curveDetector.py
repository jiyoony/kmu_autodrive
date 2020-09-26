import time

class CurveDetector:
	
	def __init__(self):
		self.curve_count = 0
		self.time_old = 0
		self.pid_list = [0.0 for i in range(30)]
		#self.pid_sum_list = []
		self.initFlag = False
		self.initTime_old = 0
		self.start_time = time.time()
		self.fuck = 0

	def check_initFlag(self):
		if time.time() - self.initTime_old < 15:
			return True
		else:
			return False


	def check_time(self):
		if time.time()-self.time_old < 5:
			return False

		else:
			return True
	
	def list_update(self,pid):
		if time.time() - self.start_time < 5:
			return
		self.pid_list.pop(0)
		self.pid_list.append(pid)
			

	def count_curve(self):
		#self.pid_sum_list.append(sum(self.pid_list))
		if self.check_time() and not self.check_initFlag():
			print('pid list : ',abs(sum(self.pid_list)))
			if abs(sum(self.pid_list)) > 1:
				self.time_old = time.time()
				self.curve_count += 1
				self.fuck += 1
		elif self.check_time():
			if abs(sum(self.pid_list)) > 1:
				self.time_old = time.time()
				self.fuck += 1

