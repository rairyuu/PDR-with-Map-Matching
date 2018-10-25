import multiprocessing
import threading
from scipy import signal
from map_matching import *


class Worker(multiprocessing.Process):
	def __init__(self, _conn1):
		multiprocessing.Process.__init__(self)

		self.conn1 = _conn1

		return

	def WritePrintLog(self, _str):
		print _str
		log_file = open(self.log_filename, 'a')
		log_file.write('\n')
		log_file.write(_str)
		log_file.close()

		return

	def Init(self):
		self.log_filename = 'log\\log_worker.txt'

		self.sensor_data_process = []
		self.sampling_rate = 400
		self.delta_time = 1.0 / self.sampling_rate
		self.beta = 0.1
		self.epsilon = 0.001

		self.gravity = 9.7
		self.magic_number_scale = 5.1
		self.scale = self.gravity * self.magic_number_scale
		self.magic_number_scale_single_integration = 2.8
		self.scale_single_integration = self.gravity * self.magic_number_scale_single_integration

		LP_Cutoff = 2.0
		self.LP_b, self.LP_a = signal.butter(2, 2.0 * LP_Cutoff * self.delta_time, 'low')

		PLP_Cutoff = 0.05
		self.PLP_b, self.PLP_a = signal.butter(1, 2.0 * PLP_Cutoff * self.delta_time, 'low')

		self.min_length_for_process = 400

		self.reserve_data_length = 200

		self.walking = False

		self.step_number = 0

		self.map_matching = MapMatching()
		self.log_list = []

		self.process_thread = None
		self.reading_data = False
		self.modifying_data = False

		return

	def run(self):
		self.Init()

		started = False

		while True:
			temp_data = self.conn1.recv()
			if temp_data == True:
				self.WritePrintLog('Terminating process thread...')
				if self.process_thread is not None:
					self.process_thread.join()
				self.WritePrintLog('Worker offline...')
				self.WritePrintLog('Step number: %d' % self.step_number)
				break
			elif temp_data[0] == True:
				self.current_pose = Quaternion()
				self.current_pose.Set(temp_data[1][0], temp_data[1][1], temp_data[1][2], temp_data[1][3])
				started = True
				print 'Worker online...'
				log_file = open(self.log_filename, 'w')
				log_file.write('Worker online...')
				log_file.close()
				self.WritePrintLog(
					'Received data: Quaternion(%.3f, %.3f, %.3f, %.3f)' %
					(
						temp_data[1][0],
						temp_data[1][1],
						temp_data[1][2],
						temp_data[1][3]
					)
				)
			elif started:
				self.modifying_data = True
				if self.reading_data:
					self.modifying_data = True
				self.sensor_data_process.extend(temp_data[1])
				if temp_data[0] < 0:
					self.log_list.append(abs(temp_data[0]))
				self.modifying_data = False
				self.WritePrintLog('Received sensor data, length: %d' % (len(temp_data[1]) // 7))
				if self.map_matching.start_pressure < 0.0:
					self.map_matching.start_pressure = 0.0
					temp_data_length = len(temp_data[1]) // 7
					for i in range(temp_data_length):
						self.map_matching.start_pressure += temp_data[1][i * 7 + 6]
					self.map_matching.start_pressure /= temp_data_length
					self.WritePrintLog('Start pressure: %.3f' % self.map_matching.start_pressure)
				if (self.process_thread is None) or (not self.process_thread.isAlive()):
					self.map_matching.ShowMap()
					self.process_thread = threading.Thread(target=self.StepDetection)
					self.process_thread.start()

		self.map_matching.Release()

		return

	def StepDisplacement(self, _data, _type=0):
		displacement = Vector3()

		delta_time = self.delta_time

		data_length = len(_data) // 4

		velocity_x = 0.0
		velocity_y = 0.0

		for i in range(data_length):
			velocity_x += _data[i * 4] * delta_time
			velocity_y += _data[i * 4 + 1] * delta_time
			displacement.x += velocity_x * delta_time
			displacement.y += velocity_y * delta_time
			displacement.z += _data[i * 4 + 3]

		displacement.x *= -self.scale
		displacement.y *= -self.scale
		displacement.z /= data_length

		if _type == -1:
			displacement.x *= -0.5
			displacement.y *= -0.5
		elif _type == -2:
			displacement.x *= 0.5
			displacement.y *= 0.5

		self.map_matching.Update(displacement.x, displacement.y, displacement.z)

		self.step_number += 1

		return displacement

	def StepDetection(self):
		self.reading_data = True
		if self.modifying_data:
			self.reading_data = True
		data_process = self.sensor_data_process
		self.sensor_data_process = []
		self.map_matching.log_list.extend(self.log_list)
		self.log_list = []
		self.reading_data = False

		self.map_matching.WritePosition()

		data_length = len(data_process) // 7
		if data_length < 800:
			# BUG in real time mode, should return data_process to self.sensor_data_process here
			# Fixed
			self.reading_data = True
			if self.modifying_data:
				self.reading_data = True
			data_process.extend(self.sensor_data_process)
			self.sensor_data_process = data_process
			self.reading_data = False
			return
		current_pose = self.current_pose
		delta_time = self.delta_time
		beta = self.beta
		epsilon = self.epsilon

		acc_norm = []
		pressure = []
		for i in range(data_length):
			acc_norm.append(
				math.sqrt(
					data_process[i * 7 + 3] ** 2 + data_process[i * 7 + 4] ** 2 + data_process[i * 7 + 5] ** 2
				)
			)
			pressure.append(data_process[i * 7 + 6])
		acc_LP = signal.filtfilt(self.LP_b, self.LP_a, acc_norm)
		pressure_LP = signal.filtfilt(self.PLP_b, self.PLP_a, pressure)

		temp_detected_step = []
		step_begin = 0
		step_mid = 0

		grad_threshold = 0.1
		length_threshold_min = self.sampling_rate * 0.3
		length_threshold_max = self.sampling_rate * 0.8

		for i in range(1, data_length - self.reserve_data_length):
			if (acc_LP[i] < acc_LP[i - 1]) and (acc_LP[i] < acc_LP[i + 1]):
				step_mid = i
			if (acc_LP[i] > acc_LP[i - 1]) and (acc_LP[i] > acc_LP[i + 1]):
				if (acc_LP[step_begin] - acc_LP[step_mid] >= grad_threshold) and (acc_LP[i] - acc_LP[step_mid] >= grad_threshold):
					if (i - step_begin >= length_threshold_min) and (i - step_begin <= length_threshold_max):
						temp_detected_step.append((step_begin, step_mid, i))
				step_begin = i

		detected_step = []

		if self.walking and len(temp_detected_step) == 0:
			detected_step.append((0, -2, int(self.sampling_rate * 0.5)))

		if len(temp_detected_step) > 0:
			last_index = 0
			for temp_step in temp_detected_step:
				if last_index == 0:
					if self.walking:
						if temp_step[0] > self.sampling_rate:
							detected_step.append((0, -2, int(self.sampling_rate * 0.5)))
							detected_step.append((temp_step[0] - int(self.sampling_rate * 0.5), -1, temp_step[0]))
					else:
						if temp_step[0] > int(self.sampling_rate * 0.3):
							detected_step.append((max(0, temp_step[0] - int(self.sampling_rate * 0.5)), -1, temp_step[0]))
				else:
					if temp_step[0] - last_index > self.sampling_rate:
						detected_step.append((last_index, -2, last_index + int(self.sampling_rate * 0.5)))
						detected_step.append((temp_step[0] - int(self.sampling_rate * 0.5), -1, temp_step[0]))
				detected_step.append(temp_step)
				last_index = temp_step[2]

			if data_length - self.reserve_data_length - temp_detected_step[-1][2] > self.sampling_rate:
				detected_step.append((temp_detected_step[-1][2], -2, temp_detected_step[-1][2] + int(self.sampling_rate * 0.5)))

		last_index = 0
		deg2rad = math.pi / 180.0
		for temp_step in detected_step:
			if temp_step[0] > last_index:
				for i in range(last_index, temp_step[0]):
					current_pose.Update(
						data_process[i * 7] * deg2rad, data_process[i * 7 + 1] * deg2rad, data_process[i * 7 + 2] * deg2rad,
						data_process[i * 7 + 3], data_process[i * 7 + 4], data_process[i * 7 + 5],
						delta_time,
						beta,
						epsilon
					)
			temp_data = []
			for i in range(temp_step[0], temp_step[2]):
				current_pose.Update(
					data_process[i * 7] * deg2rad, data_process[i * 7 + 1] * deg2rad, data_process[i * 7 + 2] * deg2rad,
					data_process[i * 7 + 3], data_process[i * 7 + 4], data_process[i * 7 + 5],
					delta_time,
					beta,
					epsilon
				)
				acc_linear = current_pose.RotateVector(data_process[i * 7 + 3], data_process[i * 7 + 4], data_process[i * 7 + 5])
				temp_data.append(acc_linear.x)
				temp_data.append(acc_linear.y)
				temp_data.append(acc_linear.z)
				temp_data.append(pressure_LP[i])
			step_disp = self.StepDisplacement(temp_data, temp_step[1])
			last_index = temp_step[2]

		if len(detected_step) > 0:
			if detected_step[-1][1] == -2:
				self.walking = False
			else:
				self.walking = True
			data_process = data_process[(last_index * 7):]

		self.reading_data = True
		if self.modifying_data:
			self.reading_data = True
		data_process.extend(self.sensor_data_process)
		self.sensor_data_process = data_process
		self.reading_data = False

		return
