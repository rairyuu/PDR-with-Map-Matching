import multiprocessing
import threading
from scipy import signal
from map_matching import *


class Worker(object):
	def __init__(self):

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

		return

	def Run(self):
		self.Init()

		in_file = open('Timer.txt', 'r')
		in_file.readline()
		temp_line = in_file.readline()
		time_calibrate = float(temp_line.split(',')[1])
		temp_line = in_file.readline()
		time_start = float(temp_line.split(',')[1])
		temp_line = in_file.readline()
		time_end = float(temp_line.split(',')[1])
		in_file.close()

		in_file = open('sensors.csv', 'r')
		in_file.readline()
		self.current_pose = Quaternion()
		deg2rad = math.pi / 180.0
		temp_data_buffer = []
		buffer_size = 0
		buffer_size_max = 800
		for temp_line in in_file.readlines():
			temp_seq = temp_line.split(',')
			temp_time = float(temp_seq[0])
			temp_data = []
			for i in range(1, 7):
				temp_data.append(float(temp_seq[i]))
			temp_data.append(float(temp_seq[10]))
			if temp_time > time_end:
				break
			elif temp_time > time_start:
				temp_data_buffer.extend(temp_data)
				buffer_size += 1
				if buffer_size > buffer_size_max:
					if self.map_matching.start_pressure < 0.0:
						self.map_matching.start_pressure = 0.0
						for i in range(buffer_size):
							self.map_matching.start_pressure += temp_data_buffer[i * 7 + 6]
						self.map_matching.start_pressure /= buffer_size
					self.sensor_data_process.extend(temp_data_buffer)
					self.StepDetection()
					temp_data_buffer = []
					buffer_size = 0
			elif temp_time > time_calibrate:
				self.current_pose.Update(
					temp_data[0] * deg2rad, temp_data[1] * deg2rad, temp_data[2] * deg2rad,
					temp_data[3], temp_data[4], temp_data[5],
					self.delta_time,
					self.beta,
					self.epsilon
				)
		in_file.close()

		self.map_matching.Release()

		return

	def StepDisplacement(self, _data, _type=0):
		# data: acc_x, acc_y, acc_z, pressure,...
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

		# Scale first step (start to move) and last step (stop)
		if _type == -1:
			displacement.x *= -0.5
			displacement.y *= -0.5
		elif _type == -2:
			displacement.x *= 0.5
			displacement.y *= 0.5

		self.map_matching.Update(displacement.x, displacement.y, displacement.z)
		self.map_matching.ShowMap()

		self.step_number += 1

		return displacement

	def StepDetection(self):
		data_process = self.sensor_data_process
		self.map_matching.WritePosition()

		data_length = len(data_process) // 7
		if data_length < 800:
			# BUG in real time mode, I should return data_process to self.sensor_data_process here
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

		self.sensor_data_process = data_process

		return


if __name__ == '__main__':
	worker = Worker()
	worker.Run()
