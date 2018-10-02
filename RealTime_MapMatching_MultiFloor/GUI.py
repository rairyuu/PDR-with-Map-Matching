import socket
import OSC
import time
from worker import *


class GUI(object):
	def __init__(self):
		self.window_width = 640
		self.window_height = 360
		self.image_show = np.ones((self.window_height, self.window_width, 3), dtype='uint8') * 255

		self.font_type = cv2.FONT_HERSHEY_SIMPLEX
		self.font_scale = 0.8
		self.text_position = (80, 150)
		self.text_color = (120, 20, 20)
		self.font_scale_2 = 0.7
		self.text_position_2 = (80, 200)
		self.text_color_2 = (0, 0, 0)

		self.current_state = 0

		self.server = None
		self.server_thread = None
		self.connected = False

		self.sensor_data_file = None

		self.current_pose = Quaternion()

		self.started = False
		self.tracking = False
		self.data_process = []

		self.log_filename = 'log\\Timer.txt'
		log_file = open(self.log_filename, 'w')
		log_file.write('Start')
		log_file.close()

		self.reading_data = False
		self.modifying_data = False

		conn = multiprocessing.Pipe()
		self.conn0 = conn[0]
		self.conn1 = conn[1]

		self.worker = Worker(self.conn1)
		self.worker.start()

		return

	def WriteLog(self, _str):
		log_file = open(self.log_filename, 'a')
		log_file.write(_str)
		log_file.close()

		return

	def Refresh(self):
		self.image_show = np.ones((self.window_height, self.window_width, 3), dtype='uint8') * 255

		return

	def WifiCallback(self, add, tags, args, source):
		self.connected = True

		return

	def SensorsCallback(self, add, tags, args, source):
		if (not self.started) or self.reading_data:
			return

		self.modifying_data = True

		self.sensor_data_file.write(
			'\n%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f' %
			(
				timeit.default_timer(),
				args[0],
				args[1],
				args[2],
				args[3],
				args[4],
				args[5],
				args[6],
				args[7],
				args[8],
				args[9]
			)
		)

		if self.tracking:
			self.data_process.append(args[0])
			self.data_process.append(args[1])
			self.data_process.append(args[2])
			self.data_process.append(args[3])
			self.data_process.append(args[4])
			self.data_process.append(args[5])
			self.data_process.append(args[9])
		else:
			deg2rad = math.pi / 180.0
			self.current_pose.Update(
				args[0] * deg2rad, args[1] * deg2rad, args[2] * deg2rad,
				args[3], args[4], args[5],
				0.0025,
				0.5,
				0.001
			)

		self.modifying_data = False

		return

	def ConnectIMU(self):
		send_address_port = ('192.168.1.1', 9000)
		client = OSC.OSCClient()
		client.connect(send_address_port)
		msg = OSC.OSCMessage()
		msg.setAddress('/wifi/send/ip')
		receive_address = socket.gethostbyname(socket.gethostname())
		msg.append(receive_address)
		client.send(msg)
		client.close()

		receive_address_port = (receive_address, 8001)
		try:
			self.server = OSC.OSCServer(receive_address_port)
		except socket.error:
			print('Cannot connect to IMU')
			return False

		self.server.addDefaultHandlers()
		self.server.addMsgHandler('/wifi/send/ip', self.WifiCallback)
		self.server.addMsgHandler('/sensors', self.SensorsCallback)

		self.server_thread = threading.Thread(target=self.server.serve_forever)
		self.server_thread.start()

		time.sleep(5.0)

		if not self.connected:
			print('Cannot connect to IMU')
			self.server.close()
			self.server_thread.join()

			return False

		return True

	def DisconnectIMU(self):
		self.server.close()
		self.server_thread.join()

		self.connected = False

		return

	def Run(self):
		KEY_SPACE = 32
		KEY_ESCAPE = 27

		string_show = ''
		string_show_2 = ''

		last_time = timeit.default_timer()

		last_time_SR = -1.0

		current_log_index = 1
		taking_log = False

		while True:
			if self.current_state == 0:
				string_show = 'Press SPACE to connect the IMU'
			elif self.current_state == 1:
				string_show = 'Connecting...'
				string_show_2 = ''
			elif self.current_state == 2:
				string_show = 'Press SPACE to calibrate IMU'
			elif self.current_state == 3:
				string_show = 'Press SPACE to start tracking'
			elif self.current_state == 4:
				string_show = 'Press SPACE to log index %i' % current_log_index
				string_show_2 = 'Press ESC to end tracking'
			else:
				string_show = 'Tracking is end'
				string_show_2 = 'Double press ESC to exit the system'

			self.Refresh()
			cv2.putText(self.image_show, string_show, self.text_position, self.font_type, self.font_scale, self.text_color, thickness=2)
			cv2.putText(self.image_show, string_show_2, self.text_position_2, self.font_type, self.font_scale_2, self.text_color_2, thickness=1)
			cv2.imshow('DataTaker', self.image_show)
			rKey = cv2.waitKey(delay=200)

			if rKey == KEY_ESCAPE:
				if self.current_state == 4:
					self.reading_data = True
					while self.modifying_data:
						self.reading_data = True
					self.started = False
					self.tracking = False
					self.data_process = []
					self.DisconnectIMU()
					self.WriteLog('\nEnd,%.3f' % timeit.default_timer())
					self.sensor_data_file.close()
					self.sensor_data_file = None
					self.conn0.send(True)
					self.worker.join()
					self.worker = None
					self.reading_data = False
					self.current_state += 1
				elif self.current_state == 5:
					cv2.imshow('DataTaker', self.image_show)
					rKey = cv2.waitKey(delay=200)
					if rKey == KEY_ESCAPE:
						break

			if self.current_state == 1:
				if self.ConnectIMU():
					self.current_state += 1
				else:
					string_show_2 = 'Failed to connect the IMU'
					self.current_state = 0

			if self.current_state == 4:
				current_time = timeit.default_timer()
				if current_time - last_time > 1.5:
					self.reading_data = True
					while self.modifying_data:
						self.reading_data = True
					temp_data = self.data_process
					self.data_process = []
					self.reading_data = False
					if last_time_SR < 0.0:
						last_time_SR = current_time
					else:
						temp_delta_time = current_time - last_time_SR
						temp_sampling_rate = len(temp_data) / (7 * temp_delta_time)
						last_time_SR = current_time
						print 'Current sampling rate: %.3f' % temp_sampling_rate
					temp_index = 0
					if taking_log:
						temp_index = current_log_index
						current_log_index += 1
						taking_log = False
					self.conn0.send([-temp_index, temp_data])
					last_time = current_time

			if rKey == KEY_SPACE:
				if self.current_state == 0:
					self.current_state += 1
				elif self.current_state == 2:
					self.WriteLog('\nCalibrate,%.3f' % timeit.default_timer())
					log_filename = 'log\\sensors.csv'
					self.sensor_data_file = open(log_filename, 'w')
					self.sensor_data_file.write('time,gx,gy,gz,ax,ay,az,mx,my,mz,p')
					self.started = True
					self.current_state += 1
				elif self.current_state == 3:
					self.reading_data = True
					while self.modifying_data:
						self.reading_data = True
					self.WriteLog('\nStart,%.3f' % timeit.default_timer())
					self.tracking = True
					self.current_state += 1
					self.conn0.send(
						[
							True,
							(self.current_pose.w, self.current_pose.x, self.current_pose.y, self.current_pose.z)
						]
					)
					self.reading_data = False
					last_time = timeit.default_timer() + 1.5
				elif self.current_state == 4:
					taking_log = True
					print 'Taking log...'

		return
