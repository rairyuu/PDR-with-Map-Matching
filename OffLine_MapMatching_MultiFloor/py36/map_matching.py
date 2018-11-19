from quaternion import *
import timeit
import numpy as np
import cv2
import random


class Node(object):
	def __init__(self, _id, _layer, _x, _y):
		self.id = _id
		self.layer = _layer
		self.x = _x
		self.y = _y

		return


class Link(object):
	def __init__(self, _id, _type, _node0, _node1):
		self.id = _id
		self.type = _type
		self.layer = _node0.layer
		self.node0 = _node0
		self.node1 = _node1

		return


class Arrow(object):
	def __init__(self, _id, _type, _node0, _node1):
		self.id = _id
		# 0: down, 1: up, 2: both
		self.type = _type
		self.layer = _node0.layer
		self.node0 = _node0
		self.node1 = _node1

		return


class Step(object):
	def __init__(self, _dx, _dy, _pressure, _current_floor, _scale=1.0):
		self.dx = _dx * _scale
		self.dy = _dy * _scale
		self.pressure = _pressure
		self.current_floor = _current_floor

		self.current_position = None

		return


class Particle(object):
	def __init__(self, _x, _y, _pressure, _current_floor, _scale, _rotate_angle):
		self.x = _x
		self.y = _y
		self.pressure = _pressure
		self.current_floor = _current_floor

		self.scale = _scale
		self.rotate_angle = _rotate_angle
		self.rotation = Quaternion()
		self.rotation.RotateZ(_rotate_angle)

		return


class MapMatching(object):
	def __init__(self):
		map_path = 'map\\'
		map_filename = 'core.txt'

		self.floor_number = 1
		self.map_images = []
		self.floor_height = 5.0
		self.start_floor = 0
		self.start_x = 0.0
		self.start_y = 0.0
		self.dir_angle = 0.0
		self.meter_to_pixel = 1.0
		self.node_filename = None
		self.link_filename = None
		self.arrow_filename = None
		self.polygon_filename = None

		in_file = open(map_path + map_filename, 'r')
		in_file.readline()
		for tLine in in_file.readlines():
			tSeq = tLine.split(',')
			if len(tSeq) < 1:
				continue
			if tSeq[0] == 'floor':
				self.floor_number = int(tSeq[1])
				for i in range(self.floor_number):
					self.map_images.append(
						cv2.imread(
							map_path + tSeq[i + 2].split('\n')[0]
						)
					)
			elif tSeq[0] == 'height':
				self.floor_height = float(tSeq[1])
			elif tSeq[0] == 'start':
				self.start_floor = int(tSeq[1])
			elif tSeq[0] == 'start_point':
				self.start_x = float(tSeq[1])
				self.start_y = float(tSeq[2])
			elif tSeq[0] == 'dir':
				self.dir_angle = float(tSeq[1])
			elif tSeq[0] == 'scale':
				self.meter_to_pixel = 1.0 / float(tSeq[1])
			elif tSeq[0] == 'node':
				self.node_filename = tSeq[1].split('\n')[0]
			elif tSeq[0] == 'link':
				self.link_filename = tSeq[1].split('\n')[0]
			elif tSeq[0] == 'arrow':
				self.arrow_filename = tSeq[1].split('\n')[0]
			elif tSeq[0] == 'polygon':
				self.polygon_filename = tSeq[1].split('\n')[0]
		in_file.close()

		self.current_floor = self.start_floor
		self.current_position = Vector3()
		self.current_position.x = self.start_x
		self.current_position.y = self.start_y
		self.current_position.z = self.dir_angle
		self.delta_pressure_to_delta_height = -9.0
		self.start_pressure = -1.0
		self.pressure_gain = 0.0

		self.map_width = self.map_images[0].shape[1]
		self.map_height = self.map_images[0].shape[0]

		self.resize_width = self.map_width
		self.resize_height = self.map_height

		self.video_writer = cv2.VideoWriter('video.avi', cv2.VideoWriter_fourcc('X', 'V', 'I', 'D'), 20, (self.resize_width, self.resize_height), True)

		self.floor_changed = False
		self.last_floor = self.current_floor

		self.initial_rotation = Quaternion()
		self.initial_rotation.RotateZ(self.dir_angle)

		self.nodes = {}
		self.links = []
		self.arrows = []
		self.polygon_zone = []
		for i in range(self.floor_number):
			self.links.append([])
			self.arrows.append([])
			self.polygon_zone.append(
				np.ones(
					(self.map_height, self.map_width, 1),
					dtype=np.uint8
				) * 255
			)

		if self.node_filename is None:
			print('Error: node file is None...')
			return

		in_file = open(map_path + self.node_filename, 'r')
		in_file.readline()
		for tLine in in_file.readlines():
			tSeq = tLine.split(',')
			if len(tSeq) < 1:
				continue
			tNode = Node(
				int(tSeq[0]),
				int(tSeq[1]),
				float(tSeq[2]),
				float(tSeq[3])
			)
			self.nodes[tNode.id] = tNode
		in_file.close()

		if self.link_filename is None:
			print('Error: link file is None...')

		in_file = open(map_path + self.link_filename, 'r')
		in_file.readline()
		for tLine in in_file.readlines():
			tSeq = tLine.split(',')
			if len(tSeq) < 1:
				continue
			tLink = Link(
				int(tSeq[0]),
				int(tSeq[1]),
				self.nodes[int(tSeq[2])],
				self.nodes[int(tSeq[3])]
			)
			self.links[tLink.layer].append(tLink)
		in_file.close()

		if self.arrow_filename is None:
			print('Error: arrow file is None...')

		in_file = open(map_path + self.arrow_filename, 'r')
		in_file.readline()
		for tLine in in_file.readlines():
			tSeq = tLine.split(',')
			tArrow = Arrow(
				int(tSeq[0]),
				int(tSeq[1]),
				self.nodes[int(tSeq[2])],
				self.nodes[int(tSeq[3])]
			)
			self.arrows[tArrow.layer].append(tArrow)
		in_file.close()

		if self.polygon_filename is None:
			print('Error: polygon file is None...')

		in_file = open(map_path + self.polygon_filename, 'r')
		in_file.readline()
		for tLine in in_file.readlines():
			tSeq = tLine.split(',')
			temp_type = int(tSeq[1])
			if temp_type != 1:
				continue
			temp_node_number = int(tSeq[2])
			temp_points = []
			temp_layer = None
			for i in range(temp_node_number):
				tNode = self.nodes[int(tSeq[i + 3])]
				temp_points.append((int(tNode.x), int(tNode.y)))
				temp_layer = tNode.layer
			temp_points_array = np.array([temp_points], dtype=np.int32)
			cv2.fillPoly(self.polygon_zone[temp_layer], temp_points_array, temp_type)
		in_file.close()

		in_file = open(map_path + self.polygon_filename, 'r')
		in_file.readline()
		for tLine in in_file.readlines():
			tSeq = tLine.split(',')
			temp_type = int(tSeq[1])
			if temp_type == 1:
				continue
			temp_node_number = int(tSeq[2])
			temp_points = []
			temp_layer = None
			for i in range(temp_node_number):
				tNode = self.nodes[int(tSeq[i + 3])]
				temp_points.append((int(tNode.x), int(tNode.y)))
				temp_layer = tNode.layer
			temp_points_array = np.array([temp_points], dtype=np.int32)
			cv2.fillPoly(self.polygon_zone[temp_layer], temp_points_array, temp_type)
		in_file.close()

		self.steps = []

		self.particles = []
		for i in range(9):
			for j in range(5):
				self.particles.append(
					Particle(
						self.start_x,
						self.start_y,
						self.start_pressure + self.pressure_gain,
						self.start_floor,
						1.0 + (i - 4) * 0.025,
						self.dir_angle + (j - 2) * 0.05
					)
				)

		self.particle_number_max = 512
		particle_number = self.particle_number_max - 45
		for i in range(particle_number):
			self.particles.append(
				Particle(
					self.start_x,
					self.start_y,
					self.start_pressure + self.pressure_gain,
					self.start_floor,
					random.uniform(0.9, 1.1),
					self.dir_angle + random.uniform(-0.1, 0.1)
				)
			)

		self.view_particle = True

		self.window_name = 'Current Position'

		self.position_filename = 'log\\position.csv'
		log_file = open(self.position_filename, 'w')
		log_file.write('x,y,z')
		log_file.close()

		self.time_start = 1515688724000
		self.log_list = []

		self.IPIN_filename = 'log\\kyushu.csv'
		log_file = open(self.IPIN_filename, 'w')
		temp_longitude, temp_latitude, temp_floor = self.MapToWorld(self.current_position.x, self.current_position.y, self.current_floor)
		log_file.write(
			'%i,%.9f,%.9f,%i,%i' %
			(
				self.time_start + int(timeit.default_timer() * 1000),
				temp_longitude,
				temp_latitude,
				temp_floor,
				0
			)
		)
		log_file.close()

		return

	def Release(self):
		cv2.destroyWindow(self.window_name)

		self.video_writer.release()

		return

	def MapToWorld(self, _x, _y, _floor):
		homography = [
			[-2.10388667722e-05, -0.000262518473727, -1.63503055188],
			[0.000604240370159, 0.00774479195666, 47.2205108312],
			[1.27234617272e-05, 0.000163991977084, 1.0]
		]

		rx = homography[0][0] * _x + homography[0][1] * _y + homography[0][2]
		ry = homography[1][0] * _x + homography[1][1] * _y + homography[1][2]
		rw = homography[2][0] * _x + homography[2][1] * _y + homography[2][2]
		rx /= rw
		ry /= rw

		return rx, ry, _floor - 1

	def DistanceToLine(self, _line, _pt):
		line_length = math.sqrt((_line[0][0] - _line[1][0]) ** 2 + (_line[0][1] - _line[1][1]) ** 2)
		dist_0 = math.sqrt((_pt[0] - _line[0][0]) ** 2 + (_pt[1] - _line[0][1]) ** 2)
		dot_0 = (_pt[0] - _line[0][0]) * (_line[1][0] - _line[0][0]) + (_pt[1] - _line[0][1]) * (_line[1][1] - _line[0][1])
		line_projection = dot_0 / line_length

		if line_projection < 0.0:
			return math.sqrt((_pt[0] - _line[0][0]) ** 2 + (_pt[1] - _line[0][1]) ** 2)
		if line_projection > line_length:
			return math.sqrt((_pt[0] - _line[1][0]) ** 2 + (_pt[1] - _line[1][1]) ** 2)

		return math.sqrt(max(0.01, dist_0 ** 2 - line_projection ** 2))

	def IntersectBase(self, _x0, _y0, _x1, _y1, _x2, _y2, _x3, _y3):
		threshold = 0.1

		mid_x_0 = (_x0 + _x1) * 0.5
		mid_y_0 = (_y0 + _y1) * 0.5

		mid_x_1 = (_x2 + _x3) * 0.5
		mid_y_1 = (_y2 + _y3) * 0.5

		if (abs(mid_x_1 - mid_x_0) > abs(_x1 - mid_x_0) + abs(_x3 - mid_x_1) + threshold) or (abs(mid_y_1 - mid_y_0) > abs(_y1 - mid_y_0) + abs(_y3 - mid_y_1) + threshold):
			return False

		return True

	def Intersect(self, _x0, _y0, _x1, _y1, _x2, _y2, _x3, _y3):
		if not self.IntersectBase(_x0, _y0, _x1, _y1, _x2, _y2, _x3, _y3):
			return False, -1.0, -1.0

		s_x_0 = _x1 - _x0
		s_y_0 = _y1 - _y0
		s_x_1 = _x3 - _x2
		s_y_1 = _y3 - _y2

		determinant = s_x_0 * s_y_1 - s_x_1 * s_y_0
		threshold = 0.0001
		if abs(determinant) < threshold:
			return False, -1.0, -1.0

		s = (s_x_0 * (_y0 - _y2) - s_y_0 * (_x0 - _x2)) / determinant
		if s < 0.0 or s > 1.0:
			return False, -1.0, -1.0

		t = (s_x_1 * (_y0 - _y2) - s_y_1 * (_x0 - _x2)) / determinant
		if t < 0.0 or t > 1.0:
			return False, -1.0, -1.0

		return True, t, s

	def GetCurrentFloor(self, _current_pressure):
		current_floor = self.start_floor
		delta_height = (_current_pressure - (self.start_pressure + self.pressure_gain)) * self.delta_pressure_to_delta_height
		while delta_height > self.floor_height * 0.5:
			current_floor += 1
			delta_height -= self.floor_height
		while delta_height < -self.floor_height * 0.5:
			current_floor -= 1
			delta_height += self.floor_height

		if current_floor >= self.floor_number:
			current_floor = self.floor_number - 1
			print('Pressure error, start pressure: %.3f, current pressure: %.3f' % (self.start_pressure + self.pressure_gain, _current_pressure))
		if current_floor < 0:
			current_floor = 0
			print('Pressure error, start pressure: %.3f, current pressure: %.3f' % (self.start_pressure + self.pressure_gain, _current_pressure))

		return current_floor

	def UpdateCurrentFloor(self, _current_pressure):
		if not self.floor_changed:
			self.last_floor = self.current_floor
		self.current_floor = self.start_floor
		delta_height = (_current_pressure - (self.start_pressure + self.pressure_gain)) * self.delta_pressure_to_delta_height
		while delta_height > self.floor_height * 0.5:
			self.current_floor += 1
			delta_height -= self.floor_height
		while delta_height < -self.floor_height * 0.5:
			self.current_floor -= 1
			delta_height += self.floor_height

		if self.current_floor >= self.floor_number:
			self.current_floor = self.floor_number - 1
			print('Pressure error, start pressure: %.3f, current pressure: %.3f' % (self.start_pressure + self.pressure_gain, _current_pressure))
		if self.current_floor < 0:
			self.current_floor = 0
			print('Pressure error, start pressure: %.3f, current pressure: %.3f' % (self.start_pressure + self.pressure_gain, _current_pressure))

		if self.current_floor != self.last_floor:
			self.floor_changed = True

			temp_particles = self.particles
			self.particles = []
			for temp_particle in temp_particles:
				temp_x = int(temp_particle.x)
				temp_y = int(temp_particle.y)
				if (temp_x > -1) and (temp_x < self.map_width) and (temp_y > -1) and (temp_y < self.map_height) and (self.polygon_zone[self.current_floor][temp_y][temp_x] != 0):
					self.particles.append(temp_particle)

		return

	def UpdateCurrentAngle(self):
		particle_number = len(self.particles)
		if particle_number > 0:
			if particle_number > 1:
				base_angle = self.particles[0].rotate_angle
				self.current_position.z = 0.0
				for i in range(1, particle_number):
					temp_angle = self.particles[i].rotate_angle - base_angle
					while temp_angle > math.pi:
						temp_angle -= math.pi * 2.0
					while temp_angle < -math.pi:
						temp_angle += math.pi * 2.0
					self.current_position.z += temp_angle
				self.current_position.z /= particle_number
				self.current_position.z += base_angle
			else:
				self.current_position.z = self.particles[0].rotate_angle

		while self.current_position.z > math.pi:
			self.current_position.z -= math.pi * 2.0
		while self.current_position.z < -math.pi:
			self.current_position.z += math.pi * 2.0

		return

	def CheckNewParticle(self, _start, _end, _floor):
		for tLink in self.links[_floor]:
			tNode0 = tLink.node0
			tNode1 = tLink.node1
			temp_hit, temp_t, temp_s = self.Intersect(_start[0], _start[1], _end[0], _end[1], tNode0.x, tNode0.y, tNode1.x, tNode1.y)
			if temp_hit:
				return False

		return True

	def BackTrackingParticle(self, _particle, _step, _position_check_number=0, _position_error_max=2.0):
		last_x = _particle.x
		last_y = _particle.y
		scale = _particle.scale
		rotation = _particle.rotation

		temp_position_check_number = _position_check_number

		for tStep in _step:
			displacement = rotation.RotateVector(tStep.dx, tStep.dy, 0.0)
			current_x = last_x - displacement.x * scale
			current_y = last_y - displacement.y * scale
			for tLink in self.links[tStep.current_floor]:
				tNode0 = tLink.node0
				tNode1 = tLink.node1
				temp_hit, temp_t, temp_s = self.Intersect(last_x, last_y, current_x, current_y, tNode0.x, tNode0.y, tNode1.x, tNode1.y)
				if temp_hit:
					return False
			if temp_position_check_number > 0:
				temp_distance = math.sqrt((current_x - tStep.current_position.x) ** 2 + (current_y - tStep.current_position.y) ** 2)
				if temp_distance > _position_error_max:
					return False
				temp_position_check_number -= 1
			last_x = current_x
			last_y = current_y

		return True

	def CheckSteps(self, _position_start, _steps, _scale, _rotation):
		last_x = _position_start.x
		last_y = _position_start.y
		for tStep in _steps:
			displacement = _rotation.RotateVector(tStep.dx, tStep.dy, 0.0)
			current_x = last_x + displacement.x * _scale
			current_y = last_y + displacement.y * _scale
			for tLink in self.links[tStep.current_floor]:
				tNode0 = tLink.node0
				tNode1 = tLink.node1
				temp_hit, temp_t, temp_s = self.Intersect(last_x, last_y, current_x, current_y, tNode0.x, tNode0.y, tNode1.x, tNode1.y)
				if temp_hit:
					return False
			last_x = current_x
			last_y = current_y

		return True

	def CheckParticleSteps(self, _particle, _steps):
		for tStep in _steps:
			displacement = _particle.rotation.RotateVector(tStep.dx, tStep.dy, 0.0)
			current_x = _particle.x + displacement.x * _particle.scale
			current_y = _particle.y + displacement.y * _particle.scale
			for tLink in self.links[tStep.current_floor]:
				tNode0 = tLink.node0
				tNode1 = tLink.node1
				temp_hit, temp_t, temp_s = self.Intersect(_particle.x, _particle.y, current_x, current_y, tNode0.x, tNode0.y, tNode1.x, tNode1.y)
				if temp_hit:
					return False
			_particle.x = current_x
			_particle.y = current_y

		return True

	def WritePosition(self):
		log_file = open(self.position_filename, 'a')
		log_file.write(
			'\n%.3f,%.3f,%d' %
			(
				self.current_position.x,
				self.current_position.y,
				self.current_floor
			)
		)
		log_file.close()

		log_file = open(self.IPIN_filename, 'a')
		temp_longitude, temp_latitude, temp_floor = self.MapToWorld(self.current_position.x, self.current_position.y, self.current_floor)
		temp_index = 0
		if len(self.log_list) > 0:
			temp_index = self.log_list[0]
			self.log_list = self.log_list[1:]
			print('Logged index %i' % temp_index)
		log_file.write(
			'\n%i,%.9f,%.9f,%i,%i' %
			(
				self.time_start + int(timeit.default_timer() * 1000),
				temp_longitude,
				temp_latitude,
				temp_floor,
				temp_index
			)
		)
		log_file.close()

		return

	def ShowMap(self):
		image_show = self.map_images[self.current_floor].copy()
		if self.view_particle:
			for tParticle in self.particles:
				cv2.circle(
					image_show,
					(int(tParticle.x), int(tParticle.y)),
					1,
					(255, 0, 0),
					thickness=-1
				)
		cv2.circle(
			image_show,
			(int(self.current_position.x), int(self.current_position.y)),
			3,
			(0, 0, 255),
			thickness=-1
		)
		resized_image = cv2.resize(image_show, (self.resize_width, self.resize_height))
		cv2.imshow(self.window_name, resized_image)
		cv2.waitKey(5)
		self.video_writer.write(resized_image)

		return

	def TryRevive(self, _radius, _angle_range, _max_try_time, _back_tracking_step_number, _pressure, _position_check_number=0, _position_error_max=2.0):
		back_tracking_steps = []
		for i in range(1, _back_tracking_step_number + 1):
			if self.steps[-i].current_floor != self.current_floor:
				break
			back_tracking_steps.append(self.steps[-i])

		for i in range(_max_try_time):
			temp_angle = random.uniform(-math.pi, math.pi)
			temp_radius = random.uniform(0.0, _radius)
			new_x = self.current_position.x + math.cos(temp_angle) * temp_radius
			new_y = self.current_position.y + math.sin(temp_angle) * temp_radius
			temp_x = int(new_x)
			temp_y = int(new_y)
			if (temp_x > -1) and (temp_x < self.map_width) and (temp_y > -1) and (temp_y < self.map_height) and (self.polygon_zone[self.current_floor][temp_y][temp_x] != 0):
				new_particle = Particle(
					new_x,
					new_y,
					_pressure,
					self.current_floor,
					random.uniform(0.9, 1.1),
					self.current_position.z + random.uniform(-_angle_range, _angle_range)
				)
				if self.BackTrackingParticle(new_particle, back_tracking_steps, _position_check_number=_position_check_number, _position_error_max=_position_error_max):
					self.particles.append(new_particle)

		return

	def UpdatePressureGain(self, _dx, _dy, _z, _threshold):
		temp_rotation = Quaternion()
		temp_rotation.RotateZ(self.current_position.z)
		temp_displacement = temp_rotation.RotateVector(_dx, -_dy, 0.0)
		temp_x = int(self.current_position.x + temp_displacement.x * self.meter_to_pixel)
		temp_y = int(self.current_position.y + temp_displacement.y * self.meter_to_pixel)

		if (temp_x > -1) and (temp_x < self.map_width) and (temp_y > -1) and (temp_y < self.map_height) and (self.polygon_zone[self.current_floor][temp_y][temp_x] != 1):
			self.pressure_gain = _z - self.start_pressure - (self.current_floor - self.start_floor) * self.floor_height / self.delta_pressure_to_delta_height

		return

	def Update(self, _dx, _dy, _z, _check_floor=True):
		step_length = math.sqrt(_dx ** 2 + _dy ** 2)
		if (step_length < 0.1) or (step_length > 1.5):
			print('Dropped step, length: %.3fm' % step_length)
			return

		self.UpdatePressureGain(_dx, _dy, _z, 5.0 * self.meter_to_pixel)

		self.UpdateCurrentFloor(_z)

		temp_scale = self.meter_to_pixel
		temp_step = Step(_dx, -_dy, _z, self.current_floor, _scale=temp_scale)
		self.steps.append(temp_step)

		step_number = len(self.steps)

		temp_particles = self.particles
		self.particles = []
		x_sum = 0.0
		y_sum = 0.0
		for tParticle in temp_particles:
			if self.CheckParticleSteps(tParticle, [temp_step]):
				x_sum += tParticle.x
				y_sum += tParticle.y
				self.particles.append(tParticle)

		particle_number = len(self.particles)
		try_time_max = 8
		sample_number_max = 1
		back_tracking_step_number = min(32, step_number) + 1

		if particle_number > 0:
			self.current_position.x = x_sum / particle_number
			self.current_position.y = y_sum / particle_number
			self.UpdateCurrentAngle()
		else:
			temp_rotation = Quaternion()
			temp_rotation.RotateZ(self.current_position.z)
			temp_displacement = temp_rotation.RotateVector(temp_step.dx, temp_step.dy, 0.0)
			self.current_position.x += temp_displacement.x
			self.current_position.y += temp_displacement.y
		self.WritePosition()
		self.steps[-1].current_position = Vector3(_v=self.current_position)

		if particle_number > 0:
			sample_number = min(sample_number_max, particle_number // 2 + 1)
			generate_radius_min = 0.2 * self.meter_to_pixel
			generate_radius_max = 2.0 * self.meter_to_pixel
			temp_particles = []
			new_particle_number = self.particle_number_max - particle_number
			while new_particle_number > 0:
				slice_particles = random.sample(self.particles, sample_number)
				x_sum = 0.0
				y_sum = 0.0
				angle_sum = 0.0
				for tParticle in slice_particles:
					x_sum += tParticle.x
					y_sum += tParticle.y
					angle_sum += tParticle.rotate_angle
				x_sum /= sample_number
				y_sum /= sample_number
				angle_sum /= sample_number
				angle_sum += random.uniform(-0.1, 0.1)
				try_time = try_time_max
				while try_time > 0:
					try_time -= 1
					temp_angle = random.uniform(-math.pi, math.pi)
					temp_radius = random.uniform(generate_radius_min, generate_radius_max)
					new_x = x_sum + math.cos(temp_angle) * temp_radius
					new_y = y_sum + math.sin(temp_angle) * temp_radius
					new_particle = Particle(
						new_x,
						new_y,
						_z,
						self.current_floor,
						random.uniform(0.9, 1.1),
						angle_sum
					)
					if self.CheckNewParticle((x_sum, y_sum), (new_x, new_y), self.current_floor):
						back_tracking_steps = []
						for i in range(1, back_tracking_step_number):
							if self.steps[-i].current_floor != self.current_floor:
								break
							back_tracking_steps.append(self.steps[-i])
						if self.BackTrackingParticle(new_particle, back_tracking_steps, _position_check_number=0, _position_error_max=3.0):
							temp_particles.append(new_particle)
							break
				new_particle_number -= 1
			self.particles.extend(temp_particles)
		else:
			print('Try reviving...')
			self.TryRevive(10.0 * self.meter_to_pixel, 0.3, 256, min(16, step_number), _z, _position_check_number=0, _position_error_max=3.0)

		if _check_floor and self.floor_changed:
			step_number_min = 6
			step_number_test = 3
			back_step_number_max = 40
			# step_number_min >= step_number_test * 2
			delta_height_threshold = 0.3
			height_distance_threshold = 0.3 * self.floor_height
			height_distance_threshold_min = 0.25
			distance_threshold = 20.0 * self.meter_to_pixel
			if step_number > step_number_min:
				temp_floor_check = True
				for i in range(1, step_number_test + 1):
					if self.steps[-i].current_floor != self.current_floor:
						temp_floor_check = False
						break
				temp_delta_height = (self.steps[-1].pressure - self.steps[-1 - step_number_test].pressure) * self.delta_pressure_to_delta_height
				temp_height_distance = (self.steps[-1].pressure - (self.start_pressure + self.pressure_gain)) * self.delta_pressure_to_delta_height - (self.current_floor - self.start_floor) * self.floor_height
				if (((abs(temp_height_distance) < height_distance_threshold) and (abs(temp_delta_height) < delta_height_threshold)) or (abs(temp_height_distance) < height_distance_threshold_min)) and temp_floor_check:
					back_step_index = min(step_number - step_number_test, back_step_number_max + step_number_test) + 1
					temp_floor_flag = True
					for i in range(step_number_test + 1, back_step_index):
						if temp_floor_flag:
							if self.steps[-i].current_floor != self.current_floor:
								temp_floor_flag = False
						if not temp_floor_flag:
							if self.steps[-i].current_floor != self.last_floor:
								break
						temp_delta_height = (self.steps[-i].pressure - self.steps[-i - step_number_test].pressure) * self.delta_pressure_to_delta_height
						temp_height_distance = (self.steps[-i].pressure - (self.start_pressure + self.pressure_gain)) * self.delta_pressure_to_delta_height - (self.last_floor - self.start_floor) * self.floor_height
						if ((abs(temp_height_distance) < height_distance_threshold) and (abs(temp_delta_height) < delta_height_threshold)) or (abs(temp_height_distance) < height_distance_threshold_min):
							entrance_arrow = None
							entrance_distance = 0.0
							temp_type = 1
							if self.current_floor > self.last_floor:
								temp_type = 0
							for tArrow in self.arrows[self.current_floor]:
								if (tArrow.type == temp_type) or (tArrow.type == 2):
									temp_entrance_distance = math.sqrt((self.steps[-i].current_position.x - tArrow.node0.x) ** 2 + (self.steps[-i].current_position.y - tArrow.node0.y) ** 2)
									if (entrance_arrow is None) or (temp_entrance_distance < entrance_distance):
										entrance_arrow = tArrow
										entrance_distance = temp_entrance_distance
							if (entrance_arrow is not None) and (entrance_distance < distance_threshold):
								self.particles = []
								self.current_position.x = entrance_arrow.node1.x
								self.current_position.y = entrance_arrow.node1.y
								for j in range(5):
									for k in range(9):
										self.particles.append(
											Particle(
												entrance_arrow.node1.x,
												entrance_arrow.node1.y,
												self.steps[-1 - step_number_test].pressure,
												self.current_floor,
												1.0 + (k - 4) * 0.025,
												self.current_position.z + (j - 2) * 0.05
											)
										)
								temp_steps = self.steps[-step_number_test:]
								self.steps = self.steps[:-step_number_test]
								for tStep in temp_steps:
									self.Update(tStep.dx / self.meter_to_pixel, -tStep.dy / self.meter_to_pixel, tStep.pressure, _check_floor=False)
							break
					self.floor_changed = False
			else:
				self.floor_changed = False

		return
