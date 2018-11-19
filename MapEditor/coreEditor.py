import math
from exceptions import Exception
import numpy as np
import cv2


class Node(object):
	def __init__(self, _id, _layer, _x, _y):
		self.id = _id
		self.layer = _layer
		self.x = _x
		self.y = _y

		self.links = []
		self.arrows = []
		self.polygons = []

		self.active = True

		return
	
	def SetActive(self, active):
		self.active = active
		if not active:
			for tLink in self.links:
				tLink.active = False
			for tArrow in self.arrows:
				tArrow.active = False
			for tPolygon in self.polygons:
				tPolygon.active = False
		
		return


class Link(object):
	def __init__(self, _id, _type, _node0, _node1):
		self.id = _id
		self.type = _type
		self.layer = _node0.layer
		self.node0 = _node0
		self.node1 = _node1

		self.active = True

		_node0.links.append(self)
		_node1.links.append(self)

		return


class Arrow(object):
	def __init__(self, _id, _type, _node0, _node1):
		self.id = _id
		# 0: down, 1: up, 2: both
		# down: green, up: orange, both: blue
		self.type = _type
		self.layer = _node0.layer
		self.node0 = _node0
		self.node1 = _node1
		
		self.active = True

		_node0.arrows.append(self)
		_node1.arrows.append(self)
		
		return


class Polygon(object):
	def __init__(self, _id, _type):
		self.id = _id

		# 0: dead zone, 1: z-free zone
		self.type = _type

		self.node_number = 0
		self.nodes = []

		self.active = True

		return

	def AddNode(self, _node):
		self.node_number += 1
		self.nodes.append(_node)
		_node.polygons.append(self)

		return


class MapEditor(object):
	def __init__(self, filename):
		self.filename = filename

		self.createByProgram = False
		self.floorNum = 1
		self.mapFilename = []
		self.mapImages = []
		self.floorHeight = 5.0
		self.startFloor = 0
		self.currentFloor = 0
		self.startX = 0.0
		self.startY = 0.0
		self.dirAngle = 0.0
		self.scale = 1.0
		self.nodeFilename = 'node.txt'
		self.linkFilename = 'link.txt'
		self.arrowFilename = 'arrow.txt'
		self.polygonFilename = 'polygon.txt'
		self.nodeID = 0
		self.linkID = 0
		self.arrowID = 0
		self.polygonID = 0

		fileReader = open(filename, 'r')
		tLine = fileReader.readline()
		tHeader = tLine.split('\n')[0]
		if tHeader == 'byProgram':
			self.createByProgram = True
		elif tHeader == 'byUser':
			self.createByProgram = False
		else:
			fileReader.close()
			raise Exception("bad file")
		for tLine in fileReader.readlines():
			tSeq = tLine.split(',')
			if tSeq[0] == 'floor':
				self.floorNum = int(tSeq[1])
				for i in range(self.floorNum):
					if i == self.floorNum - 1:
						tempImageFilename = tSeq[i + 2].split('\n')[0]
					else:
						tempImageFilename = tSeq[i + 2]
					tempImage = cv2.imread(tempImageFilename)
					self.mapFilename.append(tempImageFilename)
					self.mapImages.append(tempImage)
			elif tSeq[0] == 'height':
				self.floorHeight = float(tSeq[1].split('\n')[0])
			elif tSeq[0] == 'start':
				self.startFloor = int(tSeq[1].split('\n')[0])
				self.currentFloor = self.startFloor
			elif tSeq[0] == 'start_point':
				self.startX = float(tSeq[1])
				self.startY = float(tSeq[2].split('\n')[0])
			elif tSeq[0] == 'dir':
				self.dirAngle = float(tSeq[1].split('\n')[0])
			elif tSeq[0] == 'scale':
				self.scale = float(tSeq[1].split('\n')[0])
			elif tSeq[0] == 'node':
				self.nodeFilename = tSeq[1].split('\n')[0]
			elif tSeq[0] == 'link':
				self.linkFilename = tSeq[1].split('\n')[0]
			elif tSeq[0] == 'arrow':
				self.arrowFilename = tSeq[1].split('\n')[0]
			elif tSeq[0] == 'polygon':
				self.polygonFilename = tSeq[1].split('\n')[0]
			elif tSeq[0] == 'nodeID':
				self.nodeID = int(tSeq[1].split('\n')[0])
			elif tSeq[0] == 'linkID':
				self.linkID = int(tSeq[1].split('\n')[0])
			elif tSeq[0] == 'arrowID':
				self.arrowID = int(tSeq[1].split('\n')[0])
			elif tSeq[0] == 'polygonID':
				self.polygonID = int(tSeq[1].split('\n')[0])
		fileReader.close()

		self.currentMap = self.mapImages[self.currentFloor]
		self.minWidth = 1280
		self.minHeight = 720
		if self.currentMap.shape[1] < self.minWidth:
			raise Exception("Map width is too small.")
		if self.currentMap.shape[0] < self.minHeight:
			raise Exception("Map height is too small.")

		self.winWidth = 1600
		self.winHeight = (self.winWidth * self.minHeight) // self.minWidth
		self.minCropWidth = self.minWidth // 10
		self.minCropHeight = (self.minCropWidth * self.minHeight) // self.minWidth

		self.xTrans = 0
		self.yTrans = 0
		self.tWidth = self.minWidth
		self.tHeight = self.minHeight

		self.nodes = {}
		self.nodeColor = [
			(0, 0, 255)
		]
		self.links = {}
		self.linkColor = [
			(255, 0, 0)
		]
		self.arrows = {}
		self.arrowColor = [
			(87, 190, 48),
			(59, 127, 255),
			(180, 0, 0)
		]
		self.polygons = {}
		self.polygonColor = [
			(191, 191, 191),
			(155, 223, 255)
		]

		if self.createByProgram:
			fileReader = open(self.nodeFilename, 'r')
			fileReader.readline()
			for tLine in fileReader.readlines():
				tSeq = tLine.split(',')
				tID = int(tSeq[0])
				tLayer = int(tSeq[1])
				tX = float(tSeq[2])
				tY = float(tSeq[3])
				tNode = Node(tID, tLayer, tX, tY)
				self.nodes[tID] = tNode
			fileReader.close()

			fileReader = open(self.linkFilename, 'r')
			fileReader.readline()
			for tLine in fileReader.readlines():
				tSeq = tLine.split(',')
				tID = int(tSeq[0])
				tType = int(tSeq[1])
				tNodeID0 = int(tSeq[2])
				tNodeID1 = int(tSeq[3])
				tLink = Link(tID, tType, self.nodes[tNodeID0], self.nodes[tNodeID1])
				self.links[tID] = tLink
			fileReader.close()

			fileReader = open(self.arrowFilename, 'r')
			fileReader.readline()
			for tLine in fileReader.readlines():
				tSeq = tLine.split(',')
				tID = int(tSeq[0])
				tUp = int(tSeq[1])
				tNodeID0 = int(tSeq[2])
				tNodeID1 = int(tSeq[3])
				tArrow = Arrow(tID, tUp, self.nodes[tNodeID0], self.nodes[tNodeID1])
				self.arrows[tID] = tArrow
			fileReader.close()

			fileReader = open(self.polygonFilename, 'r')
			fileReader.readline()
			for tLine in fileReader.readlines():
				tSeq = tLine.split(',')
				tID = int(tSeq[0])
				tType = int(tSeq[1])
				tNodeNumber = int(tSeq[2])
				tPolygon = Polygon(tID, tType)
				for i in range(tNodeNumber):
					tNodeID = int(tSeq[i + 3])
					tPolygon.AddNode(self.nodes[tNodeID])
				self.polygons[tID] = tPolygon
			fileReader.close()

		self.mouseDown = False
		self.mouseX = 0
		self.mouseY = 0

		return

	def DistToLine(self, x0, y0, x1, y1, _x, _y):
		dist1 = math.sqrt((_x - x0) * (_x - x0) + (_y - y0) * (_y - y0))
		dist2 = math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0))
		dist3 = (_x - x0) * (x1 - x0) + (_y - y0) * (y1 - y0)
		dist3 /= dist2

		if dist3 < 0.0:
			return self.DistToPoint(x0, y0, _x, _y)
		if dist3 > dist2:
			return self.DistToPoint(x1, y1, _x, _y)

		return math.sqrt(max(0.01, dist1 * dist1 - dist3 * dist3))

	def DistToPoint(self, x0, y0, _x, _y):
		dist1 = math.sqrt((_x - x0) * (_x - x0) + (_y - y0) * (_y - y0))

		return dist1

	def DrawArrow(self, _img, _pt0, _pt1, _size, _color, _thickness, _offset):
		v0_x = _pt1[0] - _pt0[0]
		v0_y = _pt1[1] - _pt0[1]
		t = math.sqrt(v0_x ** 2 + v0_y ** 2) + 0.0001
		v0_x /= t
		v0_y /= t

		offset_x = v0_y * _offset
		offset_y = -v0_x * _offset

		cv2.line(
			_img,
			(int(_pt0[0] + offset_x), int(_pt0[1] + offset_y)),
			(int(_pt1[0] + offset_x), int(_pt1[1] + offset_y)),
			_color,
			thickness=_thickness
		)
		cv2.line(
			_img,
			(int(_pt1[0] + offset_x), int(_pt1[1] + offset_y)),
			(int(_pt1[0] - (v0_x - v0_y) * _size + offset_x), int(_pt1[1] - (v0_y + v0_x) * _size + offset_y)),
			_color,
			thickness=_thickness
		)
		cv2.line(
			_img,
			(int(_pt1[0] + offset_x), int(_pt1[1] + offset_y)),
			(int(_pt1[0] - (v0_x + v0_y) * _size + offset_x), int(_pt1[1] - (v0_y - v0_x) * _size + offset_y)),
			_color,
			thickness=_thickness
		)

		return

	def EditMap(self):
		CV_KEY_ESCAPE = 27

		CV_KEY_I = 73
		CV_KEY_i = 105
		CV_KEY_K = 75
		CV_KEY_k = 107
		CV_KEY_J = 74
		CV_KEY_j = 106
		CV_KEY_L = 76
		CV_KEY_l = 108

		CV_KEY_U = 85
		CV_KEY_u = 117
		CV_KEY_O = 79
		CV_KEY_o = 111

		CV_KEY_W = 87
		CV_KEY_w = 119
		CV_KEY_S = 83
		CV_KEY_s = 115
		CV_KEY_A = 65
		CV_KEY_a = 97
		CV_KEY_D = 68
		CV_KEY_d = 100

		CV_KEY_Q = 81
		CV_KEY_q = 113
		CV_KEY_E = 69
		CV_KEY_e = 101
		CV_KEY_T = 84
		CV_KEY_t = 116

		CV_KEY_TABLE = 9
		CV_KEY_BACKSPACE = 8
		CV_KEY_SPACE = 32
		CV_KEY_ZERO = 48
		CV_KEY_ONE = 49
		CV_KEY_TWO = 50
		CV_KEY_SIX = 54
		CV_KEY_SEVEN = 55

		selectingNode = -1
		selectingLink = -1
		selectedNode = -1
		selectedLink = -1
		selectingArrow = -1
		selectedArrow = -1
		selectingColor = (255.0, 255.0, 0.0)
		selectedColor = (0, 255, 255)
		selectRadius = 15.0

		edit_mode = 0

		polygon_creating = None

		winName = 'Map Editor'
		winShow = np.zeros((self.winHeight, self.winWidth, 3), dtype='uint8')
		cv2.imshow(winName, winShow)
		cv2.moveWindow(winName, 64, 64)
		cv2.setMouseCallback(winName, self.MouseHandler)

		while True:
			ROI = self.currentMap[self.yTrans:(self.yTrans + self.tHeight), self.xTrans:(self.xTrans + self.tWidth)]
			cv2.resize(ROI, (self.winWidth, self.winHeight), winShow)

			for key, value in self.polygons.items():
				if value.active and value.nodes[0].layer == self.currentFloor and value.type == 1:
					temp_points = []
					for i in range(value.node_number):
						temp_points.append([int((value.nodes[i].x - self.xTrans) * self.winWidth / self.tWidth), int((value.nodes[i].y - self.yTrans) * self.winWidth / self.tWidth)])
					temp_points_array = np.array([temp_points], dtype=np.int32)
					cv2.fillPoly(winShow, temp_points_array, self.polygonColor[value.type])

			for key, value in self.polygons.items():
				if value.active and value.nodes[0].layer == self.currentFloor and value.type != 1:
					temp_points = []
					for i in range(value.node_number):
						temp_points.append([int((value.nodes[i].x - self.xTrans) * self.winWidth / self.tWidth), int((value.nodes[i].y - self.yTrans) * self.winWidth / self.tWidth)])
					temp_points_array = np.array([temp_points], dtype=np.int32)
					cv2.fillPoly(winShow, temp_points_array, self.polygonColor[value.type])

			for key, value in self.arrows.items():
				if value.active and value.layer == self.currentFloor:
					self.DrawArrow(
						winShow,
						(int((value.node0.x - self.xTrans) * self.winWidth / self.tWidth), int((value.node0.y - self.yTrans) * self.winWidth / self.tWidth)),
						(int((value.node1.x - self.xTrans) * self.winWidth / self.tWidth), int((value.node1.y - self.yTrans) * self.winWidth / self.tWidth)),
						15,
						self.arrowColor[value.type],
						5,
						0.0
					)

			if selectedArrow > -1:
				value = self.arrows[selectedArrow]
				cv2.line(
					winShow,
					(int((value.node0.x - self.xTrans) * self.winWidth / self.tWidth), int((value.node0.y - self.yTrans) * self.winWidth / self.tWidth)),
					(int((value.node1.x - self.xTrans) * self.winWidth / self.tWidth), int((value.node1.y - self.yTrans) * self.winWidth / self.tWidth)),
					selectedColor,
					thickness=2
					)

			if selectingArrow > -1:
				value = self.arrows[selectingArrow]
				cv2.line(
					winShow,
					(int((value.node0.x - self.xTrans) * self.winWidth / self.tWidth), int((value.node0.y - self.yTrans) * self.winWidth / self.tWidth)),
					(int((value.node1.x - self.xTrans) * self.winWidth / self.tWidth), int((value.node1.y - self.yTrans) * self.winWidth / self.tWidth)),
					selectingColor,
					thickness=2
					)

			for key, value in self.links.items():
				if value.active and value.layer == self.currentFloor:
					cv2.line(
						winShow,
						(int((value.node0.x - self.xTrans) * self.winWidth / self.tWidth), int((value.node0.y - self.yTrans) * self.winWidth / self.tWidth)),
						(int((value.node1.x - self.xTrans) * self.winWidth / self.tWidth), int((value.node1.y - self.yTrans) * self.winWidth / self.tWidth)),
						self.linkColor[value.type],
						thickness=7
						)

			if selectedLink > -1:
				value = self.links[selectedLink]
				cv2.line(
					winShow,
					(int((value.node0.x - self.xTrans) * self.winWidth / self.tWidth), int((value.node0.y - self.yTrans) * self.winWidth / self.tWidth)),
					(int((value.node1.x - self.xTrans) * self.winWidth / self.tWidth), int((value.node1.y - self.yTrans) * self.winWidth / self.tWidth)),
					selectedColor,
					thickness=2
					)

			if selectingLink > -1:
				value = self.links[selectingLink]
				cv2.line(
					winShow,
					(int((value.node0.x - self.xTrans) * self.winWidth / self.tWidth), int((value.node0.y - self.yTrans) * self.winWidth / self.tWidth)),
					(int((value.node1.x - self.xTrans) * self.winWidth / self.tWidth), int((value.node1.y - self.yTrans) * self.winWidth / self.tWidth)),
					selectingColor,
					thickness=2
					)

			for key, value in self.nodes.items():
				if value.active and value.layer == self.currentFloor:
					cv2.circle(
						winShow,
						(int((value.x - self.xTrans) * self.winWidth / self.tWidth), int((value.y - self.yTrans) * self.winWidth / self.tWidth)),
						5,
						self.nodeColor[0],
						thickness=-1
						)

			if selectedNode > -1:
				value = self.nodes[selectedNode]
				cv2.circle(
					winShow,
					(int((value.x - self.xTrans) * self.winWidth / self.tWidth), int((value.y - self.yTrans) * self.winWidth / self.tWidth)),
					5,
					selectedColor,
					thickness=2
					)
				if edit_mode == 0:
					cv2.line(
						winShow,
						(int((value.x - self.xTrans) * self.winWidth / self.tWidth), int((value.y - self.yTrans) * self.winWidth / self.tWidth)),
						(self.mouseX, self.mouseY),
						self.linkColor[0],
						thickness=1
					)
				elif edit_mode == 1:
					self.DrawArrow(
						winShow,
						(int((value.x - self.xTrans) * self.winWidth / self.tWidth), int((value.y - self.yTrans) * self.winWidth / self.tWidth)),
						(self.mouseX, self.mouseY),
						15,
						self.arrowColor[1],
						1,
						0.0
					)

			if polygon_creating is not None:
				last_x = self.mouseX
				last_y = self.mouseY
				for tNode in polygon_creating.nodes:
					current_x = int((tNode.x - self.xTrans) * self.winWidth / self.tWidth)
					current_y = int((tNode.y - self.yTrans) * self.winWidth / self.tWidth)
					cv2.line(
						winShow,
						(last_x, last_y),
						(current_x, current_y),
						self.polygonColor[polygon_creating.type],
						thickness=1
					)
					last_x = current_x
					last_y = current_y
				cv2.line(
					winShow,
					(last_x, last_y),
					(self.mouseX, self.mouseY),
					self.polygonColor[polygon_creating.type],
					thickness=1
				)

			if selectingNode > -1:
				value = self.nodes[selectingNode]
				cv2.circle(
					winShow,
					(int((value.x - self.xTrans) * self.winWidth / self.tWidth), int((value.y - self.yTrans) * self.winWidth / self.tWidth)),
					5,
					selectingColor,
					thickness=2
					)

			if self.currentFloor == self.startFloor:
				cv2.circle(
					winShow,
					(int((self.startX - self.xTrans) * self.winWidth / self.tWidth), int((self.startY - self.yTrans) * self.winWidth / self.tWidth)),
					7,
					(128.0, 0.0, 255.0),
					thickness=-1
					)

			cv2.imshow(winName, winShow)
			rKey = cv2.waitKey(50)

			selectingNode = -1
			selectingLink = -1
			selectingArrow = -1

			if rKey == CV_KEY_ZERO:
				if cv2.waitKey(200) == CV_KEY_ONE:
					cv2.destroyAllWindows()
					self.SaveMap()
					break

			if rKey == CV_KEY_ONE:
				if cv2.waitKey(200) == CV_KEY_ZERO:
					cv2.destroyAllWindows()
					print 'ALL CHANGE WILL NOT BE SAVED'
					break

			if rKey == CV_KEY_SIX:
				if self.currentFloor > 0:
					self.currentFloor -= 1
					self.currentMap = self.mapImages[self.currentFloor]
					print 'current floor: ', self.currentFloor

					selectedNode = -1
					selectedLink = -1
					selectedArrow = -1

					if (polygon_creating is not None) and (polygon_creating.node_number > 2):
						self.polygons[self.polygonID] = polygon_creating
						self.polygonID += 1
					polygon_creating = None

			if rKey == CV_KEY_SEVEN:
				if self.currentFloor < self.floorNum - 1:
					self.currentFloor += 1
					self.currentMap = self.mapImages[self.currentFloor]
					print 'current floor: ', self.currentFloor

					selectedNode = -1
					selectedLink = -1
					selectedArrow = -1

					if (polygon_creating is not None) and (polygon_creating.node_number > 2):
						self.polygons[self.polygonID] = polygon_creating
						self.polygonID += 1
					polygon_creating = None

			if rKey == CV_KEY_ESCAPE:
				selectedNode = -1
				selectedLink = -1
				selectedArrow = -1

				if (polygon_creating is not None) and (polygon_creating.node_number > 2):
					self.polygons[self.polygonID] = polygon_creating
					self.polygonID += 1
				polygon_creating = None

			if rKey == CV_KEY_SPACE:
				edit_mode += 1

				if edit_mode == 1:
					print 'Creating arrows'

				if edit_mode == 2:
					print 'Creating polygons'

					selectedNode = -1
					selectedLink = -1
					selectedArrow = -1

					if (polygon_creating is not None) and (polygon_creating.node_number > 2):
						self.polygons[self.polygonID] = polygon_creating
						self.polygonID += 1
					polygon_creating = None

				if edit_mode > 2:
					if (polygon_creating is not None) and (polygon_creating.node_number > 2):
						self.polygons[self.polygonID] = polygon_creating
						self.polygonID += 1
					polygon_creating = None

					edit_mode = 0

					print 'Creating links'

			if rKey == CV_KEY_I or rKey == CV_KEY_i:
				self.yTrans = int(max(0, self.yTrans - self.tHeight * 0.1))
			if rKey == CV_KEY_K or rKey == CV_KEY_k:
				self.yTrans = int(min(self.currentMap.shape[0] - self.tHeight, self.yTrans + self.tHeight * 0.1))
			if rKey == CV_KEY_J or rKey == CV_KEY_j:
				self.xTrans = int(max(0, self.xTrans - self.tWidth * 0.1))
			if rKey == CV_KEY_L or rKey == CV_KEY_l:
				self.xTrans = int(min(self.currentMap.shape[1] - self.tWidth, self.xTrans + self.tWidth * 0.1))
			if rKey == CV_KEY_U or rKey == CV_KEY_u:
				self.tWidth = int(max(self.minCropWidth, self.tWidth * 0.9))
				self.tHeight = int(max(self.minCropHeight, self.tHeight * 0.9))
			if rKey == CV_KEY_O or rKey == CV_KEY_o:
				if (self.currentMap.shape[0] - self.yTrans) * self.winWidth > (self.currentMap.shape[1] - self.xTrans) * self.winHeight:
					self.tWidth = int(min(self.currentMap.shape[1] - self.xTrans, self.tWidth * 1.1))
					self.tHeight = int(self.tWidth * self.winHeight / self.winWidth)
				else:
					self.tHeight = int(min(self.currentMap.shape[0] - self.yTrans, self.tHeight * 1.1))
					self.tWidth = int(self.tHeight * self.winWidth / self.winHeight)

			if (rKey == CV_KEY_Q or rKey == CV_KEY_q) and selectedNode > -1:
				rKey = cv2.waitKey(200)
				if rKey == CV_KEY_E or rKey == CV_KEY_e:
					tNode = self.nodes[selectedNode]
					self.startFloor = self.currentFloor
					self.startX = tNode.x
					self.startY = tNode.y
					tNode.SetActive(False)
					selectedNode = -1

			tempScale = self.winWidth / float(self.tWidth)

			for key, value in self.nodes.items():
				if value.active and (value.layer == self.currentFloor) and self.DistToPoint(self.mouseX, self.mouseY, (value.x - self.xTrans) * tempScale, (value.y - self.yTrans) * tempScale) < selectRadius:
					selectingNode = key
					break

			if selectingNode < 0:
				for key, value in self.links.items():
					if value.active and value.layer == self.currentFloor and self.DistToLine((value.node0.x - self.xTrans) * tempScale, (value.node0.y - self.yTrans) * tempScale, (value.node1.x - self.xTrans) * tempScale, (value.node1.y - self.yTrans) * tempScale, self.mouseX, self.mouseY) < selectRadius:
						selectingLink = key
						break
				if selectingLink < 0:
					for key, value in self.arrows.items():
						if value.active and value.layer == self.currentFloor and self.DistToLine((value.node0.x - self.xTrans) * tempScale, (value.node0.y - self.yTrans) * tempScale, (value.node1.x - self.xTrans) * tempScale, (value.node1.y - self.yTrans) * tempScale, self.mouseX, self.mouseY) < selectRadius:
							selectingArrow = key
							break

			if self.mouseDown:
				self.mouseDown = False
				if selectedNode > -1:
					if selectingNode > -1:
						if selectingNode != selectedNode:
							if edit_mode == 0:
								tLink = Link(self.linkID, 0, self.nodes[selectedNode], self.nodes[selectingNode])
								self.links[self.linkID] = tLink
								self.linkID += 1
								selectedNode = selectingNode
								print 'current node id: ', selectedNode
							elif edit_mode == 1:
								tArrow = Arrow(self.arrowID, 1, self.nodes[selectedNode], self.nodes[selectingNode])
								self.arrows[self.arrowID] = tArrow
								self.arrowID += 1
								selectedNode = selectingNode
								print 'current node id: ', selectedNode
					elif selectingLink > -1:
						selectedNode = -1
						selectedLink = selectingLink
						tLink = self.links[selectedLink]
						tLength = self.DistToPoint(tLink.node0.x, tLink.node0.y, tLink.node1.x, tLink.node1.y) * self.scale
						tAngle = math.atan2(tLink.node0.y - tLink.node1.y, tLink.node1.x - tLink.node0.x)
						print 'current link id: ' + str(selectedLink) + ', length: ' + str(tLength) + 'm, angle: ' + str(tAngle) + 'rad'
					elif selectingArrow > -1:
						selectedNode = -1
						selectedArrow = selectingArrow
						tArrow = self.arrows[selectedArrow]
						tLength = self.DistToPoint(tArrow.node0.x, tArrow.node0.y, tArrow.node1.x, tArrow.node1.y) * self.scale
						tAngle = math.atan2(tArrow.node0.y - tArrow.node1.y, tArrow.node1.x - tArrow.node0.x)
						print 'current arrow id: ' + str(selectedArrow) + ', length: ' + str(tLength) + 'm, angle: ' + str(tAngle) + 'rad'
					else:
						tX = self.mouseX * self.tWidth / float(self.winWidth) + self.xTrans
						tY = self.mouseY * self.tHeight / float(self.winHeight) + self.yTrans
						tNode = Node(self.nodeID, self.currentFloor, tX, tY)
						self.nodes[self.nodeID] = tNode

						if edit_mode == 0:
							tLink = Link(self.linkID, 0, self.nodes[selectedNode], tNode)
							self.links[self.linkID] = tLink
							self.linkID += 1
						elif edit_mode == 1:
							tArrow = Arrow(self.arrowID, 1, self.nodes[selectedNode], tNode)
							self.arrows[self.arrowID] = tArrow
							self.arrowID += 1

						selectedNode = self.nodeID
						print 'current node id: ', selectedNode
						selectingNode = selectedNode
						self.nodeID += 1
				elif selectedLink > -1:
					if selectingNode > -1:
						selectedLink = -1
						selectedNode = selectingNode
						print 'current node id: ', selectedNode
					elif selectingLink > -1:
						selectedLink = selectingLink
						tLink = self.links[selectedLink]
						tLength = self.DistToPoint(tLink.node0.x, tLink.node0.y, tLink.node1.x, tLink.node1.y) * self.scale
						tAngle = math.atan2(tLink.node0.y - tLink.node1.y, tLink.node1.x - tLink.node0.x)
						print 'current link id: ' + str(selectedLink) + ', length: ' + str(tLength) + 'm, angle: ' + str(tAngle) + 'rad'
					elif selectingArrow > -1:
						selectedLink = -1
						selectedArrow = selectingArrow
						tArrow = self.arrows[selectedArrow]
						tLength = self.DistToPoint(tArrow.node0.x, tArrow.node0.y, tArrow.node1.x, tArrow.node1.y) * self.scale
						tAngle = math.atan2(tArrow.node0.y - tArrow.node1.y, tArrow.node1.x - tArrow.node0.x)
						print 'current arrow id: ' + str(selectedArrow) + ', length: ' + str(tLength) + 'm, angle: ' + str(tAngle) + 'rad'
					else:
						selectedLink = -1
				elif selectedArrow > -1:
					if selectingNode > -1:
						selectedArrow = -1
						selectedNode = selectingNode
						print 'current node id: ', selectedNode
					elif selectingLink > -1:
						selectedArrow = -1
						selectedLink = selectingLink
						tLink = self.links[selectedLink]
						tLength = self.DistToPoint(tLink.node0.x, tLink.node0.y, tLink.node1.x, tLink.node1.y) * self.scale
						tAngle = math.atan2(tLink.node0.y - tLink.node1.y, tLink.node1.x - tLink.node0.x)
						print 'current link id: ' + str(selectedLink) + ', length: ' + str(tLength) + 'm, angle: ' + str(tAngle) + 'rad'
					elif selectingArrow > -1:
						selectedArrow = selectingArrow
						tArrow = self.arrows[selectedArrow]
						tLength = self.DistToPoint(tArrow.node0.x, tArrow.node0.y, tArrow.node1.x, tArrow.node1.y) * self.scale
						tAngle = math.atan2(tArrow.node0.y - tArrow.node1.y, tArrow.node1.x - tArrow.node0.x)
						print 'current arrow id: ' + str(selectedArrow) + ', length: ' + str(tLength) + 'm, angle: ' + str(tAngle) + 'rad'
					else:
						selectedArrow = -1
				elif edit_mode == 2:
					if selectingNode > -1:
						if polygon_creating is None:
							polygon_creating = Polygon(self.polygonID, 0)
							polygon_creating.AddNode(self.nodes[selectingNode])
						elif polygon_creating.nodes[0].id == selectingNode:
							if polygon_creating.node_number > 2:
								self.polygons[self.polygonID] = polygon_creating
								self.polygonID += 1
								polygon_creating = None
						else:
							polygon_creating.AddNode(self.nodes[selectingNode])
					else:
						tX = self.mouseX * self.tWidth / float(self.winWidth) + self.xTrans
						tY = self.mouseY * self.tHeight / float(self.winHeight) + self.yTrans
						tNode = Node(self.nodeID, self.currentFloor, tX, tY)
						self.nodes[self.nodeID] = tNode
						selectingNode = self.nodeID
						self.nodeID += 1
						if polygon_creating is None:
							polygon_creating = Polygon(self.polygonID, 0)
						polygon_creating.AddNode(tNode)
				else:
					if selectingNode > -1:
						selectedNode = selectingNode
						print 'current node id: ' + str(selectedNode) + ', x: ' + str(self.nodes[selectedNode].x) + ', y: ' + str(self.nodes[selectedNode].y)
					elif selectingLink > -1:
						selectedLink = selectingLink
						tLink = self.links[selectedLink]
						tLength = self.DistToPoint(tLink.node0.x, tLink.node0.y, tLink.node1.x, tLink.node1.y) * self.scale
						tAngle = math.atan2(tLink.node0.y - tLink.node1.y, tLink.node1.x - tLink.node0.x)
						print 'current link id: ' + str(selectedLink) + ', length: ' + str(tLength) + 'm, angle: ' + str(tAngle) + 'rad'
					elif selectingArrow > -1:
						selectedArrow = selectingArrow
						tArrow = self.arrows[selectedArrow]
						tLength = self.DistToPoint(tArrow.node0.x, tArrow.node0.y, tArrow.node1.x, tArrow.node1.y) * self.scale
						tAngle = math.atan2(tArrow.node0.y - tArrow.node1.y, tArrow.node1.x - tArrow.node0.x)
						print 'current arrow id: ' + str(selectedArrow) + ', length: ' + str(tLength) + 'm, angle: ' + str(tAngle) + 'rad'
					else:
						tX = self.mouseX * self.tWidth / float(self.winWidth) + self.xTrans
						tY = self.mouseY * self.tHeight / float(self.winHeight) + self.yTrans
						tNode = Node(self.nodeID, self.currentFloor, tX, tY)
						self.nodes[self.nodeID] = tNode
						selectedNode = self.nodeID
						print 'current node id: ', selectedNode
						selectingNode = selectedNode
						self.nodeID += 1

			if selectedNode > -1:
				value = self.nodes[selectedNode]
				if rKey == CV_KEY_W:
					value.y -= 0.1 / self.scale
				if rKey == CV_KEY_w:
					value.y -= 0.01 / self.scale
				if rKey == CV_KEY_S:
					value.y += 0.1 / self.scale
				if rKey == CV_KEY_s:
					value.y += 0.01 / self.scale
				if rKey == CV_KEY_A:
					value.x -= 0.1 / self.scale
				if rKey == CV_KEY_a:
					value.x -= 0.01 / self.scale
				if rKey == CV_KEY_D:
					value.x += 0.1 / self.scale
				if rKey == CV_KEY_d:
					value.x += 0.01 / self.scale
				if rKey == CV_KEY_BACKSPACE:
					value.SetActive(False)
					selectedNode = -1

			if selectedLink > -1 and rKey == CV_KEY_BACKSPACE:
				value = self.links[selectedLink]
				value.active = False
				selectedLink = -1

			if selectedArrow > -1:
				value = self.arrows[selectedArrow]
				if rKey == CV_KEY_TABLE:
					value.type += 1
					if value.type > 2:
						value.type = 0
				if rKey == CV_KEY_BACKSPACE:
					value.active = False
					selectedArrow = -1

			if polygon_creating is not None:
				if rKey == CV_KEY_TABLE:
					polygon_creating.type += 1
					if polygon_creating.type > 1:
						polygon_creating.type = 0

		return

	def MouseHandler(self, _event, _x, _y, _flags, _param):
		self.mouseX = _x
		self.mouseY = _y

		if _event == cv2.EVENT_LBUTTONDOWN:
			self.mouseDown = True
		if _event == cv2.EVENT_LBUTTONUP:
			self.mouseDown = False

		return

	def SaveMap(self):
		print 'saving...'

		fWriter = open('core.txt', 'w')
		fWriter.write('byProgram')
		fWriter.write('\nfloor,' + str(self.floorNum))
		for i in range(self.floorNum):
			fWriter.write(',' + self.mapFilename[i])
		fWriter.write('\nheight,' + str(self.floorHeight))
		fWriter.write('\nstart,' + str(self.startFloor))
		fWriter.write('\nstart_point,' + str(self.startX) + ',' + str(self.startY))
		fWriter.write('\ndir,' + str(self.dirAngle))
		fWriter.write('\nscale,' + str(self.scale))
		fWriter.write('\nnode,node.txt')
		fWriter.write('\nlink,link.txt')
		fWriter.write('\narrow,arrow.txt')
		fWriter.write('\npolygon,polygon.txt')
		fWriter.write('\nnodeID,' + str(self.nodeID))
		fWriter.write('\nlinkID,' + str(self.linkID))
		fWriter.write('\narrowID,' + str(self.arrowID))
		fWriter.write('\npolygonID,' + str(self.polygonID))
		fWriter.close()

		fWriter = open('node.txt', 'w')
		fWriter.write('id,layer,x,y')
		for key, value in self.nodes.items():
			if value.active:
				fWriter.write('\n' + str(value.id) + ',' + str(value.layer) + ',' + str(value.x) + ',' + str(value.y))
		fWriter.close()

		fWriter = open('link.txt', 'w')
		fWriter.write('id,type,nodeID0,nodeID1')
		for key, value in self.links.items():
			if value.active:
				fWriter.write('\n' + str(value.id) + ',' + str(value.type) + ',' + str(value.node0.id) + ',' + str(value.node1.id))
		fWriter.close()

		fWriter = open('arrow.txt', 'w')
		fWriter.write('id,type,nodeID0,nodeID1')
		for key, value in self.arrows.items():
			if value.active:
				fWriter.write('\n' + str(value.id) + ',' + str(value.type) + ',' + str(value.node0.id) + ',' + str(value.node1.id))
		fWriter.close()

		fWriter = open('polygon.txt', 'w')
		fWriter.write('id,type,nodeNumber,nodeIDs')
		for key, value in self.polygons.items():
			if value.active:
				fWriter.write('\n' + str(value.id) + ',' + str(value.type) + ',' + str(value.node_number))
				for i in range(value.node_number):
					fWriter.write(',' + str(value.nodes[i].id))
		fWriter.close()

		print 'done'
		return
