import math


class Vector3(object):
	def __init__(self, _v=None):
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0

		if _v is not None:
			self.x = _v.x
			self.y = _v.y
			self.z = _v.z

		return

	def Reset(self):
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0

		return

	def Set(self, _x, _y, _z):
		self.x = _x
		self.y = _y
		self.z = _z

		return

	def Mul(self, _k):
		self.x *= _k
		self.y *= _k
		self.z *= _k

		return


class Quaternion(object):
	def __init__(self):
		self.w = 1.0
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0

		return

	def Reset(self):
		self.w = 1.0
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0

		return

	def Set(self, _w, _x, _y, _z):
		self.w = _w
		self.x = _x
		self.y = _y
		self.z = _z

		return

	def Conjugate(self):
		ret = Quaternion()
		ret.Set(self.w, -self.x, -self.y, -self.z)

		return ret

	def Rotate(self, _w, _x, _y, _z):
		tw = self.w
		tx = self.x
		ty = self.y
		tz = self.z

		self.w = _w * tw - _x * tx - _y * ty - _z * tz
		self.x = _w * tx + _x * tw + _y * tz - _z * ty
		self.y = _w * ty - _x * tz + _y * tw + _z * tx
		self.z = _w * tz + _x * ty - _y * tx + _z * tw

		return

	def RotateZ(self, _angle):
		_w = math.cos(_angle / 2.0)
		_x = 0.0
		_y = 0.0
		_z = -math.sin(_angle / 2.0)

		tw = self.w
		tx = self.x
		ty = self.y
		tz = self.z

		self.w = _w * tw - _x * tx - _y * ty - _z * tz
		self.x = _w * tx + _x * tw + _y * tz - _z * ty
		self.y = _w * ty - _x * tz + _y * tw + _z * tx
		self.z = _w * tz + _x * ty - _y * tx + _z * tw

		return

	def RotateVector(self, _x, _y, _z):
		ret = Vector3()

		w22 = 2.0 * self.w * self.w
		x22 = 2.0 * self.x * self.x
		y22 = 2.0 * self.y * self.y
		z22 = 2.0 * self.z * self.z
		xy2 = 2.0 * self.x * self.y
		zw2 = 2.0 * self.z * self.w
		xz2 = 2.0 * self.x * self.z
		yw2 = 2.0 * self.y * self.w
		yz2 = 2.0 * self.y * self.z
		xw2 = 2.0 * self.x * self.w

		ret.x = _x * (x22 + w22 - 1.0) + _y * (xy2 - zw2) + _z * (xz2 + yw2)
		ret.y = _x * (xy2 + zw2) + _y * (y22 + w22 - 1.0) + _z * (yz2 - xw2)
		ret.z = _x * (xz2 - yw2) + _y * (yz2 + xw2) + _z * (z22 + w22 - 1.0)

		return ret

	def Update(self, gx, gy, gz, ax, ay, az, delta_time, beta, epsilon):
		qDot0 = 0.5 * (-self.x * gx - self.y * gy - self.z * gz)
		qDot1 = 0.5 * (self.w * gx + self.y * gz - self.z * gy)
		qDot2 = 0.5 * (self.w * gy - self.x * gz + self.z * gx)
		qDot3 = 0.5 * (self.w * gz + self.x * gy - self.y * gx)

		tMag = math.sqrt(ax * ax + ay * ay + az * az)
		if (beta != 0.0) and (tMag > epsilon):
			ax /= tMag
			ay /= tMag
			az /= tMag

			_2q0 = 2.0 * self.w
			_2q1 = 2.0 * self.x
			_2q2 = 2.0 * self.y
			_2q3 = 2.0 * self.z
			_4q0 = 4.0 * self.w
			_4q1 = 4.0 * self.x
			_4q2 = 4.0 * self.y
			_8q1 = 8.0 * self.x
			_8q2 = 8.0 * self.y
			q0q0 = self.w * self.w
			q1q1 = self.x * self.x
			q2q2 = self.y * self.y
			q3q3 = self.z * self.z

			s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
			s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * self.x - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az
			s2 = 4.0 * q0q0 * self.y + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az
			s3 = 4.0 * q1q1 * self.z - _2q1 * ax + 4.0 * q2q2 * self.z - _2q2 * ay

			tMag = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
			s0 /= tMag
			s1 /= tMag
			s2 /= tMag
			s3 /= tMag

			qDot0 -= beta * s0
			qDot1 -= beta * s1
			qDot2 -= beta * s2
			qDot3 -= beta * s3

		self.w += qDot0 * delta_time
		self.x += qDot1 * delta_time
		self.y += qDot2 * delta_time
		self.z += qDot3 * delta_time

		tMag = math.sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
		self.w /= tMag
		self.x /= tMag
		self.y /= tMag
		self.z /= tMag

		return
