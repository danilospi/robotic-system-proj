from geometry import *
import math

class point:
	def __init__(self, x, y, obstacle = -1, test=-1):
		self.x = x
		self.y = y
		self.obstacle = obstacle
		self.test = test

	def __str__(self):	
		return ( "x = " + str(self.x) + ", y = " + str(self.y) + ", obs = " + str(self.obstacle) + " and test:"+str(self.test) )

	def equals(self, other):
		if( self.x == other.x and self.y == other.y):
			return True
		else:
			return False

	def find_point(self, point_list):	
		for i in range(len(point_list)):
			if( self.x == point_list[i].x and self.y == point_list[i].y ):
				return i
		return -1

	def find_dist(self, pt2):
		return int(math.sqrt(pow((self.x - pt2.x),2) + pow(self.y-pt2.y, 2)))

	def find_closest_point(self, list_of_vertices):
		min_dist = 99999999
		min_index = -1
		for index, i in enumerate( list_of_vertices):
			dist = self.find_dist(i)
			if(dist<min_dist):
				min_dist = dist
				min_index = index
		return min_index