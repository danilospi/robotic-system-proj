from point import *
import math

def normalize_angle(a):
    while a > math.pi:
        a = a - 2 * math.pi
    while a < - math.pi:
        a = a + 2 * math.pi
    return a

def find_dist(pt1, pt2):
	return int(math.sqrt(pow((pt1.x - pt2.x),2) + pow(pt1.y-pt2.y, 2)))

def counter_clockwise(A,B,C):
    return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)

def segment_intersection(a, b, c, d):
	if( intersect(a, b, c, d) == True):
		return line_intersection([a, b], [c, d])
	else:
		return -1

def intersect(A,B,C,D):
	# Check if any three points are co-linear
    if(((A.x * (B.y - C.y)) + (B.x * (C.y - A.y)) + (C.x * (A.y - B.y)))== 0):
        return True
    if(((A.x * (B.y - D.y)) + (B.x * (D.y - A.y)) + (D.x * (A.y - B.y)))== 0):
        return True
    if(((A.x * (C.y - D.y)) + (C.x * (D.y - A.y)) + (D.x * (A.y - C.y)))== 0):
        return True
    if(((B.x * (C.y - D.y)) + (C.x * (D.y - B.y)) + (D.x * (B.y - C.y)))== 0):
        return True    
    return counter_clockwise(A,C,D) != counter_clockwise(B,C,D) and counter_clockwise(A,B,C) != counter_clockwise(A,B,D)

def line_intersection(segment1, segment2):
	line1 = [segment1[0].x, segment1[0].y], [segment1[1].x, segment1[1].y]
	line2 = [segment2[0].x, segment2[0].y], [segment2[1].x, segment2[1].y]
	
	x_diff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
	y_diff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])
	
	def determinant(a, b):
		return ( (a[0] * b[1]) - (a[1] * b[0]) )

	div = determinant(x_diff, y_diff)
	if div == 0:
		#parallel lines
		return -1

	d = ( determinant(line1[0] , line1[1]) , determinant(line2[0], line2[1]))
	x = determinant(d, x_diff) / float(div)
	y = determinant(d, y_diff) / float(div)
	x = int(x + 0.5)
	y = int(y + 0.5)
	return point(x, y)

def centroid(vertices):
	n = len(vertices)
	if( n==0 ):
		return -1;	
	sum_x = 0
	sum_y = 0
	for i in vertices:
		sum_x = sum_x + i.x
		sum_y = sum_y + i.y
	centr_x = int(0.5 + sum_x/float(n))
	centr_y = int(0.5 + sum_y/float(n))

	return point(centr_x, centr_y)

def polygon_area(vertices, number_of_vertices):
	# Expects closed polygon
	n = number_of_vertices
	if(n %2 !=0 ):
		vertices.append(vertices[0])
	area = 0
	for i in range(0, n, 2):
		area += vertices[i+1].x*(vertices[i+2].y-vertices[i].y) + vertices[i+1].y*(vertices[i].x-vertices[i+2].x)
	return int(area/2)

def check_obstruction(obstacles, segment):
	res = True; break_out = False;
	for obs in obstacles:
		# Add the last line to make closed polygon
		n = len(obs)-1
		if ( obs[n].equals(obs[0]) is False):
			obs.append(obs[0])
		for index in range(len(obs)-1):
			if (segment_intersection( segment[0], segment[1],  obs[index],  obs[index+1]) != -1):
				res = False
				break_out = True
				break;	
		if(break_out is True):
			break
	return res