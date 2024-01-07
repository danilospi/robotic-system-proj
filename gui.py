
from config import *
from controller import *
from cart import *
from cart_painter import *
from world import *
from cell_decomposition import *
from phidias_interface import Messaging, start_message_server_http
from path_finder import *

from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QWidget

import math
import sys
import random


class MainWindow(QWidget):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.initUI()
        
    def initUI(self):
        self.setGeometry(0, 0, WIDTH, HEIGHT)
        self.setWindowTitle('Danilo Spinelli Project')
        self.show()

        self.delta_t = 1e-2  # 10ms of time-tick #put 1e-3
        self.t = 0

        mass = 20 #Kg
        radius = 0.15 #m
        friction = 7e-5
        saturation = 150 #N
        
        self.path_controller = Path2D(2.0, 2, 2, 0.01)
        
        self.robot_two_wheels = TwoWheelsCart2DEncodersOdometry(mass, radius, friction, friction,0.025, 0.025, 0.2, 0.02, 0.02, 0.24, 2 * math.pi / 4000.0, self.path_controller)

        self.left_controller = PID_Sat_Controller(5.0, 3.0, 0.0, saturation)
        self.right_controller = PID_Sat_Controller(5.0, 3.0, 0.0, saturation)
        
        self.target = (0.1, 0.1)
        self.linear_speed_profile_controller = SpeedProfileGenerator2D(self.target, 2.0, 5, 5)
        self.angular_speed_profile_controller = SpeedProfileGenerator(math.radians(90), 2, 10, 10)

        self.painter = CartTwoWheelsPainter(self.robot_two_wheels)

        self.world = World(self)

        self._timer_painter = QtCore.QTimer(self)
        self._timer_painter.timeout.connect(self.go)
        self._timer_painter.start(int(self.delta_t * 1000))
        self.notification = False

        #self._timer_sensor = QtCore.QTimer(self)
        #self._timer_sensor.timeout.connect(self.send_pos)
        #self._timer_sensor.start(500)

        self._from = None
        
        self.possible_block_list = []
        self.initialize_block_position_list()
        
        self.all_obstacle_n_group = []
        self.all_obstacle_n_group.append(obstacle1_n)
        self.all_obstacle_n_group.append(obstacle2_n)
        self.all_obstacle_n_group.append(obstacle3_n)
        
        self.boundary = [pointA, pointB, pointD, pointC]
        self.cell_decomposition = CellDecomposition(self.all_obstacle_n_group, 10)
        self.vertical_lines, self.vertex, self.edges, self.graph, self.graph_vertices = self.cell_decomposition.find_vertical_lines_vertex_and_edges()
        self.graph_vertices_without_fixed_pos = self.graph_vertices[:29]
        
    def set_from(self, _from):
        self._from = _from
        
    def go(self):
        #Prendo la velocità dal Distance Control
        v_target = self.linear_speed_profile_controller.evaluate(self.delta_t, self.robot_two_wheels.get_pose())
        self.angular_speed_profile_controller.set_target(self.linear_speed_profile_controller.target_heading)
        
        #Prendo la velocità angolare dal Distance Control
        w_target = self.angular_speed_profile_controller.evaluate(self.delta_t, self.robot_two_wheels.get_pose()[2])
        (vl, vr) = self.robot_two_wheels.get_wheel_speed()
        vref_l = v_target - w_target * self.robot_two_wheels.encoder_wheelbase / 2.0
        vref_r = v_target + w_target * self.robot_two_wheels.encoder_wheelbase / 2.0

        # same vref
        Tleft = self.left_controller.evaluate(vref_l-vl, self.delta_t)
        Tright = self.right_controller.evaluate(vref_r-vr, self.delta_t)

        self.robot_two_wheels.evaluate(self.delta_t, Tleft, Tright)
        
        self.t += self.delta_t
        self.update()
        
    def initialize_block_position_list(self):
        for pos in BLOCK_POSITIONS[4:]:
            b = Block('white')
            b.set_pose(pos[0], pos[1], 0)
            self.possible_block_list.append(b)
            
    def generate_blocks(self):
        possible_positions = BLOCK_POSITIONS.copy()
        if self.world.count_blocks() == 10:
            return
        while self.world.count_blocks() < 10:
            index = int(random.uniform(4, len(possible_positions)))
            block_position = possible_positions[index]
            possible_positions.pop(index)
            col = int(random.uniform(0, 3))
            self.world.new_block(COLOR_NAMES[col], block_position[0],block_position[1])
            
    def get_target_minus_robot(self, x_A, y_A, x_B, y_B):
        # Distanza desiderata
        distanza = 0.05
        
        delta_x = x_B - x_A
        delta_y = y_B - y_A

        # Calcola la lunghezza euclidea della differenza
        lunghezza = math.sqrt(delta_x**2 + delta_y**2)

        # Normalizza la differenza per ottenere un vettore unitario
        unitario_x = delta_x / lunghezza
        unitario_y = delta_y / lunghezza

        # Calcola le coordinate del terzo punto C
        x_C = x_B - distanza * unitario_x
        y_C = y_B - distanza * unitario_y
        return x_C, y_C
            
    def get_vertex_nearest_to_robot_position(self):
        x_pos, y_pos, w_angle = self.robot_two_wheels.get_pose()
        x_pos = (Pose.x_center + x_pos * 1000)
        y_pos = (Pose.y_center - y_pos * 1000)
        min_dist = 10000000
        nearest_vertex = 0
        for idx, vtx in enumerate(self.graph_vertices_without_fixed_pos):
            temp_dist = int(math.sqrt(pow((x_pos - vtx.x),2) + pow(y_pos - vtx.y, 2)))
            if(temp_dist < min_dist):
                min_dist = temp_dist
                nearest_vertex = idx
        return nearest_vertex
            
    def scan_nearest_position(self, destinations):
        if(destinations == []):
            return
        source = self.get_vertex_nearest_to_robot_position()
        path, destination = find_path_to_nearest_block(self.graph, self.graph_vertices, source, destinations)
        if(path is None):
            print("No path found. Sorry")
            sys.exit()
        else:
            print("Path found.")
            for i in path:
                x_c, y_c = Pose.xy_pixel_to_coodinate(self.graph_vertices[i].x, self.graph_vertices[i].y)
                if i == path[len(path)-1]:
                    j = path[len(path)-2]
                    x1 = (abs(Pose.x_center - self.graph_vertices[j].x))/1000
                    y1 = (abs(Pose.y_center - self.graph_vertices[j].y))/1000
                    x_c, y_c = self.get_target_minus_robot(x1, y1, x_c, y_c)
                
                self.linear_speed_profile_controller.set_target((x_c, y_c))
                x, y, w = self.robot_two_wheels.get_pose()
                error_x = abs(x - x_c)
                error_y = abs(y - y_c)
                while(error_x > 0.01 or error_y > 0.01):
                    x, y, w = self.robot_two_wheels.get_pose()
                    error_x = abs(x - x_c)
                    error_y = abs(y - y_c)
            self.notify_nearest_target_got(destination)
        return destination
    
    def notify_nearest_target_got(self, destination):
        self.notification = True
        if self._from is not None:
            Messaging.send_belief(self._from, 'nearest_target_reached', [destination], 'robot')
            
    def go_to_nearest_block(self, destinations):
        if(destinations == []):
            return
        source = self.get_vertex_nearest_to_robot_position()
        path, destination = find_path_to_nearest_block(self.graph, self.graph_vertices, source, destinations)
        if(path is None):
            print("No path found. Sorry")
            sys.exit()
        else:
            print("Path found.")
            for i in path:
                x_c, y_c = Pose.xy_pixel_to_coodinate(self.graph_vertices[i].x, self.graph_vertices[i].y)
                self.linear_speed_profile_controller.set_target((x_c, y_c))
                x, y, w = self.robot_two_wheels.get_pose()
                error_x = abs(x - x_c)
                error_y = abs(y - y_c)
                while(error_x > 0.01 or error_y > 0.01):
                    x, y, w = self.robot_two_wheels.get_pose()
                    error_x = abs(x - x_c)
                    error_y = abs(y - y_c)
            if(destination != None):
                difference = int(destination - 28)
                block_pos = BLOCK_POSITIONS[difference]
                block = self.world.get_block_by_position(block_pos[0], block_pos[1])
                self.world.remove_block(block)
                if block != None and block.get_color() == 'red':
                    self.notify_nearest_red_picked(destination)
                elif block != None and block.get_color() == 'green':
                    self.notify_nearest_green_picked(destination)
                elif block != None and block.get_color() == 'blue':
                    self.notify_nearest_blue_picked(destination)
        return destination
    
    def notify_nearest_red_picked(self, red):
        self.notification = True
        if self._from is not None:
            Messaging.send_belief(self._from, 'nearest_red_picked', [red], 'robot')
            
    def notify_nearest_green_picked(self, green):
        self.notification = True
        if self._from is not None:
            Messaging.send_belief(self._from, 'nearest_green_picked', [green], 'robot')
            
    def notify_nearest_blue_picked(self, blue):
        self.notification = True
        if self._from is not None:
            Messaging.send_belief(self._from, 'nearest_blue_picked', [blue], 'robot')
            
    def go_to_bucket(self, dest):
        source = self.get_vertex_nearest_to_robot_position()
        path, destination = find_path_to_nearest_block(self.graph, self.graph_vertices, source, [28 + dest])
        if(path is None):
            print("No path found. Sorry")
            sys.exit()
        else:
            print("Path found.")
            for i in path:
                x_c, y_c = Pose.xy_pixel_to_coodinate(self.graph_vertices[i].x, self.graph_vertices[i].y)
                self.linear_speed_profile_controller.set_target((x_c, y_c))
                x, y, w = self.robot_two_wheels.get_pose()
                error_x = abs(x - x_c)
                error_y = abs(y - y_c)
                while(error_x > 0.01 or error_y > 0.01):
                    x, y, w = self.robot_two_wheels.get_pose()
                    error_x = abs(x - x_c)
                    error_y = abs(y - y_c)
            #Draw Block
            if dest == 1: #Red
                x_red = random.uniform(1.08, 1.12)
                y_red = random.uniform(0.02, 0.04)
                self.world.new_block_on_bucket(COLOR_NAMES[0], x_red, y_red)
            elif dest == 2: #Green
                x_green = random.uniform(1.08, 1.12)
                y_green = random.uniform(0.25, 0.27)
                self.world.new_block_on_bucket(COLOR_NAMES[1], x_green,y_green)
            else: #Blue
                x_blue = random.uniform(1.08, 1.12)
                y_blue = random.uniform(0.49, 0.51)
                self.world.new_block_on_bucket(COLOR_NAMES[2], x_blue,y_blue)
            self.notify_bucket_got()
            
    def go_to_start(self):
        source = self.get_vertex_nearest_to_robot_position()
        path, destination = find_path_to_nearest_block(self.graph, self.graph_vertices, source, [28])
        if(path is None):
            print("No path found. Sorry")
            sys.exit()
        else:
            print("Path found.")
            for i in path:
                x_c, y_c = Pose.xy_pixel_to_coodinate(self.graph_vertices[i].x, self.graph_vertices[i].y)
                self.linear_speed_profile_controller.set_target((x_c, y_c))
                x, y, w = self.robot_two_wheels.get_pose()
                error_x = abs(x - x_c)
                error_y = abs(y - y_c)
                while(error_x > 0.01 or error_y > 0.01):
                    x, y, w = self.robot_two_wheels.get_pose()
                    error_x = abs(x - x_c)
                    error_y = abs(y - y_c)
    
    def sense_color(self, destination):
        if self._from is not None:
            d = self.world.sense_color()
            if d is None:
                params = []
            else:
                params = [destination, d]
            Messaging.send_belief(self._from, 'color', params, 'robot')
            
    def notify_bucket_got(self):
        self.notification = True
        if self._from is not None:
            Messaging.send_belief(self._from, 'bucket_reached', [], 'robot')
        
    def paint_block_bucket(self, qp):
        #Red Bucket
        qp.setPen(QPen(Qt.red, 3)) # type: ignore
        qp.drawLine(pointD[0]-80, pointD[1]-80, pointD[0], pointD[1]-80)
        qp.drawLine(pointD[0]-80, pointD[1], pointD[0], pointD[1])
        qp.drawLine(pointD[0]-80, pointD[1]-80, pointD[0]-80, pointD[1])
        qp.drawLine(pointD[0], pointD[1]-80, pointD[0], pointD[1])
        
        #Green Bucket
        qp.setPen(QPen(Qt.green, 3)) # type: ignore
        qp.drawLine(1120, 285, 1200, 285)
        qp.drawLine(1120, 365, 1200, 365)
        qp.drawLine(1120, 285, 1120, 365)
        qp.drawLine(1200, 285, 1200, 365)
        
        #Blue Bucket
        qp.setPen(QPen(Qt.blue, 3)) # type: ignore
        qp.drawLine(pointB[0]-80, pointB[1], pointB[0], pointB[1])
        qp.drawLine(pointB[0]-80, pointB[1]+80, pointB[0], pointB[1]+80)
        qp.drawLine(pointB[0]-80, pointB[1], pointB[0]-80, pointB[1]+80)
        qp.drawLine(pointB[0], pointB[1], pointB[0], pointB[1]+80)
        
    def paint_block_possible_pos(self, qp):
        for b in self.possible_block_list:
            b.paint_position(qp)
            
    def paint_fixed_cell_lines(self, qp):
        qp.setPen(Qt.black) # type: ignore
        
        for line in self.vertical_lines:
            qp.drawLine(line[0], line[1], line[2], line[3])
            
    def paint_fixed_cell_graph(self, qp):
        for vrt in self.vertex:
            qp.drawEllipse(vrt[0], vrt[1], vrt[2], vrt[3])
            
        for edge in self.edges:
            qp.drawLine(edge[0], edge[1], edge[2], edge[3])
        
    def paintEvent(self, event):
        qp = QtGui.QPainter()
        qp.begin(self)
        qp.setPen(QtGui.QColor(255, 255, 255))
        qp.setBrush(QtGui.QColor(255, 255, 255))
        qp.drawRect(event.rect())

        qp.setPen(Qt.black)
        qp.drawLine(pointC[0], pointC[1], pointD[0], pointD[1])
        qp.drawLine(pointC[0], pointC[1], pointA[0], pointA[1])
        qp.drawLine(pointA[0], pointA[1], pointB[0], pointB[1])
        qp.drawLine(pointB[0], pointB[1], pointD[0], pointD[1])
        
        qp.setBrush(QtGui.QColor(0, 0, 0))
        qp.drawPolygon(obstacle1)
        qp.drawPolygon(obstacle2)
        qp.drawPolygon(obstacle3)
        
        self.paint_block_bucket(qp)
        
        #Optional Draw
        self.paint_block_possible_pos(qp)
        self.paint_fixed_cell_lines(qp)
        self.paint_fixed_cell_graph(qp)

        qp.setPen(Qt.black)
        self.world.paint(qp)
        self.painter.paint(qp, self.t)

        qp.end()


def main():
    app = QApplication(sys.argv)
    ex = MainWindow()
    start_message_server_http(ex)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()