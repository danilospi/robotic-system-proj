
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

        self.delta_t = 1e-2  # 10ms of time-tick
        self.t = 0

        mass = 20 #Kg
        radius = 0.15 #m
        friction = 7e-5
        saturation = 130 #N
        
        #Inizializzo il robot mobile a due ruote indipenti (all'interno del metodo evalute sono presenti le formule di odometria per convertire lo spostamento in una posa del robot)
        self.robot_two_wheels = TwoWheelsCart2DEncodersOdometry(mass, radius, friction, friction,0.025, 0.025, 0.2, 0.02, 0.02, 0.24, 2 * math.pi / 4000.0, None)

        #Inizializzo due controlli proporzionali integrali (uno per ogni ruota)
        self.left_controller = PID_Sat_Controller(18.0, 2.0, 0.0, saturation)
        self.right_controller = PID_Sat_Controller(18.0, 2.0, 0.0, saturation)
        
        #Imposto il primo target per il robot (questo mi permette di posizionarlo sopra il vertice più vicino)
        self.target = (0.0, 0.0)
        
        #Inizializzo il Distance control utilizzando un controllo in posizione con profilo di velocità
        self.linear_speed_profile_controller = SpeedProfileGenerator2D(self.target, 2.0, 2, 2)
        
        #Inizializzo il Rotation control utilizzando un controllo in posizione con profilo di velocità
        self.angular_speed_profile_controller = SpeedProfileGenerator(math.radians(90), 2, 2, 2)

        #Inizializzo il Painter che mi permette di disegnare il robot e i suoi parametri
        self.painter = CartTwoWheelsPainter(self.robot_two_wheels)

        #Inizializzo l'oggetto world
        self.world = World(self)

        #Imposto un timer di 10ms che schedula il metodo go()
        self._timer_painter = QtCore.QTimer(self)
        self._timer_painter.timeout.connect(self.go)
        self._timer_painter.start(int(self.delta_t * 1000))
        self.notification = False

        self._from = None
        
        #La lista possible_block_list[] contiene tutte le 20 possibili posizioni in cui potrebbero trovarsi i dischi colorati
        self.possible_block_list = []
        #Questo metodo permette di inizializzare la lista
        self.initialize_block_position_list()
        
        #La lista all_obstacle_n_group[] contiene le posizioni dei 3 ostacoli che vengono disegnati nell'ambiente
        self.all_obstacle_n_group = []
        self.all_obstacle_n_group.append(obstacle1_n)
        self.all_obstacle_n_group.append(obstacle2_n)
        self.all_obstacle_n_group.append(obstacle3_n)
        
        #Inizializzo l'oggetto cell_decomposizion con il quale sarà possibile suddividere l'ambiente in celle
        self.cell_decomposition = CellDecomposition(self.all_obstacle_n_group, 10)
        #Grazie ai metodi dell'oggetto cell_decomposition ottengo in output: 
        # le posizioni (x1,y1,x2,y2) delle line verticali che compongono le celle, 
        # le posizioni (x,y,width, height) per poter disegnare i vertici del grafo, 
        # le posizioni (x1,y1,x2,y2) per poter disegnare gli archi del grafo, 
        # il grafo sotto forma di lista di adiacenza ed infine la lista dei vertici del grafo
        self.vertical_lines, self.vertex, self.edges, self.graph, self.graph_vertices = self.cell_decomposition.find_vertical_lines_vertex_and_edges()
        
        #Inizializzo una lista che contiene i vertici del grafo esclusi quelli dove potrebbero trovarsi i blocchi
        self.graph_vertices_without_fixed_pos = self.graph_vertices[:29]
        
    def set_from(self, _from):
        self._from = _from
        
    def go(self):
        
        #Passo in input al Distance Control la posa del robot e la converte in una velocità target,
        #riesce a fare questo perchè il metodo evaluate() contiene anche il blocco Cartesian to Polar,
        #questo significa che converte la posa in coordinate polari utilizzando la distanza euclidea e l'algoritmo dell'headingTo
        v_target = self.linear_speed_profile_controller.evaluate(self.delta_t, self.robot_two_wheels.get_pose(), True)
        
        #Passo al rotation control la theta target calcolata precedentemente
        self.angular_speed_profile_controller.set_target(self.linear_speed_profile_controller.target_heading)
        
        #Passo in input al Rotation Control la theta corrente ed ottengo in output la velocità angolare target
        w_target = self.angular_speed_profile_controller.evaluate(self.delta_t, self.robot_two_wheels.get_pose()[2])
        
        #Tramite la velocità e la velocità angolare ottengo in output la velocità target su ruota destra e ruota sinistra
        (vl, vr) = self.robot_two_wheels.get_wheel_speed()
        vref_l = v_target - w_target * self.robot_two_wheels.encoder_wheelbase / 2.0
        vref_r = v_target + w_target * self.robot_two_wheels.encoder_wheelbase / 2.0

        #Confronto le velocità target e le velocità correnti e tramite i controllori proporzionali-integrali
        # e li converto in coppie da applicare ai motori
        Tleft = self.left_controller.evaluate(vref_l-vl, self.delta_t)
        Tright = self.right_controller.evaluate(vref_r-vr, self.delta_t)

        #Infine chiamo il metodo evaluate() del robot che aggiornerà la posa del robot
        self.robot_two_wheels.evaluate(self.delta_t, Tleft, Tright)
        
        self.t += self.delta_t
        self.update()
        
    #Metodo per inizializzare la lista delle possibili posizioni dei dischi
    def initialize_block_position_list(self):
        for pos in BLOCK_POSITIONS[4:]:
            b = Block('white')
            b.set_pose(pos[0], pos[1], 0)
            self.possible_block_list.append(b)
            
    #Metodo per poter generare 10 dischi colorati (Invocato dalla procedura PHIDIAS: gen_block())
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
            
    #Metodo per ottenere le coordinate X,Y per poter far avvicinare il robot al disco senza farlo salire su di esso
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
            
    #Metodo per rilevare il vertice più vicino al robot
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
            
    #Metodo per poter scansionare le 20 possibili posizioni dei dischi (Invocato dalla procedura PHIDIAS: scan())
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
    
    #Metodo per notificare di aver raggiunto il target
    def notify_nearest_target_got(self, destination):
        self.notification = True
        if self._from is not None:
            Messaging.send_belief(self._from, 'nearest_target_reached', [destination], 'robot')
    
    #Questo metodo prende in input una lista di destinazioni, trova quella più vicina
    # e fa seguire al robot il percorso più veloce per raggiungerla
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
    
    #Metodo per notificare di aver preso un disco di colore rosso
    def notify_nearest_red_picked(self, red):
        self.notification = True
        if self._from is not None:
            Messaging.send_belief(self._from, 'nearest_red_picked', [red], 'robot')
    
    #Metodo per notificare di aver preso un disco di colore verde
    def notify_nearest_green_picked(self, green):
        self.notification = True
        if self._from is not None:
            Messaging.send_belief(self._from, 'nearest_green_picked', [green], 'robot')
    
    #Metodo per notificare di aver preso un disco di colore blu
    def notify_nearest_blue_picked(self, blue):
        self.notification = True
        if self._from is not None:
            Messaging.send_belief(self._from, 'nearest_blue_picked', [blue], 'robot')
    
    #Metodo per impostare come target al robot il raggiungimento di uno specifico bucket
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
            
    #Metodo per riportare il robot alla sua posizione iniziale
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
    
    #Metodo che simula l'azione di un sensore, permette di far capire al robot il colore di un disco
    def sense_color(self, destination):
        if self._from is not None:
            d = self.world.sense_color()
            if d is None:
                params = []
            else:
                params = [destination, d]
            Messaging.send_belief(self._from, 'color', params, 'robot')
            
    #Metodo per notificare di aver raggiunto il bucket
    def notify_bucket_got(self):
        self.notification = True
        if self._from is not None:
            Messaging.send_belief(self._from, 'bucket_reached', [], 'robot')
        
    #Metodo per disegnare i bucket
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
        
    #Metodo per disegnare le possibili posizioni in cui può trovarsi un disco
    def paint_block_possible_pos(self, qp):
        for b in self.possible_block_list:
            b.paint_position(qp)
            
    #Metodo per disegnare le celle di suddivisione dell'ambiente
    def paint_fixed_cell_lines(self, qp):
        qp.setPen(Qt.black) # type: ignore
        
        for line in self.vertical_lines:
            qp.drawLine(line[0], line[1], line[2], line[3])
    
    #Metodo per disegnare il grafo su cui il robot andrà a muoversi
    def paint_fixed_cell_graph(self, qp):
        for vrt in self.vertex:
            qp.drawEllipse(vrt[0], vrt[1], vrt[2], vrt[3])
            
        for edge in self.edges:
            qp.drawLine(edge[0], edge[1], edge[2], edge[3])
    
    #Metodo per disegnare tutto ciò che vediamo in esecuzione
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
        
        #Questi metodi possiamo anche non utilizzarli (al momento vengono utilizzati per capire meglio come è stato strutturato l'ambiente)
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