from block import *
import math

class World:
    def __init__(self, ui):
        self.__blocks = []
        self.__blocks_to_draw = []
        self.ui = ui

    def new_block(self, uColor, uX, uY):
        b = Block(uColor)
        b.set_pose(uX, uY, 0)
        self.__blocks.append(b)
        self.__blocks_to_draw.append(b)

    def count_blocks(self):
        return len(self.__blocks)
    
    def sense_color(self):
        (x_pos, y_pos, w) = self.ui.robot_two_wheels.get_pose()
        min_dist = 0.078
        nearest_color = "None"
        for b in self.__blocks:
            (xb, yb, ab) = b.get_pose()
            temp_dist = math.sqrt(pow((x_pos - xb),2) + pow(y_pos - yb, 2))
            if(temp_dist < min_dist):
                min_dist = temp_dist
                nearest_color = b.get_color()
        return nearest_color

    def paint(self, qp):
        for b in self.__blocks_to_draw:
            b.paint(qp)
            
    def get_block_by_position(self, x, y):
        for b in self.__blocks:
            (xb, yb, ab) = b.get_pose()
            if(x == xb and y == yb):
                return b
        return None
    
    def remove_block(self, block):
        self.__blocks.remove(block)
        self.__blocks_to_draw.remove(block)
        
    def new_block_on_bucket(self, uColor, uX, uY):
        b = Block(uColor)
        b.set_pose(uX, uY, 0)
        self.__blocks_to_draw.append(b)