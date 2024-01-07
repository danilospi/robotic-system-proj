from pose import *
from PyQt5 import QtGui
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPen

COLOR_MAP = {'red': QtGui.QColor(255, 0, 0),
             'green': QtGui.QColor(0, 255, 0),
             'blue': QtGui.QColor(0, 0, 255),
             'white': QtGui.QColor(255, 255, 255)}

class Block:
    WIDTH = 0.02
    HEIGHT = 0.02
    GAP = 0.01

    def __init__(self, uColor):
        self.__color = uColor
        self.__pose = Pose()
        self.__w = Pose.pixel_scale(Block.WIDTH)
        self.__h = Pose.pixel_scale(Block.HEIGHT)

    def get_pose(self):
        return self.__pose.get_pose()

    def set_pose(self, x, y, a):
        self.__pose.set_pose(x, y, a)

    def get_color(self):
        return self.__color

    def paint(self, qp):
        qp.setPen(Qt.black) # type: ignore
        qp.setBrush(COLOR_MAP[self.__color])

        (x, y) = self.__pose.to_pixel()
        
        x = int(x)
        y = int(y)

        t = QtGui.QTransform()
        t.translate(x + self.__w / 2, y - self.__h / 2)
        t.rotate(-self.__pose.get_a())
        t.translate(-(x + self.__w / 2), -(y - self.__h / 2))

        qp.setTransform(t)
        qp.drawEllipse(x, y - int(self.__h), int(self.__w), int(self.__h))
        
    def paint_position(self, qp):
        qp.setPen(QPen(Qt.gray, 2, Qt.DotLine)) # type: ignore
        qp.setBrush(QtGui.QColor(255, 255, 255))

        (x, y) = self.__pose.to_pixel()
        
        x = int(x)
        y = int(y)

        t = QtGui.QTransform()
        t.translate(x + self.__w / 2, y - self.__h / 2)
        t.rotate(-self.__pose.get_a())
        t.translate(-(x + self.__w / 2), -(y - self.__h / 2))

        qp.setTransform(t)
        qp.drawEllipse(x, y - int(self.__h), int(self.__w), int(self.__h))