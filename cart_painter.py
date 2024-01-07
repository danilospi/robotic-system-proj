from config import *
from PyQt5 import QtGui
import math
import pathlib

class CartTwoWheelsPainter():

    def __init__(self, cart, _img='mobile_robot_2d.png'):
        self.cart = cart
        current_path = pathlib.Path(__file__).parent.resolve()
        self.image = str(current_path) + '/' + _img

    def paint(self, qp, t):
        
        p = self.cart.get_pose()
        
        x = p[0]
        y = p[1]
        theta = p[2]
        
        qp.drawText(WIDTH-90, 20, "X  = %6.3f m" % (x))
        qp.drawText(WIDTH-90, 40, "Y  = %6.3f m" % (y))
        qp.drawText(WIDTH-90, 60, "Th = %6.3f deg" % (math.degrees(theta)))
        qp.drawText(WIDTH-90, 80, "T  = %6.3f s" % (t))
        
        self.robot_pic = QtGui.QPixmap(self.image)
        s = self.robot_pic.size()
        x_pos = int(50 + x * 1000 - s.width() / 2)
        y_pos = int(HEIGHT-100 - y * 1000 - s.height() / 2)
        
        t = QtGui.QTransform()
        t.translate(x_pos + s.width() / 2, y_pos + s.height() / 2)
        t.rotate(-math.degrees(theta))
        t.translate(-(x_pos + s.width() / 2), - (y_pos + s.height() / 2))

        qp.setTransform(t)
        qp.drawPixmap(x_pos, y_pos, self.robot_pic)