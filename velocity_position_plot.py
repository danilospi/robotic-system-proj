import math
import sys
import math
import pathlib
from cart import TwoWheelsCart2DEncodersOdometry
from controller import PID_Sat_Controller, SpeedProfileGenerator, SpeedProfileGenerator2D
from plot import DataPlotter
from PyQt5 import QtGui, QtCore
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QApplication

from robot import RoboticSystem

class VelocityPositionPlot(RoboticSystem):
    
    def __init__(self):
        self.delta_t = 1e-3  # 10ms of time-tick #put 1e-3
        self.t = 0

        mass = 20 #Kg
        radius = 0.15 #m
        friction = 7e-5
        saturation = 130 #N
        
        self.robot_two_wheels = TwoWheelsCart2DEncodersOdometry(mass, radius, friction, friction,0.025, 0.025, 0.2, 0.02, 0.02, 0.24, 2 * math.pi / 4000.0, None)

        self.plotter = DataPlotter()
        self.left_controller = PID_Sat_Controller(3.1, 0.0, 0.0, saturation)
        self.right_controller = PID_Sat_Controller(3.1, 0.0, 0.0, saturation)
        
        self.target = (0.8, 0.0)
        self.linear_speed_profile_controller = SpeedProfileGenerator2D(self.target, 2.0, 2, 2)
        self.angular_speed_profile_controller = SpeedProfileGenerator(math.radians(90), 2, 10, 10)
        
    def run(self):
        v_target = self.linear_speed_profile_controller.evaluate(self.delta_t, self.robot_two_wheels.get_pose(), True)
        self.angular_speed_profile_controller.set_target(self.linear_speed_profile_controller.target_heading)
        
        w_target = self.angular_speed_profile_controller.evaluate(self.delta_t, self.robot_two_wheels.get_pose()[2])
        (vl, vr) = self.robot_two_wheels.get_wheel_speed()
        vref_l = v_target - w_target * self.robot_two_wheels.encoder_wheelbase / 2.0
        vref_r = v_target + w_target * self.robot_two_wheels.encoder_wheelbase / 2.0

        Tleft = self.left_controller.evaluate(vref_l-vl, self.delta_t)
        Tright = self.right_controller.evaluate(vref_r-vr, self.delta_t)

        self.robot_two_wheels.evaluate(self.delta_t, Tleft, Tright)

        self.plotter.add( 't', self.t)
        self.plotter.add( 'vl', vl)
        self.plotter.add( 'vr', vr)
        self.plotter.add( 'vref_l', vref_l)
        self.plotter.add( 'vref_r', vref_r)
        self.plotter.add('target', 0.8)
        self.plotter.add('position', self.get_pose()[0])
        if self.t > 3:
            self.plotter.plot( ['t', 'time'] , [ [ 'vref_l', 'Vref' ],
                                                 [ 'vl', 'VL' ] ])
            self.plotter.plot( ['t', 'time'] , [ [ 'vref_r', 'Vref' ],
                                                 [ 'vr', 'VR' ] ])
            self.plotter.plot(['t', 'time'], [['target', 'Target'],
                                              ['position', 'Current Position']])
            self.plotter.show()
            return False
        else:
            return True

    def get_pose(self):
        return self.robot_two_wheels.get_pose()

    def get_speed(self):
        return self.robot_two_wheels.get_speed()
    
class CartWindow(QWidget):

    def __init__(self, _compound_sys, _img='mobile_robot_2d.png'):
        super(CartWindow, self).__init__()
        self.compound_system = _compound_sys
        self.image = _img
        self.initUI()

    def initUI(self):
        self.setGeometry(0, 0, 1000, 600)
        self.setWindowTitle('Robot 2D Simulator')
        self.show()

        current_path = pathlib.Path(__file__).parent.resolve()
        image = str(current_path) + '/' + self.image

        self.robot_pic = QtGui.QPixmap(image)

        self.delta_t = 1e-3

        self._timer_painter = QtCore.QTimer(self)
        self._timer_painter.start(int(self.delta_t * 1000))
        self._timer_painter.timeout.connect(self.go)

    def go(self):
        if not (self.compound_system.step()):
            self._timer_painter.stop()
        self.update()  # repaint window

    def paintEvent(self, event):
        qp = QtGui.QPainter()
        qp.begin(self)
        qp.setPen(QtGui.QColor(255, 255, 255))
        qp.setBrush(QtGui.QColor(255, 255, 255))
        qp.drawRect(event.rect())

        (x, y, theta) = self.compound_system.get_pose()

        qp.setPen(Qt.black)
        qp.drawLine(50, 500, 900, 500)
        qp.drawLine(50, 500, 50, 50)
        qp.drawLine(50, 50, 900, 50)
        qp.drawLine(900, 50, 900, 500)

        qp.drawText(910, 20, "X  = %6.3f m" % (x))
        qp.drawText(910, 40, "Y  = %6.3f m" % (y))
        qp.drawText(910, 60, "Th = %6.3f deg" % (math.degrees(theta)))
        qp.drawText(910, 80, "T  = %6.3f s" % (self.compound_system.t))

        s = self.robot_pic.size()

        x_pos = int(50 + x * 1000 - s.width() / 2)
        y_pos = int(500 - y * 1000 - s.height() / 2)

        t = QtGui.QTransform()
        t.translate(x_pos + s.width() / 2, y_pos + s.height() / 2)
        t.rotate(-math.degrees(theta))
        t.translate(-(x_pos + s.width() / 2), - (y_pos + s.height() / 2))

        qp.setTransform(t)
        qp.drawPixmap(x_pos, y_pos, self.robot_pic)

        qp.end()


if __name__ == '__main__':
    cart_robot = VelocityPositionPlot()
    app = QApplication(sys.argv)
    ex = CartWindow(cart_robot)
    sys.exit(app.exec_())