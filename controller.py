import math
from geometry import *

class VirtualRobot:
    ACCEL = 0
    CRUISE = 1
    DECEL = 2
    TARGET = 3

    def __init__(self, _p_target, _vmax, _acc, _dec):
        self.p_target = _p_target
        self.vmax = _vmax
        self.accel = _acc
        self.decel = _dec
        self.v = 0  # current speed
        self.p = 0  # current position
        self.phase = VirtualRobot.ACCEL
        self.decel_distance = 0.5 * _vmax * _vmax / _dec

    def evaluate(self, delta_t):
        if self.phase == VirtualRobot.ACCEL:
            self.p = self.p + self.v * delta_t \
                     + self.accel * delta_t * delta_t / 2
            self.v = self.v + self.accel * delta_t
            distance = self.p_target - self.p
            if distance < 0:
                distance = 0
            if self.v >= self.vmax:
                self.v = self.vmax
                self.phase = VirtualRobot.CRUISE
            elif distance <= self.decel_distance:
                v_exp = math.sqrt(2 * self.decel * distance)
                if v_exp < self.v:
                    self.phase = VirtualRobot.DECEL

        elif self.phase == VirtualRobot.CRUISE:
            self.p = self.p + self.vmax * delta_t
            distance = self.p_target - self.p
            if distance <= self.decel_distance:
                self.phase = VirtualRobot.DECEL

        elif self.phase == VirtualRobot.DECEL:
            self.p = self.p + self.v * delta_t \
                     - self.decel * delta_t * delta_t / 2
            self.v = self.v - self.decel * delta_t
            if self.p >= self.p_target:
                self.v = 0
                self.p = self.p_target
                self.phase = VirtualRobot.TARGET

class StraightLine2DMotion:

    def __init__(self, _vmax, _acc, _dec):
        self.vmax = _vmax
        self.accel = _acc
        self.decel = _dec

    def start_motion(self, start, end):
        (self.xs, self.ys) = start
        (self.xe, self.ye) = end

        dx = self.xe - self.xs
        dy = self.ye - self.ys

        self.heading = math.atan2(dy, dx)
        self.distance = math.sqrt(dx * dx + dy * dy)

        self.virtual_robot = VirtualRobot(self.distance, self.vmax, self.accel, self.decel)

    def evaluate(self, delta_t):
        self.virtual_robot.evaluate(delta_t)

        xt = self.xs + self.virtual_robot.p * math.cos(self.heading)
        yt = self.ys + self.virtual_robot.p * math.sin(self.heading)

        return xt, yt

class Path2D:

    def __init__(self, _vmax, _acc, _dec, _threshold):
        self.threshold = _threshold
        self.path = []
        self.trajectory = StraightLine2DMotion(_vmax, _acc, _dec)

    def set_path(self, path):
        self.path = path

    def start(self, start_pos):
        print(start_pos)
        self.current_target = self.path.pop(0)
        self.trajectory.start_motion(start_pos, self.current_target)

    def evaluate(self, delta_t, pose):
        (x, y) = self.trajectory.evaluate(delta_t)
        self.x_current = x
        self.y_current = y
        target_distance = math.hypot(pose[0] - self.current_target[0],
                                     pose[1] - self.current_target[1])
        if target_distance < self.threshold:
            if len(self.path) == 0:
                return None
            else:
                self.start((x, y))

        return x, y
    
class PID_Sat_Controller:

    def __init__(self, kp, ki, kd, sat):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.saturation = sat
        self.integral = 0
        self.prev_error = 0
        self.saturation_flag = False

    def evaluate(self, u, delta_t):
        error = u
        if not self.saturation_flag:
            self.integral = self.integral + error * delta_t
        deriv = (error - self.prev_error) / delta_t
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * deriv
        if output > self.saturation:
            output = self.saturation
            self.saturation_flag = True
        elif output < -self.saturation:
            output = -self.saturation
            self.saturation_flag = True
        else:
            self.saturation_flag = False
        return output
    
# DA RICONTROLLARE ------------------------------------------------------------

class SpeedProfileGenerator2D:
    ACCEL = 0
    CRUISE = 1
    DECEL = 2
    TARGET = 3

    def __init__(self, _p_target, _vmax, _acc, _dec):
        self.p_target = _p_target
        self.vmax = _vmax
        self.accel = _acc
        self.decel = _dec
        self.v = 0  # current speed
        self.vp = 0  # current POSTIVE speed
        self.phase = SpeedProfileGenerator.ACCEL
        self.decel_distance = 0.5 * _vmax * _vmax / _dec
        self.threshold = 0.005 #5mm 

    def set_target(self, p):
        self.p_target = p
        self.v = 0  # current speed
        self.vp = 0  # current POSTIVE speed
        self.phase = SpeedProfileGenerator.ACCEL

    def evaluate(self, delta_t, current_pos, angle_constraint):
        dx = self.p_target[0] - current_pos[0]
        dy = self.p_target[1] - current_pos[1]

        distance = math.hypot(dy, dx)
        self.target_heading = math.atan2(dy, dx)

        if distance < self.threshold:
            self.v = 0
            self.vp = 0
            self.phase = SpeedProfileGenerator.TARGET
            self.target_heading = math.radians(90)
            return 0
        else:
            heading_error = normalize_angle(self.target_heading - current_pos[2])
            
            if angle_constraint and abs(heading_error) > math.radians(1):
                self.v = 0
                self.vp = 0
            else:
                self.v = self.accel * distance
                
            if self.v > self.vmax:
                self.v = self.vmax
            elif self.v < -self.vmax:
                self.v = -self.vmax
            
            return self.v
        
class SpeedProfileGenerator:
    ACCEL = 0
    CRUISE = 1
    DECEL = 2
    TARGET = 3

    def __init__(self, _p_target, _vmax, _acc, _dec):
        self.p_target = _p_target
        self.vmax = _vmax
        self.accel = _acc
        self.decel = _dec
        self.v = 0  # current speed
        self.vp = 0  # current POSTIVE speed
        self.phase = SpeedProfileGenerator.ACCEL
        self.decel_distance = 0.5 * _vmax * _vmax / _dec

    def set_target(self, p):
        self.p_target = p

    def evaluate(self, delta_t, current_pos):
        #distance = self.p_target - current_pos
        distance = normalize_angle(self.p_target - current_pos)
        if distance == 0:
            self.v = 0
            self.vp = 0
            self.phase = SpeedProfileGenerator.TARGET
            return 0
        elif distance < 0:
            distance = -distance
            sign = -1
        else:
            sign = 1

        if self.phase == SpeedProfileGenerator.ACCEL:
            self.vp = self.vp + self.accel * delta_t
            if self.vp >= self.vmax:
                self.vp = self.vmax
                self.phase = SpeedProfileGenerator.CRUISE
            elif distance <= self.decel_distance:
                v_exp = math.sqrt(2 * self.decel * distance)
                if v_exp < self.vp:
                    self.phase = SpeedProfileGenerator.DECEL

        elif self.phase == SpeedProfileGenerator.CRUISE:
            if distance <= self.decel_distance:
                self.phase = SpeedProfileGenerator.DECEL

        elif self.phase == SpeedProfileGenerator.DECEL:
            self.vp = math.sqrt(2 * self.decel * distance)

        self.v = sign * self.vp
        return self.v