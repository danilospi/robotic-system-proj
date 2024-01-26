import math
from geometry import *
    
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
#Questo blocco include sia il Distance control sia il Cartesian to Polar
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