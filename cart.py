from geometry import *
import math

class Cart2D:

    def __init__(self, _mass: float, _radius: float, _lin_friction: float, _ang_friction: float):
        self.M: float = _mass
        self.b: float = _lin_friction
        self.beta: float = _ang_friction
        self.Iz: float = 0.5 * _mass * _radius * _radius
        # Iz = moment of inertia (the robot is a cylinder)
        self.v: float = 0
        self.w: float = 0
        self.x: float = 0
        self.y: float = 0
        self.theta: float = 0

    def evaluate(self, delta_t: float, _force, _torque: float) -> None:
        new_v: float = self.v * (1 - self.b * delta_t / self.M) + delta_t * _force / self.M
        new_w: float = self.w * (1 - self.beta * delta_t / self.Iz) + delta_t * _torque / self.Iz
        self.x = self.x + self.v * delta_t * math.cos(self.theta)
        self.y = self.y + self.v * delta_t * math.sin(self.theta)
        self.theta = self.theta + delta_t * self.w
        self.v = new_v
        self.w = new_w

    def get_pose(self) -> (float, float, float):
        return self.x, self.y, self.theta

class TwoWheelsCart2D(Cart2D):

    def __init__(self, _mass: float, _radius: float, _lin_friction: float, _ang_friction: float, _traction_wheelbase: float):
        super().__init__(_mass, _radius, _lin_friction, _ang_friction)
        self.traction_wheelbase: float = _traction_wheelbase

    def evaluate(self, delta_t: float, f_left: float, f_right: float) -> None:
        f = f_left + f_right
        t = self.traction_wheelbase * (f_right - f_left)
        super().evaluate(delta_t, f, t)
        
    def get_pose(self) -> (float, float, float):
        return self.x_r, self.y_r, self.theta_r

    def get_speed(self) -> (float, float):
        return self.v_r, self.w_r

    def get_wheel_speed(self) -> (float, float):
        return self.vleft, self.vright

class TwoWheelsCart2DEncoders(TwoWheelsCart2D):

    def __init__(self, _mass, _radius, _lin_friction, _ang_friction,
                 _r_traction_left, _r_traction_right, _traction_wheelbase,
                 _r_encoder_left, _r_encoder_right, _encoder_wheelbase, _encoder_resolution):
        super().__init__(_mass, _radius, _lin_friction, _ang_friction, _traction_wheelbase)
        self.r_traction_left = _r_traction_left
        self.r_traction_right = _r_traction_right
        self.r_encoder_left = _r_encoder_left
        self.r_encoder_right = _r_encoder_right
        self.encoder_wheelbase = _encoder_wheelbase
        self.encoder_resolution = _encoder_resolution

    def evaluate(self, delta_t, torque_left, torque_right) -> None:
        # torque to force
        f_left = torque_left / self.r_traction_left
        f_right = torque_right / self.r_traction_right
        # dynamic model
        super().evaluate(delta_t, f_left, f_right)

        # sensing wheels
        vl = self.v - self.w * self.encoder_wheelbase / 2
        vr = self.v + self.w * self.encoder_wheelbase / 2

        self.delta_rot_left = int(
            (vl / self.r_encoder_left) * (delta_t / self.encoder_resolution)) * self.encoder_resolution
        self.delta_rot_right = int(
            (vr / self.r_encoder_right) * (delta_t / self.encoder_resolution)) * self.encoder_resolution

class TwoWheelsCart2DEncodersOdometry(TwoWheelsCart2DEncoders):

    def __init__(self, _mass: float, _radius: float, _lin_friction: float, _ang_friction: float,
                 _r_traction_left: float, _r_traction_right: float, _traction_wheelbase: float,
                 _r_encoder_left: float, _r_encoder_right: float, _encoder_wheelbase: float, _encoder_resolution: float, path_controller):
        super().__init__(_mass, _radius, _lin_friction, _ang_friction,
                         _r_traction_left, _r_traction_right, _traction_wheelbase,
                         _r_encoder_left, _r_encoder_right, _encoder_wheelbase, _encoder_resolution)
        self.x_r: float = 0
        self.y_r: float = 0
        self.theta_r: float = 0
        self.vleft: float = 0
        self.vright: float = 0
        self.path_controller = path_controller

    def evaluate(self, delta_t, torque_left, torque_right):
        super().evaluate(delta_t, torque_left, torque_right)

        # odometry model

        p_left: float = self.delta_rot_left * self.r_encoder_left
        p_right: float = self.delta_rot_right * self.r_encoder_right

        self.vleft = p_left / delta_t
        self.vright = p_right / delta_t

        self.v_r = (self.vleft + self.vright) / 2
        self.w_r = (self.vright - self.vleft) / self.encoder_wheelbase

        delta_p: float = (p_left + p_right)

        delta_theta: float = (p_right - p_left) / self.encoder_wheelbase

        self.x_r = self.x_r + delta_p * math.cos(self.theta_r + delta_theta / 2)
        self.y_r = self.y_r + delta_p * math.sin(self.theta_r + delta_theta / 2)
        self.theta_r = normalize_angle(self.theta_r + delta_theta)

    def get_pose(self) -> (float, float, float):
        return self.x_r, self.y_r, self.theta_r

    def get_speed(self) -> (float, float):
        return self.v_r, self.w_r

    def get_wheel_speed(self) -> (float, float):
        return self.vleft, self.vright
    
    def set_target_xy(self, x, y):
        self.path_controller.set_path([(x,y)])
        (x, y, _) = self.get_pose()
        self.path_controller.start((x, y))