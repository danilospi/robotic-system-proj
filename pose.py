class Pose:

    x_center = 50
    y_center = 600

    def __init__(self):
        self.__x = 0
        self.__y = 0
        self.__a = 0

    def get_pose(self):
        return self.__x, self.__y, self.__a

    def get_a(self):
        return self.__a

    def set_pose(self, x, y, a):
        self.__x = x
        self.__y = y
        self.__a = a

    def to_pixel(self):
        return Pose.x_center + self.__x * 1000, Pose.y_center - self.__y * 1000
    
    @classmethod
    def xy_pixel_to_coodinate(_cls_, x, y):
        x_c = (abs(Pose.x_center - x))/1000
        y_c = (abs(Pose.y_center - y))/1000
        return x_c, y_c

    @classmethod
    def xy_to_pixel(_cls_, x, y):
        return Pose.x_center + x * 1000, Pose.y_center - y * 1000

    @classmethod
    def pixel_scale(_cls_, val):
        return val * 1000