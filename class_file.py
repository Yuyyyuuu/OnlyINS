#存放陀螺的输出
class Gyro:
    def __init__(self):
        self.t = 0
        self.x = 0
        self.y = 0
        self.z = 0

#存放加速度计的输出
class Acc:
    def __init__(self):
        self.t = 0
        self.x = 0
        self.y = 0
        self.z = 0

#存放位置
class Pos:
    def __init__(self):
        self.t = 0
        self.lat = 0 #rad
        self.lon = 0 #rad
        self.h = 0 #m

#存放速度
class Vel:
    def __init__(self):
        self.t = 0
        self.n = 0 #m/s
        self.e = 0 #m/s
        self.d = 0 #m/s

#存放姿态
class Att:
    def __init__(self):
        self.t = 0
        self.y = 0  #航向 rad
        self.p = 0  #俯仰 rad
        self.r = 0  #横滚 rad