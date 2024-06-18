#导入矩阵、向量运算库
import numpy as np
#导入包的函数而非整个包便于提高代码运行速度
from math import sin
from math import cos
from math import tan
from math import atan
from math import atan2
from math import sqrt
#导入自编类
from class_file import Gyro
from class_file import Acc
from class_file import Pos
from class_file import Vel
from class_file import Att

#长度单位统一用m，角度单位统一用rad
pi=4*atan(1)
a=6378137 # m
const_e=0.08181919104
we = 7.292115e-5 # rad/s
Rad=pi/180 # rad/deg
Deg=180/pi # deg/rad
f=200 #采样率
dt=1/f # 采样时间间隔

'''
函数名：姿态矩阵转为欧拉角
参数：C 待转换的姿态矩阵
返回值：yaw、pitch、roll分别为航向角、俯仰角、横滚角（单位为rad）
'''
def Matrix2Euler(C):
    yaw = atan2(C[1, 0], C[0, 0])
    roll = atan2(C[2, 1], C[2, 2])
    pitch = atan2(-C[2, 0], sqrt(C[2, 1] * C[2, 1] + C[2, 2] * C[2, 2]))
    if (pitch > (pi / 2)):
        pitch = pitch - pi
    elif (pitch < -(pi / 2)):
        pitch = pitch + pi
    return yaw,pitch,roll


'''
函数名：欧拉角转为姿态矩阵
参数： yaw、pitch、roll分别为航向角、俯仰角、横滚角（单位为rad）
返回值：C 转换后的的姿态矩阵
'''
def Euler2Matrix(yaw,pitch,roll):
    C = np.array([[cos(pitch)*cos(yaw),
                   -cos(roll)*sin(yaw)+sin(roll)*sin(pitch)*cos(yaw) ,
                   sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw)],
                  [cos(pitch)*sin(yaw),
                   cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw),
                   -sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw)],
                  [-sin(pitch), sin(roll)*cos(pitch), cos(roll)*cos(pitch)]])
    return C


'''
函数名：计算子午圈和卯酉圈半径
参数： lat 纬度（单位为rad）
返回值：Rm和Rn分别是子午圈半径和卯酉圈半径(m)
'''
def Rmn(lat):
    Rm=a*(1-const_e*const_e)/sqrt((1-const_e*const_e*sin(lat)*sin(lat))**3)
    Rn=a/sqrt(1-const_e*const_e*sin(lat)*sin(lat))
    return Rm,Rn


'''
函数名：求向量的反对称阵
参数： vector 一个三维向量
返回值：M 它的反对称阵
'''
def v2m(vector):
    M = np.array([[0,-vector[2],vector[1]],
                  [vector[2],0,-vector[0]],
                  [-vector[1],vector[0], 0]])
    return M

'''
函数名：经纬度转换为站心坐标
参数： pos0 站心的经纬度(rad)、大地高坐标
      pos 待转点的经纬度(rad)、大地高坐标
返回值：N,E 站心坐标 m
'''
def lh2ne(pos0,pos):
    Rm,Rn=Rmn(pos.lat)
    N=(pos.lat-pos0.lat)*(Rm+pos.h)
    E=(pos.lon-pos0.lon)*(Rn+pos.h)*cos(pos.lat)
    return N,E


'''
函数名：读取输出值文件
参数： gyro 存放陀螺输出的列表 rad(角度增量)
      acc 存放加速度计输出的列表 m/s（速度增量）
'''
def read_file(gyro,acc,filename):
    # 读取文件中的 IMU 原始数据到 mImuDataEpoch 的 numpy 数组
    mImuDataEpoch = np.fromfile(filename, dtype="float64")
    mImuDataEpoch = mImuDataEpoch.reshape(int(len(mImuDataEpoch) / 7), 7)
    for row in mImuDataEpoch:
        # 创建 Gyro 对象并赋值
        gyro_obj = Gyro()
        gyro_obj.t = row[0]
        gyro_obj.x = row[1]
        gyro_obj.y = row[2]
        gyro_obj.z = row[3]
        gyro.append(gyro_obj)
        # 创建 Acc 对象并赋值
        acc_obj = Acc()
        acc_obj.t = row[0]
        acc_obj.x = row[4]
        acc_obj.y = row[5]
        acc_obj.z = row[6]
        acc.append(acc_obj)

'''
函数名：读取参考结果文件
参数： pos 存放参考位置的列表 
      vel 存放参考速度的列表 m/s
      att 存放参考姿态的列表 
'''
def read_refer_file(pos,vel,att,filename):
    # 读取文件中的纯惯导参考结果到 mInsNavEpoch 的 numpy 数组
    mInsNavEpoch = np.fromfile(filename, dtype= "float64")
    mInsNavEpoch = mInsNavEpoch.reshape(int(len(mInsNavEpoch) / 10), 10)
    for row in mInsNavEpoch:
        # 创建 Pos 对象并赋值
        pos_obj=Pos()
        pos_obj.t = row[0]
        pos_obj.lat = row[1]*Rad
        pos_obj.lon = row[2]*Rad
        pos_obj.h = row[3]
        pos.append(pos_obj)
        # 创建 Vel 对象并赋值
        vel_obj=Vel()
        vel_obj.t = row[0]
        vel_obj.n = row[4]
        vel_obj.e = row[5]
        vel_obj.d = row[6]
        vel.append(vel_obj)
        # 创建 Att 对象并赋值
        att_obj = Att()
        att_obj.t = row[0]
        att_obj.y = row[9]*Rad
        att_obj.p = row[8]*Rad
        att_obj.r = row[7]*Rad
        if att_obj.y>=pi:
            att_obj.y=att_obj.y-2*pi
        att.append(att_obj)


'''
函数名：读取输出值文件
参数： gyro 存放陀螺输出的列表 rad(角度增量)
      acc 存放加速度计输出的列表 m/s（速度增量）
'''
def read_measure_file(gyro,acc,filename):
    with open(filename, 'r') as file:
        for line in file:
            gyro_obj = Gyro()# 创建 Gyro 对象并赋值
            acc_obj = Acc()# 创建 Acc 对象并赋值
            line_data = line.strip().split(' ')
            # 读入为速度增量，单位m/s
            acc_obj.t=float(line_data[0])
            acc_obj.x = float(line_data[4])
            acc_obj.y = float(line_data[5])
            acc_obj.z = float(line_data[6])
            acc.append(acc_obj)
            # 读入角增量，单位rad
            gyro_obj.t=float(line_data[0])
            gyro_obj.x = float(line_data[1])
            gyro_obj.y = float(line_data[2])
            gyro_obj.z = float(line_data[3])
            gyro.append(gyro_obj)

'''
函数名：读取参考结果文件
参数： pos 存放参考位置的列表 
      vel 存放参考速度的列表 m/s
      att 存放参考姿态的列表 
'''
def read_measure_refer_file(pos,vel,att,filename):
    # 打开文件
    with open(filename, 'r') as file:
        # 跳过前两行
        for _ in range(2):
            next(file)
        # 逐行读取数据，并按两个空格分割
        for line in file:
            data = line.strip().split('  ')
            #判断是否为多余数据（t为整数时为多余数据）
            if float(data[0])% 1 == 0:
                continue
            # 创建 Pos 对象并赋值
            pos_obj = Pos()
            pos_obj.t = float(data[0])
            pos_obj.lat = float(data[2]) * Rad
            pos_obj.lon = float(data[1]) * Rad
            pos_obj.h = float(data[3])
            pos.append(pos_obj)
            # 创建 Vel 对象并赋值
            vel_obj = Vel()
            vel_obj.t = float(data[0])
            vel_obj.n = float(data[7])
            vel_obj.e = float(data[8])
            vel_obj.d = float(data[9])
            vel.append(vel_obj)
            # 创建 Att 对象并赋值
            att_obj = Att()
            att_obj.t = float(data[0])
            att_obj.y = float(data[6]) * Rad
            att_obj.p = float(data[5]) * Rad
            att_obj.r = float(data[4]) * Rad
            #控制读入的航向角在规定的-180°-180°之间
            if att_obj.y >= pi:
                att_obj.y = att_obj.y - 2 * pi
            att.append(att_obj)


'''
函数名：姿态更新函数
参数： gyro_k 存放k历元的陀螺输出的类实例 rad
      gyro_k1 存放k-1历元的陀螺输出的类实例 rad
      att 存放姿态的列表 rad
      vel 存放速度的列表 
      pos 存放位置的列表 
      t 当前历元时刻
'''
def att_update(gyro_k,gyro_k1,att,vel,pos,t):
    #b系从k到k-1的转换矩阵
    #当前时刻和前一时刻陀螺输出的角增量
    d_sigma_k=np.array([gyro_k.x, gyro_k.y, gyro_k.z])
    d_sigma_before_k=np.array([gyro_k1.x, gyro_k1.y, gyro_k1.z])

    #k的等效旋转矢量
    equ_k=d_sigma_k+np.cross(d_sigma_before_k, d_sigma_k)/12
    equ_norm = np.linalg.norm(equ_k) # 旋转矢量的模
    equ_matrix = v2m(equ_k) # 旋转矢量的反对称阵
    I=np.eye(3)
    Cb=I+sin(equ_norm)*equ_matrix/equ_norm+(1-cos(equ_norm))*(equ_matrix@equ_matrix)/(equ_norm**2)
    # n系从k-1到k的转换矩阵
    # 取k-1时刻的东向和北向速度
    ve=vel[-1].e
    vn=vel[-1].n
    # 取k-1时刻的纬度和大地高
    lat=pos[-1].lat
    h=pos[-1].h

    wie= np.array([we*cos(lat), 0, -we*sin(lat)])
    Rm,Rn=Rmn(lat)
    wen = np.array([ve/(Rn+h), -vn/(Rm+h), -ve*tan(lat)/(Rn+h)])
    equ1_k=(wie+wen)*dt
    equ1_norm = np.linalg.norm(equ1_k)  # 旋转矢量的模
    equ1_matrix =v2m(equ1_k)  # 旋转矢量的反对称阵
    Cn = I - sin(equ1_norm) * equ1_matrix / equ1_norm + (1 - cos(equ1_norm)) * (equ1_matrix @ equ1_matrix) / (equ1_norm ** 2)

    #更新
    # 找到上一历元的姿态
    y_k1 = att[-1].y
    p_k1 = att[-1].p
    r_k1 = att[-1].r

    Ck_before=Euler2Matrix(y_k1,p_k1,r_k1)
    Ck_=Cn@Ck_before
    Ck=Ck_@Cb
    att_obj1=Att()
    att_obj1.t=t
    att_obj1.y,att_obj1.p,att_obj1.r=Matrix2Euler(Ck)
    att.append(att_obj1)


'''
函数名：速度更新函数
参数： gyro_k 存放k历元的陀螺输出的类实例 rad
      gyro_k1 存放k-1历元的陀螺输出的类实例 rad
      acc_k 存放k历元的加速度计输出的类实例 
      acc_k1 存放k-1的历元的加速度计输出的类实例 
      vel 存放速度的列表 
      pos 存放位置的列表 
      att 存放姿态的列表 
      t 当前历元时刻
'''
def vel_update(gyro_k,gyro_k1,acc_k,acc_k1,vel,pos,att,t):
    #对需要线性外推到 k-1/2 的量操作:vn\ve\vd\lat\h
    vn_mid = 0
    ve_mid = 0
    vd_mid = 0
    if(len(vel)<2):
        vn_mid = vel[-1].n
        ve_mid = vel[-1].e
        vd_mid = vel[-1].d
    elif(len(vel)>=2):
        vn_mid = 3 * vel[-1].n / 2 - vel[-2].n / 2
        ve_mid = 3 * vel[-1].e / 2 - vel[-2].e / 2
        vd_mid = 3 * vel[-1].d / 2 - vel[-2].d / 2

    lat_mid = 0
    h_mid = 0
    if(len(pos)<2):
        lat_mid = pos[-1].lat
        h_mid = pos[-1].h
    elif(len(pos)>=2):
        lat_mid = 3 * pos[-1].lat / 2 - pos[-2].lat / 2
        h_mid = 3 * pos[-1].h / 2 - pos[-2].h / 2

    '''
    #探究：不进行线性外推，直接将k-1/2的中间时刻量取k-1即前一个历元的量
    vn_mid = vel[-1].n
    ve_mid = vel[-1].e
    vd_mid = vel[-1].d
    lat_mid = pos[-1].lat
    h_mid = pos[-1].h
    '''

    #重力
    g0=9.7803267715*(1+0.0052790414*(sin(lat_mid)**2)+0.0000232718*(sin(lat_mid)**4))
    g1=g0-(3.087691089e-6-4.397731e-9*(sin(lat_mid)**2))*h_mid+0.721e-12*(h_mid**2)
    g=np.array([0, 0, g1])
    #半径
    Rm, Rn = Rmn(lat_mid)
    #地球自转角速度在n系的投影
    wie = np.array([we * cos(lat_mid), 0, -we * sin(lat_mid)])
    #位移角速度在n系的投影
    wen = np.array([ve_mid / (Rn + h_mid), -vn_mid / (Rm + h_mid), -ve_mid * tan(lat_mid) / (Rn + h_mid)])
    #中间时刻的速度向量
    v_mid=np.array([vn_mid, ve_mid, vd_mid])
    #由重力项和哥式项引起的速度变化
    dvgc=(g-np.cross(2*wie+wen, v_mid))*dt
    #等效旋转矢量
    epk=(wie+wen)*dt
    epk_matrix=v2m(epk)
    #速度双子样项
    #提取双子样
    d_vk=np.array([acc_k.x, acc_k.y, acc_k.z])
    d_vk_before=np.array([acc_k1.x, acc_k1.y, acc_k1.z])
    d_sigma_k=np.array([gyro_k.x, gyro_k.y, gyro_k.z])
    d_sigma_before_k=np.array([gyro_k1.x, gyro_k1.y, gyro_k1.z])

    dvbfk=d_vk+np.cross(d_sigma_k, d_vk)/2+(np.cross(d_sigma_before_k, d_vk)+np.cross(d_vk_before, d_sigma_k))/12

    #k-1的姿态矩阵
    y = att[-2].y
    p = att[-2].p
    r = att[-2].r
    Ck_before=Euler2Matrix(y,p,r)
    I = np.eye(3)
    #比力积分项引起的速度变化
    dvf_matrix=((I-epk_matrix/2)@Ck_before)@np.reshape(dvbfk, (3, 1))
    #转换为正常的三维向量
    dvf = np.reshape(dvf_matrix, (3,))
    #速度更新
    v_obj=Vel()
    v_obj.t=t
    v_obj.n = vel[-1].n + dvgc[0] + dvf[0]
    v_obj.e = vel[-1].e + dvgc[1] + dvf[1]
    v_obj.d = vel[-1].d + dvgc[2] + dvf[2]
    vel.append(v_obj)


'''
函数名：位置更新函数
      vel 存放速度的列表 
      pos 存放位置的列表 
      t 当前历元时刻
'''
def pos_update(vel,pos,t):
    pos_obj=Pos()
    pos_obj.t=t
    pos_k_before=np.zeros(3)
    vk_before=np.zeros(3)
    vk=np.zeros(3)
    pos_k_before[0] = pos[-1].lat
    pos_k_before[1] = pos[-1].lon
    pos_k_before[2] = pos[-1].h
    #找到上一历元和本历元的速度信息
    vk_before[0] = vel[-2].n
    vk_before[1] = vel[-2].e
    vk_before[2] = vel[-2].d
    vk[0] = vel[-1].n
    vk[1] = vel[-1].e
    vk[2] = vel[-1].d
    #h更新
    pos_obj.h=pos_k_before[2]-(vk_before[2]+vk[2])*dt/2

    Rm,_=Rmn(pos_k_before[0])
    h_mean=(pos_k_before[2]+pos_obj.h)/2
    #lat更新
    pos_obj.lat=pos_k_before[0]+(vk_before[0]+vk[0])*dt/(2*(Rm+h_mean))

    lat_mean=(pos_k_before[0]+pos_obj.lat)/2
    _,Rn=Rmn(lat_mean)
    #lon更新
    pos_obj.lon=pos_k_before[1]+(vk_before[1]+vk[1])*dt/(2*(Rn+h_mean)*cos(lat_mean))
    pos.append(pos_obj)


'''
零速修正：将传入时段的位置置零，姿态和位置锁死（与该时段开始时保持一致）
函数名：零速修正函数
参数： t_correct 修正时刻
      att 存放姿态的列表 rad
      vel 存放速度的列表 
      pos 存放位置的列表 
'''
def zero_v(t_correct,att,vel,pos):
        att_ob = Att()
        vel_ob = Vel()
        pos_ob = Pos()
        att_ob.t=vel_ob.t=pos_ob.t=t_correct
        vel_ob.n=vel_ob.e=vel_ob.d=0
        att_ob.y=att[-1].y
        att_ob.p = att[-1].p
        att_ob.r = att[-1].r
        pos_ob.lat=pos[-1].lat
        pos_ob.lon = pos[-1].lon
        pos_ob.h = pos[-1].h
        att.append(att_ob)
        vel.append(vel_ob)
        pos.append(pos_ob)


'''
函数名：角度控制函数
参数： angle 待转化的角度 rad
函数功能： 将不在规定范围-pi~pi的角度传入，通过加减周期来归算到规定范围（主要用于航向角）并以返回值形式输出
'''
def adjust_angle(angle):
    while angle < -pi or angle > pi:
        if angle < -pi:
            angle += 2*pi
        elif angle > pi:
            angle -= 2*pi
    return angle