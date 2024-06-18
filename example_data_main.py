import class_file
import function_file
import matplotlib.pyplot as plt
import os

# 进行示例数据的惯性导航解算
if __name__ == '__main__':

    # 创建存放图像的文件夹
    folder_path = os.path.join(os.getcwd(), "example_images")  # 在当前工作目录下创建名为 "example_images" 的文件夹
    os.makedirs(folder_path, exist_ok=True)  # 如果文件夹已存在则忽略

    # 存放IMU输出的列表
    gyro = []
    acc = []
    # 存放自解算位置、参考真值位置的列表
    pos = []
    refer_pos = []
    # 存放自解算速度、参考真值速度的列表
    vel = []
    refer_vel = []
    # 存放自解算姿态、参考真值姿态的列表
    att = []
    refer_att = []

    # 读入测量数据和参考真值数据
    filename1 = r"D:\惯性导航编程作业\实验三 纯惯导动态定位\示例数据\IMU.bin"
    function_file.read_file(gyro, acc, filename1)
    filename2 = r"D:\惯性导航编程作业\实验三 纯惯导动态定位\示例数据\PureINS.bin"
    function_file.read_refer_file(refer_pos, refer_vel, refer_att, filename2)

    #检查读入的IMU测量数据
    # 加速度计输出绘图
    acc_t = [value.t for value in acc]
    acc_x = [value.x for value in acc]
    acc_y = [value.y for value in acc]
    acc_z = [value.z for value in acc]
    # 绘制曲线
    figure1=plt.figure(figsize=(10,5),num="acc_output")  # 设置画布的大小
    plt.plot(acc_t, acc_x, label="x")
    plt.plot(acc_t, acc_y, label="y")
    plt.plot(acc_t, acc_z, label="z")
    # 添加标题和标签
    plt.title("acc_output")  # 设置整个图的标题
    plt.xlabel("t/s")  # x 轴标签
    plt.ylabel("m/s")  # y 轴标签
    # 添加图例
    plt.legend()
    # 显示图形
    plt.show()
    # 保存图像到指定路径
    figure1.savefig(os.path.join(folder_path, 'acc_output.png'))

    # 陀螺输出绘图
    gyro_t = [value.t for value in gyro]
    gyro_x = [value.x for value in gyro]
    gyro_y = [value.y for value in gyro]
    gyro_z = [value.z for value in gyro]
    # 绘制曲线
    figure2=plt.figure(figsize=(10,5),num="gyro_output")  # 设置画布的大小
    plt.plot(gyro_t, gyro_x, label="x")
    plt.plot(gyro_t, gyro_y, label="y")
    plt.plot(gyro_t, gyro_z, label="z")
    # 添加标题和标签
    plt.title("gyro_output")  # 设置整个图的标题
    plt.xlabel("t/s")  # x 轴标签
    plt.ylabel("rad")  # y 轴标签
    # 添加图例
    plt.legend()
    # 显示图形
    plt.show()
    # 保存图像到指定路径
    figure2.savefig(os.path.join(folder_path, 'gyro_output.png'))

    #初始导航状态量赋值
    # 初始对准
    att_obj=class_file.Att()
    att_obj.t = 91620
    att_obj.y = -75.7498049314083 * function_file.Rad
    att_obj.p = -2.14251290749072 * function_file.Rad
    att_obj.r = 0.0107951084511778 * function_file.Rad
    att.append(att_obj)

    # 位置赋初值
    pos_obj = class_file.Pos()
    pos_obj.t = 91620
    pos_obj.lat = 23.1373950708 * function_file.Rad
    pos_obj.lon = 113.3713651222 * function_file.Rad
    pos_obj.h = 2.175
    pos.append(pos_obj)

    # 速度赋初值
    vel_obj = class_file.Vel()
    vel_obj.t = 91620
    vel_obj.n = vel_obj.e = vel_obj.d = 0
    vel.append(vel_obj)

    # 逐一历元调用函数进行航位推算，以t作为推进器
    t0 = 91620  # 初始时刻
    t = t0 + function_file.dt  # 循环推进器
    t_end = 95220  # 设置更新到的时刻

    # 画平面轨迹图所用E、N列表
    N = []
    E = []

    # 查找第一个需解算历元对应的IMU数据在列表内的索引
    index = 0
    for i in range(len(acc)):
        if abs(acc[i].t - (t0 + function_file.dt)) < 1e-4:
            index = i
            break

    while (t <= t_end):
        function_file.att_update(gyro[index], gyro[index - 1], att, vel, pos, t)
        function_file.vel_update(gyro[index], gyro[index - 1], acc[index], acc[index - 1], vel, pos, att, t)
        function_file.pos_update(vel, pos, t)
        index = index + 1
        t = t + function_file.dt
        n, e = function_file.lh2ne(pos[0], pos[-1])
        N.append(n)
        E.append(e)
        print(t) #检测程序历元更新进度

    print("calculate finish!") # 解算完毕

    # 任务1：推算结果输出到txt文件
    with open('INS result(example).txt', 'w') as file:
        # 遍历列表中的每个元素，并在一行中输出每个结构体的属性
        for i in range(len(att)):
            # 使用字符串格式化控制数据精度
            line = ' '.join(
                [f"{att[i].t:.3f}s", f"{att[i].y * function_file.Deg :.8f}°", f"{att[i].p * function_file.Deg:.8f}°", f"{att[i].r * function_file.Deg:.8f}°",
                 f"{vel[i].n:.8f}m/s", f"{vel[i].e:.8f}m/s", f"{vel[i].d:.8f}m/s",
                 f"{pos[i].lat * function_file.Deg :.14f}°", f"{pos[i].lon * function_file.Deg:.14f}°", f"{pos[i].h:.8f}m"])
            file.write(line + '\n')

    # 任务2：画出位置的差异时序曲线、速度的差异时序曲线、姿态的差异时序曲线
    diff_att = []
    diff_vel = []
    diff_pos = []
    # 计算差值
    for i in range(1, len(att)):
        a_obj = class_file.Att()
        v_obj = class_file.Vel()
        p_obj = class_file.Pos()
        a_obj.t = v_obj.t = p_obj.t = refer_att[i].t
        a_obj.y = att[i].y - refer_att[i - 1].y
        a_obj.p = att[i].p - refer_att[i - 1].p
        a_obj.r = att[i].r - refer_att[i - 1].r
        v_obj.n = vel[i].n - refer_vel[i - 1].n
        v_obj.e = vel[i].e - refer_vel[i - 1].e
        v_obj.d = vel[i].d - refer_vel[i - 1].d
        p_obj.lat = pos[i].lat - refer_pos[i - 1].lat
        p_obj.lon = pos[i].lon - refer_pos[i - 1].lon
        p_obj.h = pos[i].h - refer_pos[i - 1].h
        diff_att.append(a_obj)
        diff_vel.append(v_obj)
        diff_pos.append(p_obj)

    # 位置差异绘图列表准备
    pos_t = [value.t for value in diff_pos]
    pos_lat = [value.lat * function_file.Deg for value in diff_pos]
    pos_lon = [value.lon * function_file.Deg for value in diff_pos]
    pos_h = [value.h for value in diff_pos]

    # 绘制经纬度的曲线
    figure3 = plt.figure(figsize=(10, 5), num="pos(lat_lon)difference")  # 设置画布的大小
    plt.plot(pos_t, pos_lat, label="lat")  # 绘制 lat-t 曲线
    plt.plot(pos_t, pos_lon, label="lon")  # 绘制 lon-t 曲线
    # 添加标题和标签
    plt.title("pos(lat_lon)difference")  # 设置整个图的标题
    plt.xlabel("t/s")  # x 轴标签
    plt.ylabel("lat/deg, lon/deg")  # y 轴标签
    # 添加图例
    plt.legend()
    # 显示图形
    plt.show()
    # 保存图像到指定路径
    figure3.savefig(os.path.join(folder_path, 'pos(lat_lon)difference.png'))

    # 绘制高程的曲线
    figure4 = plt.figure(figsize=(10, 5), num="pos(h)difference")  # 设置画布的大小
    plt.plot(pos_t, pos_h, label="h")  # 绘制 h-t 曲线
    # 添加标题和标签
    plt.title("pos(h)difference")  # 设置整个图的标题
    plt.xlabel("t/s")  # x 轴标签
    plt.ylabel("h/m")  # y 轴标签
    # 添加图例
    plt.legend()
    # 显示图形
    plt.show()
    # 保存图像到指定路径
    figure4.savefig(os.path.join(folder_path, 'pos(h)difference.png'))

    # 速度差异绘图
    vel_t = [value.t for value in diff_vel]
    vel_n = [value.n for value in diff_vel]
    vel_e = [value.e for value in diff_vel]
    vel_d = [value.d for value in diff_vel]
    # 绘制曲线
    figure5 = plt.figure(figsize=(10, 5), num="vel_difference")  # 设置画布的大小
    plt.plot(vel_t, vel_n, label="vn")  # 绘制 vn-t 曲线
    plt.plot(vel_t, vel_e, label="ve")  # 绘制 ve-t 曲线
    plt.plot(vel_t, vel_d, label="vd")  # 绘制 d-t 曲线
    # 添加标题和标签
    plt.title("vel_difference")  # 设置整个图的标题
    plt.xlabel("t/s")  # x 轴标签
    plt.ylabel("vn(m/s), ve(m/s), vd(m/s)")  # y 轴标签
    # 添加图例
    plt.legend()
    # 显示图形
    plt.show()
    # 保存图像到指定路径
    figure5.savefig(os.path.join(folder_path, 'vel_difference.png'))

    # 姿态差异绘图
    att_t = [value.t for value in diff_att]
    att_y = [value.y * function_file.Deg for value in diff_att]
    att_p = [value.p * function_file.Deg for value in diff_att]
    att_r = [value.r * function_file.Deg for value in diff_att]
    # 绘制曲线
    figure6 = plt.figure(figsize=(10, 5), num="att_difference")  # 设置画布的大小
    plt.plot(att_t, att_y, label="yaw")  # 绘制 yaw-t 曲线
    plt.plot(att_t, att_p, label="pitch")  # 绘制 pitch-t 曲线
    plt.plot(att_t, att_r, label="roll")  # 绘制 roll-t 曲线
    # 添加标题和标签
    plt.title("att_difference")  # 设置整个图的标题
    plt.xlabel("t/s")  # x 轴标签
    plt.ylabel("yaw/deg, pitch/deg, roll/deg")  # y 轴标签
    # 添加图例
    plt.legend()
    # 显示图形
    plt.show()
    # 保存图像到指定路径
    figure6.savefig(os.path.join(folder_path, 'att_difference.png'))

    # 任务3：画出解算位置的轨迹图和对应参考真值的轨迹图
    refer_E = []
    refer_N = []
    for i in range(len(refer_pos)):
        r_N, r_E = function_file.lh2ne(refer_pos[0], refer_pos[i])
        refer_E.append(r_E)
        refer_N.append(r_N)

    figure7 = plt.figure(num="E-N tracking")  # 设置画布的大小
    plt.plot(E, N, label="my tracking")
    plt.plot(refer_E, refer_N, label="real tracking")
    plt.xlabel('E/m')
    plt.ylabel('N/m')
    plt.title('E-N tracking')
    plt.axis('equal')
    plt.grid(True)
    # 添加图例
    plt.legend()
    plt.show()
    # 保存图像到指定路径
    figure7.savefig(os.path.join(folder_path, 'E-N tracking'))

    # 示例任务4：画出解算位置的高程变化图和对应参考真值的高程变化图
    H = [value.h for value in pos]
    len_H = len(H)
    H_t = [value.t for value in refer_pos]
    H_t = H_t[:len_H]
    refer_H = [value.h for value in refer_pos]
    refer_H = refer_H[:len_H]
    figure8 = plt.figure(num="H")  # 设置画布的大小
    plt.plot(H_t, H, label="my H")
    plt.plot(H_t, refer_H, label="real H")
    plt.xlabel('t/s')
    plt.ylabel('H/m')
    plt.title('H')
    plt.grid(True)
    # 添加图例
    plt.legend()
    plt.show()
    # 保存图像到指定路径
    figure8.savefig(os.path.join(folder_path, 'H'))
