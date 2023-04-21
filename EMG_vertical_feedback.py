import os
import sys
#import time
import math
import serial

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI

if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read('../robot.conf')
        ip = parser.get('xArm', 'ip')
    except:
        ip = input('Please input the xArm ip address:')
        if not ip:
            print('input error, exit')
            sys.exit(1)

arm = XArmAPI(ip)
arm.motion_enable(enable=True)

# 设置运动参数
speed_up = 1000 # 运动速度
speed_down = 400
z_step_up = 2
z_step_down = 0.5 # 每次垂直移动的距离
z_min = 370.0
z_max = 530.0
L = 5 #水平单位运行距离
x_min = 350
x_max = 630
y_min = -100
y_max = 240
# 设置拉力参数
#ts_threshold = 200

# 打开串口
arduinoData = serial.Serial('/dev/cu.usbserial-0001', 115200)

while True:
    # 读取数据
    while (arduinoData.inWaiting() == 0):
        pass
    dataPacket = arduinoData.readline()  # reply
    dataPacket = str(dataPacket, 'utf-8')
    # print(dataPacket)
    splitPacket = dataPacket.split(",")
    print(splitPacket)
    command = splitPacket[0]

    if command == 'down':
        pos = arm.get_position()
        pos = pos[1]
        print(pos)
        z_new = pos[2]-z_step_down
        if z_new < z_min: z_new = z_min

        arm.set_position(x=pos[0], y=pos[1], z=z_new,
                         roll=pos[3], pitch=pos[4], yaw=pos[5], speed=speed_down, wait=True)
        print('Move down')
    elif command == 'up':
        pos = arm.get_position()
        pos = pos[1]
        print(pos)
        z_new = pos[2] + z_step_up
        if z_new > z_max: z_new = z_max

        arm.set_position(x=pos[0], y=pos[1], z=z_new,
                         roll=pos[3], pitch=pos[4], yaw=pos[5], speed=speed_up, wait=False)
        print('Move up')
    elif command == 'stable':
        print('Stable')

    if command == '1':
        ID = int(splitPacket[1])
        xZF = -int(splitPacket[2])
        yZF = -int(splitPacket[3])
        absangle = float(splitPacket[4])
        pos = arm.get_position()
        pos = pos[1]
        m = L * math.sin(math.radians(absangle)) * xZF + pos[0]
        if m > x_max: m = x_max
        if m < x_min: m = x_min

        n = L * math.cos(math.radians(absangle)) * yZF + pos[1]
        if n > y_max: n = y_max
        if n < y_min: n = y_min
        print(math.sin(math.radians(absangle)), "    ", math.cos(math.radians(absangle)))
        # time.sleep(1)
        arm.set_position(x=m, y=n, z=pos[2], roll=pos[3], pitch=pos[4], yaw=pos[5], speed=1000, wait=False)
        # print(arm.get_position(), arm.get_position(is_radian=True))
