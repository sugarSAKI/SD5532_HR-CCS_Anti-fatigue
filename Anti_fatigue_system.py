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
speed_up = 1200 # 运动速度
speed_down = 500
z_step_up = 2
z_step_down = 0.5 # 每次垂直移动的距离
z_min = 370.0
z_max = 480.0
L = 5 #水平单位运行距离
x_min = 350
x_max = 600
y_min = -100
y_max = 260
# 设置拉力参数
#ts_threshold = 200

# 打开串口
arduinoData = serial.Serial('/dev/cu.usbserial-0001', 115200)

# 控制OLED的串口
serialPort = '/dev/cu.usbmodem1201'
ser = serial.Serial(serialPort, 9600, timeout=0.5)

while True:
    # 读取数据
    while (arduinoData.inWaiting() == 0):
        pass
    dataPacket = arduinoData.readline()  # reply
    dataPacket = str(dataPacket, 'utf-8')
    splitPacket = dataPacket.split(",")
    print(splitPacket)

    command_v = 'stable'
    command_h = '0'
    wl = "0"
    it = "0"
    print(len(splitPacket))
    if(splitPacket[0] in ['up','down','stable']):
        # 读垂直
        splitPacket_v = splitPacket[0:3]
        print(splitPacket_v)
        command_v = splitPacket_v[0]
        #wl = int(splitPacket[3])
        #it = int(splitPacket[4])
        wl = splitPacket[3]
        it = splitPacket[4]

    if(len(splitPacket)>5):
        if(splitPacket[5] in ['0', '1']):
            # 再读水平
            splitPacket_h = splitPacket[5:]
            print(splitPacket_h)
            command_h = splitPacket_h[0]

    print(command_v)
    print(command_h)
    if(command_v == 'down'):
        state_v = "d"
    elif(command_v == 'up'):
        state_v = "u"
    else:
        state_v = "s"

    ser.write(state_v.encode())
    ser.write(b",")
    #wl = bytes(wl)
    ser.write(wl.encode())
    ser.write(b",")
    #it = bytes(it)
    ser.write(it.encode())

    print(state_v.encode(),",",wl.encode(),",",it.encode())

    # 读x-arm数据
    pos = arm.get_position()
    pos = pos[1]
    print(pos)

    if command_v == 'down' and command_h == '1':
        #pos = arm.get_position()
        #pos = pos[1]
        #print(pos)
        # vertical
        splitPacket_h = splitPacket[5:]
        z_new = pos[2] - z_step_down
        if z_new < z_min: z_new = z_min
        # horizontal
        ID = int(splitPacket_h[1])
        xZF = -int(splitPacket_h[2])
        yZF = -int(splitPacket_h[3])
        absangle = float(splitPacket_h[4])
        m = L * math.sin(math.radians(absangle)) * xZF + pos[0]
        if m > x_max: m = x_max
        if m < x_min: m = x_min
        n = L * math.cos(math.radians(absangle)) * yZF + pos[1]
        if n > y_max: n = y_max
        if n < y_min: n = y_min
        arm.set_position(x=m, y=n, z=z_new, roll=pos[3], pitch=pos[4], yaw=pos[5], speed=speed_down, wait=False)

        print('Move down and move in x-y.')

    elif command_v == 'down' and command_h == '0':
        #pos = arm.get_position()
        #pos = pos[1]
        #print(pos)
        # vertical
        z_new = pos[2] - z_step_down
        if z_new < z_min: z_new = z_min
        # x-y不变
        arm.set_position(x=pos[0], y=pos[1], z=z_new,
                         roll=pos[3], pitch=pos[4], yaw=pos[5], speed=speed_down, wait=True)
        print('Move down.')

    elif command_v == 'up' and command_h == '1':
        #pos = arm.get_position()
        #pos = pos[1]
        #print(pos)
        # vertical
        z_new = pos[2] + z_step_up
        if z_new > z_max: z_new = z_max
        # horizontal
        splitPacket_h = splitPacket[5:]
        ID = int(splitPacket_h[1])
        xZF = -int(splitPacket_h[2])
        yZF = -int(splitPacket_h[3])
        absangle = float(splitPacket_h[4])
        m = L * math.sin(math.radians(absangle)) * xZF + pos[0]
        if m > x_max: m = x_max
        if m < x_min: m = x_min
        n = L * math.cos(math.radians(absangle)) * yZF + pos[1]
        if n > y_max: n = y_max
        if n < y_min: n = y_min
        arm.set_position(x=m, y=n, z=z_new, roll=pos[3], pitch=pos[4], yaw=pos[5], speed=speed_up, wait=False)

        print('Move up and move in x-y.')

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
speed_up = 1200 # 运动速度
speed_down = 500
z_step_up = 2
z_step_down = 0.5 # 每次垂直移动的距离
z_min = 370.0
z_max = 480.0
L = 5 #水平单位运行距离
x_min = 350
x_max = 600
y_min = -100
y_max = 260
# 设置拉力参数
#ts_threshold = 200

# 打开串口
arduinoData = serial.Serial('/dev/cu.usbserial-0001', 115200)

# 控制OLED的串口
serialPort = '/dev/cu.usbmodem1201'
ser = serial.Serial(serialPort, 9600, timeout=0.5)

while True:
    # 读取数据
    while (arduinoData.inWaiting() == 0):
        pass
    dataPacket = arduinoData.readline()  # reply
    dataPacket = str(dataPacket, 'utf-8')
    splitPacket = dataPacket.split(",")
    print(splitPacket)

    command_v = 'stable'
    command_h = '0'
    wl = "0"
    it = "0"
    print(len(splitPacket))
    if(splitPacket[0] in ['up','down','stable']):
        # 读垂直
        splitPacket_v = splitPacket[0:3]
        print(splitPacket_v)
        command_v = splitPacket_v[0]
        #wl = int(splitPacket[3])
        #it = int(splitPacket[4])
        wl = splitPacket[3]
        it = splitPacket[4]

    if(len(splitPacket)>5):
        if(splitPacket[5] in ['0', '1']):
            # 再读水平
            splitPacket_h = splitPacket[5:]
            print(splitPacket_h)
            command_h = splitPacket_h[0]

    print(command_v)
    print(command_h)
    if(command_v == 'down'):
        state_v = "d"
    elif(command_v == 'up'):
        state_v = "u"
    else:
        state_v = "s"

    ser.write(state_v.encode())
    ser.write(b",")
    #wl = bytes(wl)
    ser.write(wl.encode())
    ser.write(b",")
    #it = bytes(it)
    ser.write(it.encode())

    print(state_v.encode(),",",wl.encode(),",",it.encode())

    # 读x-arm数据
    pos = arm.get_position()
    pos = pos[1]
    print(pos)

    if command_v == 'down' and command_h == '1':
        #pos = arm.get_position()
        #pos = pos[1]
        #print(pos)
        # vertical
        splitPacket_h = splitPacket[5:]
        z_new = pos[2] - z_step_down
        if z_new < z_min: z_new = z_min
        # horizontal
        ID = int(splitPacket_h[1])
        xZF = -int(splitPacket_h[2])
        yZF = -int(splitPacket_h[3])
        absangle = float(splitPacket_h[4])
        m = L * math.sin(math.radians(absangle)) * xZF + pos[0]
        if m > x_max: m = x_max
        if m < x_min: m = x_min
        n = L * math.cos(math.radians(absangle)) * yZF + pos[1]
        if n > y_max: n = y_max
        if n < y_min: n = y_min
        arm.set_position(x=m, y=n, z=z_new, roll=pos[3], pitch=pos[4], yaw=pos[5], speed=speed_down, wait=False)

        print('Move down and move in x-y.')

    elif command_v == 'down' and command_h == '0':
        #pos = arm.get_position()
        #pos = pos[1]
        #print(pos)
        # vertical
        z_new = pos[2] - z_step_down
        if z_new < z_min: z_new = z_min
        # x-y不变
        arm.set_position(x=pos[0], y=pos[1], z=z_new,
                         roll=pos[3], pitch=pos[4], yaw=pos[5], speed=speed_down, wait=True)
        print('Move down.')

    elif command_v == 'up' and command_h == '1':
        #pos = arm.get_position()
        #pos = pos[1]
        #print(pos)
        # vertical
        z_new = pos[2] + z_step_up
        if z_new > z_max: z_new = z_max
        # horizontal
        splitPacket_h = splitPacket[5:]
        ID = int(splitPacket_h[1])
        xZF = -int(splitPacket_h[2])
        yZF = -int(splitPacket_h[3])
        absangle = float(splitPacket_h[4])
        m = L * math.sin(math.radians(absangle)) * xZF + pos[0]
        if m > x_max: m = x_max
        if m < x_min: m = x_min
        n = L * math.cos(math.radians(absangle)) * yZF + pos[1]
        if n > y_max: n = y_max
        if n < y_min: n = y_min
        arm.set_position(x=m, y=n, z=z_new, roll=pos[3], pitch=pos[4], yaw=pos[5], speed=speed_up, wait=False)

        print('Move up and move in x-y.')

    elif command_v == 'up' and command_h == '0':
        #pos = arm.get_position()
        #pos = pos[1]
        #print(pos)
        # vertical
        z_new = pos[2] + z_step_up
        if z_new > z_max: z_new = z_max
        # x-y不变
        arm.set_position(x=pos[0], y=pos[1], z=z_new,
                         roll=pos[3], pitch=pos[4], yaw=pos[5], speed=speed_up, wait=False)
        print('Move up.')

    elif command_v == 'stable' and command_h == '1':
        #pos = arm.get_position()
        #pos = pos[1]
        #print(pos)
        # horizontal
        splitPacket_h = splitPacket[5:]
        ID = int(splitPacket_h[1])
        xZF = -int(splitPacket_h[2])
        yZF = -int(splitPacket_h[3])
        absangle = float(splitPacket_h[4])
        #需要校准
        distance = int(splitPacket_h[5])
        speed_h = distance*0.1
        if (distance>10000): distance = 10000
        L = (distance/10000)*20+1
        m = L * math.sin(math.radians(absangle)) * xZF + pos[0]
        if m > x_max: m = x_max
        if m < x_min: m = x_min
        n = L * math.cos(math.radians(absangle)) * yZF + pos[1]
        if n > y_max: n = y_max
        if n < y_min: n = y_min
        arm.set_position(x=m, y=n, z=pos[2], roll=pos[3], pitch=pos[4], yaw=pos[5], speed=speed_h, wait=False)
        print('Stable in z and move in x-y.')

    elif command_v == 'stable' and command_h == '0':
        print('Stable in x-y-z.')
