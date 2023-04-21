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
# arm.set_mode(0)
# arm.set_state(state=0)
# arm.reset(wait=True)

L = 5 #单位运行距离

#os.system('cls')


while 1:
    arduinoData = serial.Serial('/dev/cu.usbserial-0001', 115200)
    # time.sleep(1)
    while (arduinoData.inWaiting() == 0):
        pass
    dataPacket = arduinoData.readline()  # reply
    dataPacket = str(dataPacket, 'utf-8')
    #print(dataPacket)
    splitPacket = dataPacket.split(",")
    #print(splitPacket)
    Command = splitPacket[0]
    ID = int(splitPacket[1])
    xZF = -int(splitPacket[2])
    yZF = -int(splitPacket[3])
    absangle = float(splitPacket[4])
    print(Command, ID , "yZF=", yZF, "xZF=", xZF, "absangle=", absangle)
    if Command == '1' :
        pos = arm.get_position()
        pos1 = pos[1]
        m = L * math.sin(math.radians(absangle)) * xZF + pos1[0]
        n = L * math.cos(math.radians(absangle)) * yZF + pos1[1]
        print(math.sin(math.radians(absangle)),"    ",math.cos(math.radians(absangle)))
        # time.sleep(1)
        arm.set_position(x=m, y=n, z=160, roll=-180, pitch=0, yaw=0, speed=1000 , wait=False)
    #print(arm.get_position(), arm.get_position(is_radian=True))
