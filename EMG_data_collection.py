import time
import serial
import csv
import pandas
import numpy as np
arduinoData = serial.Serial('/dev/cu.usbserial-0001', 115200)
raw_data_path = "fatigue_level0_20.csv"
time.sleep(1)

def write_csv(data_row):
    path = raw_data_path
    with open(path, 'a+') as f:
        csv_write = csv.writer(f);
        csv_write.writerow(data_row)

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    data_row = list()
    while(True):
        while (arduinoData.inWaiting()==0):
            pass
        dataPacket = arduinoData.readline()  # reply
        dataPacket = str(dataPacket, 'utf-8')
        splitPacket = dataPacket.split(",")

        t = time.time()
        timestamp = int(round(t * 1000))
        data_row.append(timestamp)

        for i in range(2):
           data_row.append(int(splitPacket[i]))

        write_csv(data_row)
        print(data_row)
        data_row.clear()
