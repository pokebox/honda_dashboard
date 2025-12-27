import serial
import time
import honda_data_pb2
import logging

ser = serial.Serial('COM13', 115200, timeout=1)

while True:
    try:
        data = ser.read_all()
        if len(data) > 0:
            pb_data = honda_data_pb2.CarStatus()
            pb_data.ParseFromString(data)
            print(pb_data)
        else:
            time.sleep(0.1)
    except Exception as e:
        logging.error(e)
ser.close()
