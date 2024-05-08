
import serial
import time

ser_anc_1 = serial.Serial(
    port        = 'COM5',
    baudrate    = 115200
)

ser_anc_2 = serial.Serial(
    port        = 'COM6',
    baudrate    = 115200
)

for i in range(10000):
    try:
        anc_1_ns    = int(ser_anc_1.readline().decode())
        anc_1_ts    = int(ser_anc_1.readline().decode())
        anc_2_ns    = int(ser_anc_2.readline().decode())
        anc_2_ts    = int(ser_anc_2.readline().decode())

        print(anc_1_ns, ': ', anc_1_ts)
        print(anc_2_ns, ': ', anc_2_ts)

    except:
        pass