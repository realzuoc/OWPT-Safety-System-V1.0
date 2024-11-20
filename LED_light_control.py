import serial
import socket
from threading import Thread
import time

HOST = '127.0.0.1'
PORT = 61234




class LightController(Thread):
    def __init__(self):
        super().__init__()
        self.file_name = 'ctl_flag_off.flag'
        self.light_on = True
        self.stop_flag = False

    @staticmethod
    def sendData (sendData):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            s.sendall(bytes(sendData, 'ascii'))
            recData = s.recv(1024)
            s.close()

    def run(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))
            s.listen()
            while not self.stop_flag:
                conn, addr = s.accept()
                with conn:
                    print('Connected by', addr)
                    data = conn.recv(1024)
                    if data == b'stop server':
                        self.stop_flag = True
                    if data == b'light on':
                        self.light_on = True
                    if data == b'light off':
                        self.light_on = False
                    while True:
                        if not data:
                            break
                        self.last_received = data
                        conn.sendall(data)
                        data = conn.recv(1024)

    @staticmethod
    def stop():
        LightController.sendData('stop server')

    @staticmethod
    def setLightOn():
        LightController.sendData('light on')
        print("light on")
        print(time.time() * 1000)

    @staticmethod
    def setLightOff():
        LightController.sendData('light off')
        print('light off')
        print(time.time() * 1000)


    def controller(self):
        ser = serial.Serial("com7", baudrate=115200, bytesize=8, parity='N', stopbits=2)
        while ( not self.stop_flag):
            b = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            # b[0] = 20
            b[1] = 0  # x轴pan
            b[3] = 14  # y轴高度
            b[4] = 150
            b[5] = 6  # color 0-7 white; 8-14 red; 36-42 yellow;
            # b[7] = 60 # Gobo
            b[9] = 0  # pan/tilt speed 0-255
            # b[10] = 60  # mode, when moving, light on/off
            if not self.light_on:
                # continue
                b[8] = 0  # 光强

            else:
                b[8] = 255

            a = bytes([0])

        # b[1] = int(xy) // 256
        # b[2] = int(xy) % 256
            ser.baudrate = 115200
            ser.write(a)
            time.sleep(0.001)
            ser.baudrate = 250000
            ser.write(b)
            time.sleep(0.001)
        ser.close()

if __name__ == "__main__":
    lc = LightController()
    lc.start()
    # lc.controller()
    # workbook.close()