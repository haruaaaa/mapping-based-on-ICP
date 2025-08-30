import time
import keyboard
from math import pi, cos, sin, atan2, sqrt
import threading
import socket
import struct
import serial
from hokuyo.driver import hokuyo
from hokuyo.tools import serial_port
import pickle


uart_port = 'COM5'
uart_speed = 19200

# подключение лазера
laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout= 1000)
port = serial_port.SerialPort(laser_serial)
laser = hokuyo.Hokuyo(port)


Host = '169.254.139.247' #изменяется 
Port = 8089

T = 0.08
inputSignals = [0, 0]
outputSignals = [0, 0]

# запись данных для одометрии в файл
def receive_data(inputData: list, s: socket.socket, odometry_file):

    while True:
        st = time.time()

        chunk = s.recv(8)
        if chunk == b'':
            raise RuntimeError("socket connection broken")

        wr, wl = struct.unpack(">ff", chunk)
        inputData[0] = wr
        inputData[1] = wl

        et = time.time()
        pickle.dump((et, wr, wl), odometry_file)
        odometry_file.flush()
        dt = T - (et - st)

        if dt < 0:
            print('warn')
        else:
            time.sleep(dt)

# отправка данных на сервер
def send_data(outputData: list, s: socket.socket):

    while True:
        st = time.time()
        sent = s.send(struct.pack(">ff", outputData[0], outputData[1]))
        if sent == 0:
            raise RuntimeError("socket connection broken")

        et = time.time()
        dt = T - (et - st)
        if dt < 0:
            print('warn')
        else:
            time.sleep(dt)

# управление с клавиатуры
def keyboard_control():
    speed = 25
    while True:
        # вперед / назад
        if keyboard.is_pressed('w'):
            outputSignals[0] = 25
            outputSignals[1] = 25
        elif keyboard.is_pressed('s'):
            outputSignals[0] = -25
            outputSignals[1] = -25
        # поворот вокруг колеса
        elif keyboard.is_pressed('a'):
            outputSignals[0] = 0
            outputSignals[1] = 20
        elif keyboard.is_pressed('d'):
            outputSignals[0] = 20
            outputSignals[1] = 0
        # поворот с движением
        elif keyboard.is_pressed('q'):
            outputSignals[0] = 15
            outputSignals[1] = 25
        elif keyboard.is_pressed('e'):
            outputSignals[0] = 25
            outputSignals[1] = 15
        # стоп / выкл
        elif keyboard.is_pressed('z'):
            outputSignals[0] = 0
            outputSignals[1] = 0
        elif keyboard.is_pressed('esc'):
            outputSignals[0] = 0
            outputSignals[1] = 0
            break
        time.sleep(1/30)

# включение лидара, сканирование, запись данных
def lidar(laser, lidar_file):

    laser.laser_on()
    while True:
        data = laser.get_single_scan()

        if data == b'':
            raise RuntimeError("socket connection broken")

        t = time.time()

        # запись данных с лидара за каждый отдельный скан
        for a, d in data.items():
            a = a * pi/180
            d = d / 1000
            pickle.dump((t, d, a), lidar_file)
            lidar_file.flush()
        


def main():
    print("Wait for connecting")
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s, \
        open('data_odom.pkl', 'wb') as odometry_file, \
        open('data_laser.pkl', 'wb') as lidar_file:

        try:

            s.connect((Host, Port))
            print("Connected")


            pubThr = threading.Thread(target=send_data, args=(outputSignals, s ))
            subThr = threading.Thread(target=receive_data, args=(inputSignals, s, odometry_file))
            lidarThr = threading.Thread(target=lidar, args=(laser, lidar_file))

            pubThr.start()
            subThr.start()
            lidarThr.start()
            
            keyboard_control()

        except ConnectionRefusedError:
            print("NO")
        except KeyboardInterrupt:
            print('klava')
        finally:
            outputSignals[0] = 0
            outputSignals[1] = 0
            laser.reset()
            laser.laser_off()
            laser_serial.close()
            

if __name__ == "__main__":
    main()