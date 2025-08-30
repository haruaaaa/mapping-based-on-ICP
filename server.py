#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, MoveTank, OUTPUT_D, OUTPUT_A
import socket
import threading
import struct # для работы с бинарными данными
import time

motor_a = LargeMotor(OUTPUT_D)
motor_b = LargeMotor(OUTPUT_A)

# Инициализация моторов
# MoveTank - независимое управление моторами (A-левый , D- правый)
motors = MoveTank(OUTPUT_A, OUTPUT_D)

# Настройки сервера
HOST = '169.254.139.247'
PORT = 8089

# conn - объект соединения
# addr - адрес клиента
def handle_client(conn, addr):
    print("Conected:"+ str({addr}))
    try:
        st = time.time()
        while True:
            # recv - получение данных с клиента (8 байт) 
            data = conn.recv(8)

            if not data:
                break

            # Распаковываем данные
            #'>ff' - два числа float
            left_speed, right_speed= struct.unpack('>ff', data)
            
            # Управляем моторами
            # on - включаем моторы
            motors.on(left_speed, right_speed)
            
            speed_1 = motor_a.speed 
            speed_2 = motor_b.speed 

            # send - передаем угловые скорости на клиента
            # '>ff' - два числа float
            response = struct.pack('>ff', speed_1, speed_2)
            conn.send(response)

    # Ошибка соединения
    except ConnectionResetError:
        print("Otkl")
    # В любом случае выключаем моторы и закрываем соединение
    finally:
        motors.off()
        conn.close()

def main():
    # Создаем TCP-сокет (для передачи данных между устройствами в сети)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:

        # Для быстрого перезапуска сервера на том же порту

        # SOL_SOCKET - показывает что мы работаем на уровне сокета 
        # SO_REUSEADDR - позволяет повторно использовать порт
        # 1 - опция включена 
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Привязываем сокет к конкретному хосту и порту
        s.bind((HOST, PORT))
        # Переводит сокет в режим прослушивания входящих соединений
        s.listen()
        print("Pusk"+str({PORT}))
        
        try:
            while True:
                #  Принимаем соединение
                conn, addr = s.accept()
                # Поток для обработки клиента
                thread = threading.Thread(target=handle_client, args=(conn, addr))
                # Запускаем поток
                thread.start()

        # Обработка прерывания
        except KeyboardInterrupt:
            motors.off()

if __name__ == "__main__":
    main()