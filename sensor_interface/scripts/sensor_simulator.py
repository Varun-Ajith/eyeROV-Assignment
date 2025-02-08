#!/usr/bin/env python3
import socket
import time
import struct
import math
from threading import Thread


class SensorSimulator:
    def __init__(self, host='127.0.0.1', port=2000):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((host, port))
        self.server_socket.listen(1)
        self.running = True
        self.interval = 1000
        self.sending_data = False
        print("Sensor simulator started on port 2000")

    def start(self):
        while self.running:
            client_socket, addr = self.server_socket.accept()
            print(f"Connected to client at {addr}")
            self.handle_client(client_socket)

    def handle_client(self, client_socket):
        data_thread = None
        try:
            while self.running:
                data = client_socket.recv(1024).decode('ascii')
                if not data:
                    break
                
                print(f"Received command: {data.strip()}")
                
                if data.startswith('#03'):
                    self.process_start_command(data, client_socket, data_thread)
                elif data.startswith('#09'):
                    print("Stopping data stream")
                    self.sending_data = False
        
        except Exception as e:
            print(f"Error handling client: {e}")
        finally:
            self.sending_data = False
            client_socket.close()

    def process_start_command(self, data, client_socket, data_thread):
        hex_interval = data[3:7]
        self.interval = int(bytes.fromhex(hex_interval)[::-1].hex(), 16)
        print(f"Starting data stream with interval: {self.interval}ms")
        
        self.sending_data = True
        if data_thread is None or not data_thread.is_alive():
            data_thread = Thread(target=self.send_status_data, args=(client_socket,))
            data_thread.daemon = True
            data_thread.start()

    def send_status_data(self, client_socket):
        start_time = time.time()
        while self.sending_data:
            try:
                response = self.generate_sensor_data(start_time)
                client_socket.send(response.encode('ascii'))
                time.sleep(self.interval / 1000.0)
            except Exception as e:
                print(f"Error sending data: {e}")
                break

    def generate_sensor_data(self, start_time):
        t = time.time() - start_time
        supply_voltage = int(12000 + 100 * math.sin(t))
        env_temp = int(250 + 50 * math.sin(t * 0.5))
        yaw = int(1800 * math.sin(t))
        pitch = int(450 * math.sin(t * 0.7))
        roll = int(300 * math.sin(t * 0.3))
        
        message = struct.pack('<HHHHH',
                              supply_voltage & 0xFFFF,
                              env_temp & 0xFFFF,
                              yaw & 0xFFFF,
                              pitch & 0xFFFF,
                              roll & 0xFFFF)
        
        hex_data = ''.join([f'{b:02X}' for b in message])
        
        print(f"Sent: V={supply_voltage}mV, T={env_temp/10}°C, "
              f"Y={yaw/10}°, P={pitch/10}°, R={roll/10}°")
        
        return f'$11{hex_data}\r\n'

    def stop(self):
        self.running = False
        self.sending_data = False
        self.server_socket.close()


if __name__ == '__main__':
    simulator = SensorSimulator()
    try:
        simulator.start()
    except KeyboardInterrupt:
        print("\nShutting down simulator...")
        simulator.stop()
