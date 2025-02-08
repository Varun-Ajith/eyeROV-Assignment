#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16, Int16
import socket
import struct
import threading
from interface.srv import StopSensor


class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        
        self.declare_parameter('interval', 1000)
        self.declare_parameter('sensor_ip', '127.0.0.1')
        self.declare_parameter('sensor_port', 2000)
        
        self.supply_voltage_pub = self.create_publisher(UInt16, '/sensor/supply_voltage', 10)
        self.env_temp_pub = self.create_publisher(Int16, '/sensor/env_temp', 10)
        self.yaw_pub = self.create_publisher(Int16, '/sensor/yaw', 10)
        self.pitch_pub = self.create_publisher(Int16, '/sensor/pitch', 10)
        self.roll_pub = self.create_publisher(Int16, '/sensor/roll', 10)
        
        self.sensor_socket = None
        self.connected = False
        self.running = True
        
        self.connect_to_sensor()
        
        self.stop_service = self.create_service(
            StopSensor,
            'stop_sensor',
            self.stop_sensor_callback
        )
    
    def connect_to_sensor(self):
        try:
            sensor_ip = self.get_parameter('sensor_ip').value
            sensor_port = self.get_parameter('sensor_port').value
            
            self.sensor_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sensor_socket.connect((sensor_ip, sensor_port))
            self.connected = True
            self.get_logger().info(f"Connected to sensor at {sensor_ip}:{sensor_port}")
            
            self.receive_thread = threading.Thread(target=self.receive_data)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            
            interval = self.get_parameter('interval').value
            self.send_start_command(interval)
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to sensor: {str(e)}')
            self.connected = False
    
    def send_start_command(self, interval):
        if not self.connected:
            return
        try:
            interval_bytes = struct.pack('<H', interval)
            interval_hex = ''.join([f'{b:02X}' for b in interval_bytes])
            command = f'#03{interval_hex}\r\n'
            
            self.sensor_socket.send(command.encode('ascii'))
            self.get_logger().info(f'Sent start command with interval {interval}ms')
            
        except Exception as e:
            self.get_logger().error(f'Failed to send start command: {str(e)}')
            self.connected = False
    
    def receive_data(self):
        buffer = ''
        while self.running and self.connected:
            try:
                data = self.sensor_socket.recv(1024).decode('ascii')
                if not data:
                    raise ConnectionError("Connection closed by sensor")
                
                buffer += data
                
                while '\r\n' in buffer:
                    message, buffer = buffer.split('\r\n', 1)
                    if message.startswith('$11'):
                        self.process_status_message(message[3:])
                        
            except Exception as e:
                self.get_logger().error(f'Error receiving data: {str(e)}')
                self.connected = False
                break
    
    def process_status_message(self, payload):
        try:
            if len(payload) != 20:
                self.get_logger().warn(f"Invalid payload length: {len(payload)}")
                return
            
            data_bytes = bytes.fromhex(payload)
            supply_voltage, env_temp, yaw, pitch, roll = struct.unpack('<HHHHH', data_bytes)
            
            env_temp = self.convert_to_signed_16(env_temp)
            yaw = self.convert_to_signed_16(yaw)
            pitch = self.convert_to_signed_16(pitch)
            roll = self.convert_to_signed_16(roll)
            
            self.supply_voltage_pub.publish(UInt16(data=supply_voltage))
            self.env_temp_pub.publish(Int16(data=env_temp))
            self.yaw_pub.publish(Int16(data=yaw))
            self.pitch_pub.publish(Int16(data=pitch))
            self.roll_pub.publish(Int16(data=roll))
            
            self.get_logger().info(
                f'Published: voltage={supply_voltage}mV, temp={env_temp/10}°C, '
                f'yaw={yaw/10}°, pitch={pitch/10}°, roll={roll/10}°'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error processing status message: {str(e)}')
    
    def convert_to_signed_16(self, value):
        return (value & 0x7FFF) - (value & 0x8000)
    
    def stop_sensor_callback(self, request, response):
        try:
            command = '#09\r\n'
            self.sensor_socket.send(command.encode('ascii'))
            self.get_logger().info("Sent stop command")
            response.success = True
        except Exception as e:
            self.get_logger().error(f'Failed to send stop command: {str(e)}')
            response.success = False
        return response
    
    def cleanup(self):
        self.running = False
        if self.sensor_socket:
            try:
                self.sensor_socket.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
