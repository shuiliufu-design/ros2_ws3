#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import time

class BaseDriver(Node):
    def __init__(self):
        super().__init__('base_driver')
        # Params
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value

        self.ser = None
        self.connect_serial()

        self.sub = self.create_subscription(Int32MultiArray, 'wheel_targets', self.targets_cb, 10)
        self.pub = self.create_publisher(Int32MultiArray, 'wheel_ticks', 10)
        
        self.timer = self.create_timer(0.05, self.update) # 20Hz

    def connect_serial(self):
        self.get_logger().info(f'Attempting to connect to {self.port} at {self.baud}...')
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2.0) # Wait for Arduino reset
            self.get_logger().info(f'Successfully connected to {self.port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to {self.port}: {e}')
            self.ser = None

    def targets_cb(self, msg):
        if not self.ser:
            self.get_logger().warn('Cannot send targets: Serial not connected', throttle_duration_sec=2.0)
            return
            
        # Format: m <t1>:<t2>:<t3>:<t4>
        if len(msg.data) == 4:
            cmd = f'm {msg.data[0]}:{msg.data[1]}:{msg.data[2]}:{msg.data[3]}\r'
            # self.get_logger().info(f'Sending: {cmd.strip()}')
            try:
                self.ser.write(cmd.encode())
            except Exception as e:
                self.get_logger().error(f'Serial write error: {e}')
                self.ser = None 

    def update(self):
        if not self.ser: 
            self.get_logger().warn('Serial not connected, retrying...', throttle_duration_sec=5.0)
            self.connect_serial()
            return

        # Read Encoders
        try:
            # Clear buffer before write to reduce lag
            self.ser.reset_input_buffer()
            self.ser.write(b'e\r')
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            
            # Debug: Log raw line (throttled to avoid spam)
            # if line:
            #    self.get_logger().info(f'Raw Serial: {line}', throttle_duration_sec=2.0)

            # Expecting "e1 e2 e3 e4" (space separated)
            if not line: return
            
            parts = line.split()
            if len(parts) == 4:
                try:
                    ticks = [int(p) for p in parts]
                    msg = Int32MultiArray(data=ticks)
                    self.pub.publish(msg)
                except ValueError:
                    self.get_logger().warn(f'Parse error, parts: {parts}')
            else:
                 self.get_logger().warn(f'Unexpected format: "{line}"')

        except Exception as e:
            self.get_logger().error(f'Serial read error: {e}')
            self.ser = None

def main(args=None):
    rclpy.init(args=args)
    node = BaseDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
