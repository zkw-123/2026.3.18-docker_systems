#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
from serial import SerialException

class FaulhaberMotorController(Node):
    def __init__(self):
        super().__init__('faulhaber_motor_controller')
        
        # Load parameters from params.yaml or use default values
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('speed_scaling', 100.0)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.speed_scaling = self.get_parameter('speed_scaling').value

        # Log loaded parameters for debugging
        self.get_logger().info(f"Loaded parameters: serial_port={self.serial_port}, baud_rate={self.baud_rate}, speed_scaling={self.speed_scaling}")

        # Initialize serial connection
        try:
            self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to {self.serial_port} at {self.baud_rate} baud")
        except SerialException as e:
            self.get_logger().error(f"Failed to connect to {self.serial_port}: {e}")
            raise SystemExit

        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

    def cmd_vel_callback(self, msg):
        """Handle velocity control commands"""
        try:
            speed = int(msg.linear.x * self.speed_scaling)
            command = f'V{speed}\r\n'
            self.send_command(command)
        except ValueError as e:
            self.get_logger().error(f"Invalid speed value: {e}")

    def send_command(self, command):
        """Send command to the motor controller"""
        try:
            if self.serial_connection.is_open:
                self.serial_connection.write(command.encode())
                self.get_logger().info(f"Sent command: {command.strip()}")
            else:
                self.get_logger().warning("Serial connection is closed")
        except SerialException as e:
            self.get_logger().error(f"Failed to send command: {e}")

    def __del__(self):
        if hasattr(self, 'serial_connection') and self.serial_connection.is_open:
            self.serial_connection.close()
            self.get_logger().info("Serial connection closed")

def main(args=None):
    rclpy.init(args=args)
    node = FaulhaberMotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node")
    finally:
        node.destroy_node()
        rclpy.shutdown()
