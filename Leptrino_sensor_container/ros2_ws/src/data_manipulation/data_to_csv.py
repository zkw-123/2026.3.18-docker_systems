import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped 
import csv
import time
import sys

class ForceTorqueLogger(Node):
    def __init__(self, filename):
        super().__init__('force_torque_logger')
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/force_torque',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Open CSV file
        self.csv_file = open(filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # Write CSV header
        self.csv_writer.writerow(['Index', 'Timestamp', 'Force_X', 'Force_Y', 'Force_Z', 'Torque_X', 'Torque_Y', 'Torque_Z'])

        # Initialize variables
        self.start_time = time.time()
        self.index = 0

    def listener_callback(self, msg):
        current_time = time.time()
        # Stop after 10 seconds
        if current_time - self.start_time > 100:
            self.csv_file.close()
            rclpy.shutdown()
            return

        # Extract data from message
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        force_x = msg.wrench.force.x
        force_y = msg.wrench.force.y
        force_z = msg.wrench.force.z
        torque_x = msg.wrench.torque.x
        torque_y = msg.wrench.torque.y
        torque_z = msg.wrench.torque.z

        # Write data to CSV
        self.csv_writer.writerow([self.index, timestamp, force_x, force_y, force_z, torque_x, torque_y, torque_z])
        self.index += 1

        # Log the data
        self.get_logger().info(f'Recorded data point {self.index}')

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) > 2:
        rs = sys.argv[1]
        ps = sys.argv[2]
        filename = f'RS{rs}_PS{ps}.csv'
    else:
        filename = 'force_torque_data.csv'  # Default filename
    force_torque_logger = ForceTorqueLogger(filename)
    rclpy.spin(force_torque_logger)

if __name__ == '__main__':
    main()




