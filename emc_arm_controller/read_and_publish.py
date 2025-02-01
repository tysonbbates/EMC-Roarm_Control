#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
import serial

# Global float variables
float1 = 0.0
float2 = 1.0
float3 = 2.0
float4 = 3.0

class SerialReader:
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200, timeout=0.1):
        """
        Initialize the serial reader.

        :param port: Serial port to connect to (default: /dev/ttyUSB1)
        :param baudrate: Baud rate for the serial connection (default: 115200)
        :param timeout: Read timeout in seconds (default: 0.1)
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = serial.Serial(port, baudrate, timeout=timeout)

    def read_and_parse(self):
        """
        Reads a line from the serial port, parses it, and updates the global float values.
        The line should be in the format: float1,float2,float3,float4
        """
        global float1, float2, float3, float4  # Declare global variables

        try:
            # Read a line from the serial port
            line = self.serial_conn.readline().decode('utf-8').strip()

            # Skip empty lines
            if not line:
                return

            # Split the line by commas
            parts = line.split(',')

            # Ensure there are exactly 4 parts
            if len(parts) == 4:
                # Convert each part to a float and update the global variables
                float1 = float(parts[0])
                float2 = float(parts[1])
                float3 = float(parts[2])
                float4 = float(parts[3])

            else:
                print(f"Unexpected data format: {line}")

        except ValueError as e:
            print(f"Error parsing data: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

    def close(self):
        """
        Closes the serial connection.
        """
        if self.serial_conn.is_open:
            self.serial_conn.close()

reader = SerialReader()

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_state)  # Publish every 0.1 seconds
    
    def publish_joint_state(self):
        global float1, float2, float3, float4  # Use global variables

        # Read and update global values from serial
        reader.read_and_parse()

        # Create the JointState message
        msg = JointState()

        # Populate the header with the current time
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""

        # Joint names
        msg.name = ["base_link_to_link1", "link1_to_link2", "link2_to_link3", "link3_to_gripper_link"]

        # Joint positions, velocities, efforts (all same length as name array)
        msg.position = [float1, float2, float3, float4]  # ✅ Corrected
        msg.velocity = []  # Empty list (optional field)
        msg.effort = []    # Empty list (optional field)

        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info(f"Published JointState: {msg.position}")

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()