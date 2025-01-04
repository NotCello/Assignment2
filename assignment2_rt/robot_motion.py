import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

LINEAR_THR = 5.0

class Robot(Node):
    def __init__(self):
        super().__init__('move_robot_node')
        # Publish the new velocity on the /cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.velocity = Twist()

    def move_robot(self, x, y, z):
        self.velocity.linear.x = x
        self.velocity.linear.y = y
        self.velocity.angular.z = z
        self.publisher_.publish(self.velocity)
        self.get_logger().info(f'Moving robot: {self.velocity.linear.x} forward and {self.velocity.angular.z} angular')

def main(args=None):
    rclpy.init(args=args)
    robot = Robot()

    try:
        while rclpy.ok():
            # Ask the user to insert the values for linear velocity and angular
            try:
                x = float(input("Enter the linear velocity x: "))
                x = min(x, LINEAR_THR)
                x = max(x, -LINEAR_THR)
                #y = float(input("Enter the linear velocity y: ")) # Not supported here
                z = float(input("Enter angular velocity z: "))
            except ValueError:
                print("Invalid input. Please enter numeric values.")
                continue 

            robot.move_robot(x, 0.0, z)
            # Waits 3 seconds before stopping the robot and asks the user for the new velocity again
            time.sleep(3)
            # Stop the robot by publishing a zero velocity
            robot.move_robot(0.0, 0.0, 0.0)
    except KeyboardInterrupt:
        # Handle the interrupt (Ctrl+C)
        print("Program interrupted")
    except Exception as e:
        # Handle other exception
        print(f"An error occurred: {e}")
    finally:
        robot.destroy_node()
        #rclpy.shutdown()

if __name__ == '__main__':
    main()