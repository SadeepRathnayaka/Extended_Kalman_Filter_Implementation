import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import JointState
import numpy as np

class EncoderReadingPublisher(Node):
    def __init__(self):
        super().__init__("Encoder_Reading_Publisher")

        self.declare_parameter("wheel_radius", 0.036)
        self.declare_parameter("wheel_separation", 0.19)

        self.wheel_radius     = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.left_motor_pos_init  = 0.0
        self.right_motor_pos_init = 0.0
        self.x     = 0.0
        self.y     = 0.0 
        self.theta = 0.0

        self.sub_ = self.create_subscription(JointState, "/joint_states", self.joint_callback, 10)
        self.pub_ = self.create_publisher(Float32MultiArray, "/encoders/measurments", 10)


    def joint_callback(self, msg):

        wheel_encoder_left  = msg.position[1]
        wheel_encoder_right = msg.position[0]

        delta_left  = wheel_encoder_left  - self.left_motor_pos_init
        delta_right = wheel_encoder_right - self.right_motor_pos_init
    
        linear_delta_pos    = (self.wheel_radius * delta_left + self.wheel_radius * delta_right) / 2
        angular_delta_theta = (self.wheel_radius * delta_right - self.wheel_radius * delta_left) / self.wheel_separation
        self.theta         += angular_delta_theta
        self.x             += linear_delta_pos * np.cos(self.theta)
        self.y             += linear_delta_pos * np.sin(self.theta)

        current_state = Float32MultiArray()
        current_state.data.append(self.x)
        current_state.data.append(self.y)
        current_state.data.append(self.theta)
        self.pub_.publish(current_state)
    
        self.left_motor_pos_init = msg.position[1]
        self.right_motor_pos_init = msg.position[0]


def main(args=None):
    rclpy.init(args=args)

    encoder_readings_publisher = EncoderReadingPublisher()

    try:
        rclpy.spin(encoder_readings_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        encoder_readings_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()