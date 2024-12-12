import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import Odometry, Path
from .include.ExtendedKalmanFilterUtils import ExtendedKalmanFilterUtils
from sensor_msgs.msg import JointState
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np


linear_vel , angular_vel = Float32(), Float32()
encoder_data_x, encoder_data_y, encoder_data_theta = Float32(), Float32(), Float32()


class ExtendedKalmanFilter(Node):
    def __init__(self):
        super().__init__("Extended_Kalman_Filter")
        self.cb_group     = ReentrantCallbackGroup()

        self.x_      = np.asarray([0.0, 0.0, 0.0,])  # Predicted state
        self.x_t_1   = np.asarray([0.0, 0.0, 0.0,])  # Corrected previous state
        self.cov_t_1 = np.eye(3)                     # Initial estimation for covariance matrix   
        self.C       = np.eye(3)                     # Measurement model matrix for both Wheel encoders and IMU 

        self.x_t_visualize   = np.asarray([0.0, 0.0, 0.0,])
        self.cov_t_visualize = np.asarray([0.0, 0.0, 0.0,])

        self.predicted_path_pub = self.create_publisher(Float32MultiArray, "/ekf/predicted_state", 10)
        self.corrected_path_pub = self.create_publisher(Float32MultiArray, "/ekf/corrected_state", 10)

        self.control_input_sub_ = self.create_subscription(TwistStamped, "/turtlebot_controller/cmd_vel", self.control_callback, 10, callback_group=self.cb_group)
        self.ekf_odom_pub_      = self.create_publisher(Odometry, "turtlebot_ekf/odom_ekf", 10)
        self.timer_period_      = 0.1
        self.timer_             = self.create_timer(self.timer_period_, self.timer_callback)
        self.ekf_utils          = ExtendedKalmanFilterUtils(self)

        self.get_logger().info("Starting the Extended Kalman Filter...")


    def control_callback(self, msg):
        global linear_vel , angular_vel

        linear_vel.data  = msg.twist.linear.x
        angular_vel.data = msg.twist.angular.z 

        
    def timer_callback(self):
        global linear_vel, angular_vel
        global encoder_data_x, encoder_data_y, encoder_data_theta

        u_t   = np.array([linear_vel.data, angular_vel.data])  # Current control input
        dt    = self.timer_period_                             # Time period 

        ########################################## State Prediction Only #############################################################################

        x_t_visualize, cov_t_visualize = self.ekf_utils.prediction(self.x_t_visualize, self.cov_t_visualize, u_t, dt)
        self.x_t_visualize = x_t_visualize
        self.cov_t_visualize = cov_t_visualize

        ##############################################################################################################################################

        ########################################## State Prediction, Correction ######################################################################
        
        x_t_, cov_t_ = self.ekf_utils.prediction(self.x_t_1, self.cov_t_1, u_t, dt)

        z = np.asarray([encoder_data_x.data, encoder_data_y.data, encoder_data_theta.data]).T   
        C = self.C
        K = self.ekf_utils.kalman_gain(cov_t_, C)  

        x_t, cov_t = self.ekf_utils.correction(x_t_, z, C, cov_t_, K)

        ##############################################################################################################################################

        self.x_t_1 = x_t
        self.cov_t_1 = cov_t

        predicted_msg = Float32MultiArray()
        predicted_msg.data = x_t_visualize.tolist()  
        self.predicted_path_pub.publish(predicted_msg)

        corrected_msg = Float32MultiArray()
        corrected_msg.data = x_t.tolist()  
        self.corrected_path_pub.publish(corrected_msg)


class EncoderReadingPublisher(Node):
    def __init__(self):
        super().__init__("Encoder_Reading_Publisher")
        self.cb_group     = ReentrantCallbackGroup()

        self.declare_parameter("wheel_radius", 0.036)
        self.declare_parameter("wheel_separation", 0.19)

        self.wheel_radius     = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.left_motor_pos_init  = 0.0
        self.right_motor_pos_init = 0.0
        self.x     = 0.0
        self.y     = 0.0 
        self.theta = 0.0

        self.sub_ = self.create_subscription(JointState, "/joint_states", self.joint_callback, 10, callback_group=self.cb_group)


    def joint_callback(self, msg):

        global encoder_data_x, encoder_data_y, encoder_data_theta

        wheel_encoder_left  = msg.position[1]
        wheel_encoder_right = msg.position[0]

        delta_left  = wheel_encoder_left  - self.left_motor_pos_init
        delta_right = wheel_encoder_right - self.right_motor_pos_init
    
        linear_delta_pos    = (self.wheel_radius * delta_left + self.wheel_radius * delta_right) / 2
        angular_delta_theta = (self.wheel_radius * delta_right - self.wheel_radius * delta_left) / self.wheel_separation
        self.theta         += angular_delta_theta
        self.x             += linear_delta_pos * np.cos(self.theta)
        self.y             += linear_delta_pos * np.sin(self.theta)

        encoder_data_x.data     = self.x
        encoder_data_y.data     = self.y
        encoder_data_theta.data = self.theta
    
        self.left_motor_pos_init = msg.position[1]
        self.right_motor_pos_init = msg.position[0]



def main(args=None):
    rclpy.init(args=args)

    extended_kalman_filter    = ExtendedKalmanFilter()
    encoder_reading_publisher = EncoderReadingPublisher()

    executor  = MultiThreadedExecutor()
    executor.add_node(extended_kalman_filter)
    executor.add_node(encoder_reading_publisher)

    executor.spin()

if __name__ == "__main__":
    main()


