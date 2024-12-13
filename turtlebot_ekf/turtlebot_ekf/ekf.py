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
GPS_data_x , GPS_data_y = Float32(), Float32()


class ExtendedKalmanFilter(Node):
    def __init__(self):
        super().__init__("Extended_Kalman_Filter")
        self.cb_group = ReentrantCallbackGroup()

        self.x_      = np.asarray([0.0, 0.0, 0.0,])  # Predicted state
        self.x_t_1   = np.asarray([0.0, 0.0, 0.0,])  # Initial estimated state
        self.cov_t_1 = np.eye(3,3)                  # Initial estimation for covariance matrix  
        # self.cov_t_1[0,0] = 1e-9 
        # self.cov_t_1[1,1] = 1e-9
        # self.cov_t_1[2,2] = 1e-9
        
        self.C          = np.eye(3)                  # Measurement model matrix for Wheel encoders
        self.C_GPS      = np.zeros((2,3))            # Measurement model matrix for GPS sensor
        self.C_GPS[0,0] = 1 
        self.C_GPS[1,1] = 1

        self.x_t_predicted   = np.asarray([0.0, 0.0, 0.0,])
        self.cov_t_visualize = np.asarray([0.0, 0.0, 0.0,])

        self.predicted_path_pub = self.create_publisher(Float32MultiArray, "/ekf/predicted_state", 10)
        self.corrected_path_pub = self.create_publisher(Float32MultiArray, "/ekf/corrected_state", 10)
        self.fused_path_pub     = self.create_publisher(Float32MultiArray, "/ekf/fused_state", 10)

        self.timer_period_      = 0.1
        self.timer_             = self.create_timer(self.timer_period_, self.timer_callback)
        self.ekf_utils          = ExtendedKalmanFilterUtils(self)

        self.get_logger().info("Starting the Extended Kalman Filter...")


    def timer_callback(self):
        global linear_vel, angular_vel
        global encoder_data_x, encoder_data_y, encoder_data_theta
        global GPS_data_x, GPS_data_y

        u_t   = np.array([linear_vel.data, angular_vel.data])  # Current control input
        dt    = self.timer_period_                             # Time period 

        ########################################## State Prediction Only #############################################################################

        x_t_predicted, _ = self.ekf_utils.prediction(self.x_t_predicted, self.cov_t_visualize, u_t, dt)
        self.x_t_predicted = x_t_predicted

        ########################################## State Prediction, Correction ######################################################################
        
        x_t_, cov_t_ = self.ekf_utils.prediction(self.x_t_1, self.cov_t_1, u_t, dt)

        z_encoder = np.asarray([encoder_data_x.data, encoder_data_y.data, encoder_data_theta.data]).T   
        C_encoder = self.C
        K_encoder = self.ekf_utils.encoder_kalman_gain(cov_t_, C_encoder)  

        x_t, cov_t = self.ekf_utils.correction(x_t_, z_encoder, C_encoder, cov_t_, K_encoder)
        corrected_state = x_t

        ########################################## GPS Sensor Fusion #################################################################################
        
        z_GPS = np.asarray([GPS_data_x.data, GPS_data_y.data]).T   
        C_GPS = self.C_GPS
        K_GPS = self.ekf_utils.GPS_kalman_gain(cov_t, C_GPS)  

        x_t, cov_t = self.ekf_utils.correction(x_t, z_GPS, C_GPS, cov_t, K_GPS)
        fused_state = x_t

        ##############################################################################################################################################

        self.x_t_1 = x_t
        self.cov_t_1 = cov_t

        predicted_msg = Float32MultiArray()
        predicted_msg.data = x_t_predicted.tolist()  
        self.predicted_path_pub.publish(predicted_msg)

        corrected_msg = Float32MultiArray()
        corrected_msg.data = corrected_state.tolist()  
        self.corrected_path_pub.publish(corrected_msg)

        fused_msg = Float32MultiArray()
        fused_msg.data = fused_state.tolist()  
        self.fused_path_pub.publish(fused_msg)



class ControlInputPublisher(Node):
    def __init__(self):
        super().__init__("Control_Input_Publsiher")
        self.cb_group = ReentrantCallbackGroup()

        self.control_input_sub_ = self.create_subscription(TwistStamped, "/turtlebot_controller/cmd_vel", self.control_callback, 10, callback_group=self.cb_group)

    def control_callback(self, msg):
        global linear_vel , angular_vel

        linear_vel.data  = msg.twist.linear.x
        angular_vel.data = msg.twist.angular.z 



class EncoderReadingPublisher(Node):
    def __init__(self):
        super().__init__("Encoder_Reading_Publisher")
        self.cb_group = ReentrantCallbackGroup()

        self.sub_ = self.create_subscription(Float32MultiArray, "/encoders/measurments", self.callback, 10, callback_group=self.cb_group)

    def callback(self, msg):
        global encoder_data_x, encoder_data_y, encoder_data_theta

        current_coordinates = msg.data
        encoder_data_x.data     = current_coordinates[0]
        encoder_data_y.data     = current_coordinates[1]
        encoder_data_theta.data = current_coordinates[2]



class GPSReadingPublisher(Node):
    def __init__(self):
        super().__init__("GPS_Reading_Publisher")
        self.cb_group = ReentrantCallbackGroup()

        self.sub_ = self.create_subscription(Float32MultiArray, "/gps/measurments", self.callback, 10, callback_group=self.cb_group)

    def callback(self, msg):
        global GPS_data_x, GPS_data_y

        current_coordinates = msg.data
        GPS_data_x.data     = current_coordinates[0]
        GPS_data_y.data     = current_coordinates[1]


def main(args=None):
    rclpy.init(args=args)

    extended_kalman_filter    = ExtendedKalmanFilter()
    control_input_filter      = ControlInputPublisher()
    encoder_reading_publisher = EncoderReadingPublisher()
    gps_reading_publisher     = GPSReadingPublisher()

    executor  = MultiThreadedExecutor()
    executor.add_node(extended_kalman_filter)
    executor.add_node(control_input_filter)
    executor.add_node(encoder_reading_publisher)
    executor.add_node(gps_reading_publisher)

    executor.spin()

if __name__ == "__main__":
    main()


