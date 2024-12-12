import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class PredictedPathSub(Node):
    def __init__(self):
        super().__init__("Predicted_Path_Subscriber")
        self.cb_group     = ReentrantCallbackGroup()

        self.predicted_path_sub = self.create_subscription(Float32MultiArray, "/ekf/predicted_state", self.callback, 10, callback_group=self.cb_group)
        self.predicted_path_pub = self.create_publisher(Path, "/ekf/predicted_path", 10)

        self.predicted_path = Path()
        self.predicted_path.header.frame_id = "odom"

        self.br_ = TransformBroadcaster(self)
        self.transfrom_stamped_ = TransformStamped()
        self.transfrom_stamped_.header.frame_id = "odom"
        self.transfrom_stamped_.child_frame_id = "ekf_prediction"


    def callback(self, msg):
        x_t_ = msg.data

        pred_q  = quaternion_from_euler(0, 0, x_t_[2])

        self.transfrom_stamped_.header.stamp  = self.get_clock().now().to_msg()
        self.transfrom_stamped_.transform.translation.x = x_t_[0]
        self.transfrom_stamped_.transform.translation.y = x_t_[1]
        self.transfrom_stamped_.transform.rotation.x    = pred_q[0]
        self.transfrom_stamped_.transform.rotation.y    = pred_q[1]
        self.transfrom_stamped_.transform.rotation.z    = pred_q[2]
        self.transfrom_stamped_.transform.rotation.w    = pred_q[3]
        self.br_.sendTransform(self.transfrom_stamped_)

        pred_pose = PoseStamped()
        pred_pose.header.stamp = self.get_clock().now().to_msg()
        pred_pose.pose.position.x = x_t_[0]
        pred_pose.pose.position.y = x_t_[1]
        pred_pose.pose.orientation.x = pred_q[0]
        pred_pose.pose.orientation.y = pred_q[1]
        pred_pose.pose.orientation.z = pred_q[2]
        pred_pose.pose.orientation.w = pred_q[3]

        self.predicted_path.header.stamp = self.get_clock().now().to_msg()
        self.predicted_path.poses.append(pred_pose)
        self.predicted_path_pub.publish(self.predicted_path)
        

class CorrectedPathSub(Node):
    def __init__(self):
        super().__init__("Corrected_Path_Subscriber")
        self.cb_group     = ReentrantCallbackGroup()

        self.corrected_path_sub = self.create_subscription(Float32MultiArray, "/ekf/corrected_state", self.callback, 10, callback_group=self.cb_group)
        self.corrected_path_pub = self.create_publisher(Path, "/ekf/corrected_path", 10)

        self.corrected_path = Path()
        self.corrected_path.header.frame_id = "odom"

        self.br_ = TransformBroadcaster(self)
        self.transfrom_stamped = TransformStamped()
        self.transfrom_stamped.header.frame_id = "odom"
        self.transfrom_stamped.child_frame_id = "ekf_correction"


    def callback(self, msg) :

        x_t = msg.data

        corr_q = quaternion_from_euler(0, 0, x_t[2])

        self.transfrom_stamped.header.stamp  = self.get_clock().now().to_msg()
        self.transfrom_stamped.transform.translation.x = x_t[0]
        self.transfrom_stamped.transform.translation.y = x_t[1]
        self.transfrom_stamped.transform.rotation.x    = corr_q[0]
        self.transfrom_stamped.transform.rotation.y    = corr_q[1]
        self.transfrom_stamped.transform.rotation.z    = corr_q[2]
        self.transfrom_stamped.transform.rotation.w    = corr_q[3]
        self.br_.sendTransform(self.transfrom_stamped)

        corr_pose = PoseStamped()
        corr_pose.header.stamp = self.get_clock().now().to_msg()
        corr_pose.pose.position.x = x_t[0]
        corr_pose.pose.position.y = x_t[1]
        corr_pose.pose.orientation.x = corr_q[0]
        corr_pose.pose.orientation.y = corr_q[1]
        corr_pose.pose.orientation.z = corr_q[2]
        corr_pose.pose.orientation.w = corr_q[3]

        self.corrected_path.header.stamp = self.get_clock().now().to_msg()
        self.corrected_path.poses.append(corr_pose)
        self.corrected_path_pub.publish(self.corrected_path)


class OdomPathSub(Node):
    def __init__(self):
        super().__init__("Odom_Path_Subscriber")
        self.cb_group     = ReentrantCallbackGroup()

        self.odom_path_sub = self.create_subscription(Odometry, "/turtlebot_controller/odom", self.callback, 10, callback_group=self.cb_group)
        self.real_path_pub = self.create_publisher(Path, "/ekf/real_path", 10)

        self.real_path = Path()
        self.real_path.header.frame_id = "odom"


    def callback(self, msg):

        real_pose = PoseStamped()
        real_pose.header.stamp = self.get_clock().now().to_msg()
        real_pose.pose = msg.pose.pose

        self.real_path.header.stamp = self.get_clock().now().to_msg()
        self.real_path.poses.append(real_pose)        
        self.real_path_pub.publish(self.real_path)


def main(args=None):
    rclpy.init(args=args)

    predicted_path_sub = PredictedPathSub()
    corrected_path_sub = CorrectedPathSub()
    real_path_sub      = OdomPathSub()

    executor  = MultiThreadedExecutor()
    executor.add_node(predicted_path_sub)
    executor.add_node(corrected_path_sub)
    executor.add_node(real_path_sub)

    executor.spin()

if __name__ == "__main__":
    main()