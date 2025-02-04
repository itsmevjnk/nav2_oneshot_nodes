import rclpy
from rclpy import qos
from rclpy.node import Node

import rclpy.time
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped, PoseWithCovariance

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from scipy.spatial.transform import Rotation

class LocalisationInit(Node):
    def __init__(self):
        super().__init__('localisation_init')

        self.map_frame = self.declare_parameter('map_frame', 'map').get_parameter_value().string_value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').get_parameter_value().string_value
        self.x = self.declare_parameter('x', 0.0).get_parameter_value().double_value
        self.y = self.declare_parameter('y', 0.0).get_parameter_value().double_value
        self.yaw = self.declare_parameter('yaw', 0.0).get_parameter_value().double_value # in radians

        self.initpose_pub = self.create_publisher(PoseStamped, 'init_pose/nocov', qos.qos_profile_system_default)
        self.initpose_cov_pub = self.create_publisher(PoseWithCovarianceStamped, 'init_pose/cov', qos.qos_profile_system_default)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(0.1, self.timer_cb)

    def timer_cb(self):
        if self.tf_buffer.can_transform(self.map_frame, self.odom_frame, rclpy.time.Time()):
            self.get_logger().info(f'{self.map_frame} -> {self.odom_frame} transform exists - exiting')
            raise SystemExit

        if self.initpose_pub.get_subscription_count() == 0 and self.initpose_cov_pub.get_subscription_count() == 0:
            self.get_logger().info('waiting for initial pose subscriber')
            return # no subscriptions - AMCL not running yet?

        self.get_logger().info(f'sending out initial pose: (({self.x}, {self.y}), {self.yaw})')
        
        header = Header(
            stamp = self.get_clock().now().to_msg(),
            frame_id = self.map_frame
        )

        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = Rotation.from_euler('z', self.yaw).as_quat()

        self.initpose_pub.publish(PoseStamped(
            header = header,
            pose = pose
        ))
        
        self.initpose_cov_pub.publish(PoseWithCovarianceStamped(
            header = header,
            pose = PoseWithCovariance(
                pose = pose,
                covariance = [
                    0.25, 0.0 , 0.0 , 0.0 , 0.0 , 0.0 ,
                    0.0 , 0.25, 0.0 , 0.0 , 0.0 , 0.0 ,
                    0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 ,
                    0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 ,
                    0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 ,
                    0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.06853891909122467 # from rviz
                ]
            )
        ))

def main():
    rclpy.init()
    node = LocalisationInit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass

    rclpy.shutdown()
