import rclpy
from rclpy import qos
from rclpy.node import Node

import rclpy.time

from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatusArray, GoalStatus

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from scipy.spatial.transform import Rotation

class SetGoal(Node):
    def __init__(self):
        super().__init__('set_goal')

        self.map_frame = self.declare_parameter('map_frame', 'map').get_parameter_value().string_value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').get_parameter_value().string_value
        self.x = self.declare_parameter('x', 0.0).get_parameter_value().double_value
        self.y = self.declare_parameter('y', 0.0).get_parameter_value().double_value
        self.yaw = self.declare_parameter('yaw', 0.0).get_parameter_value().double_value # in radians

        qos_profile = qos.QoSProfile(
            reliability=qos.QoSReliabilityPolicy.RELIABLE,
            depth=1,
            durability=qos.QoSDurabilityPolicy.VOLATILE
        ) # same as bt_navigator
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', qos_profile)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.published = False
        self.create_timer(0.1, self.timer_cb)

        latched_qos = qos.QoSProfile(
            reliability=qos.QoSReliabilityPolicy.RELIABLE,
            depth=1,
            durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(GoalStatusArray, 'goal_status', self.status_cb, latched_qos)

    def timer_cb(self):
        if not self.tf_buffer.can_transform(self.map_frame, self.odom_frame, rclpy.time.Time()):
            self.get_logger().info(f'{self.map_frame} -> {self.odom_frame} transform does not exist - localisation might not be ready yet')
            return

        if self.goal_pub.get_subscription_count() == 0:
            self.get_logger().info('waiting for goal pose subscriber')
            return # no subscriptions - navigator not running yet?

        self.get_logger().info(f'sending out goal pose: (({self.x}, {self.y}), {self.yaw})')
        
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = Rotation.from_euler('z', self.yaw).as_quat()
        self.goal_pub.publish(msg)
        self.published = True

    def status_cb(self, data: GoalStatusArray):
        if not self.published or len(data.status_list) == 0: return # nothing to do

        latest_status: GoalStatus = data.status_list[-1] # newest goal is last
        self.get_logger().info(f'latest goal status: {latest_status.status}')
        if latest_status.status == 1 or latest_status.status == 2: # accepted/executing
            self.get_logger().info(f'goal started execution, exiting')
            raise SystemExit

def main():
    rclpy.init()
    node = SetGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass

    rclpy.shutdown()
