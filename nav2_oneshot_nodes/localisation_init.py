import rclpy
from rclpy import qos
from rclpy.node import Node

import rclpy.time
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped, PoseWithCovariance
from nav2_msgs.srv import ClearEntireCostmap
from lifecycle_msgs.srv import GetState

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from scipy.spatial.transform import Rotation

from threading import Event
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

import time

class LocalisationInit(Node):
    def __init__(self):
        super().__init__('localisation_init')

        self.map_frame = self.declare_parameter('map_frame', 'map').get_parameter_value().string_value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_link').get_parameter_value().string_value

        self.callback_group = ReentrantCallbackGroup()
        clear_costmaps = self.declare_parameter('clear_costmaps', False).get_parameter_value().bool_value
        self.clear_costmap_srvs = dict()
        if clear_costmaps:
            self.clear_costmap_srvs = {
                node: self.create_client(ClearEntireCostmap, f'/{node}/clear_entirely_{node}')
                for node in ['local_costmap', 'global_costmap']
            }
        
        self.x = self.declare_parameter('x', 0.0).get_parameter_value().double_value
        self.y = self.declare_parameter('y', 0.0).get_parameter_value().double_value
        self.yaw = self.declare_parameter('yaw', 0.0).get_parameter_value().double_value # in radians
        self.pose = Pose()
        self.pose.position.x = self.x
        self.pose.position.y = self.y
        self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w = Rotation.from_euler('z', self.yaw).as_quat()

        self.pos_err = self.declare_parameter('pos_err', 0.05).get_parameter_value().double_value
        self.yaw_err = self.declare_parameter('yaw_err', 0.1).get_parameter_value().double_value

        self.initpose_pub = self.create_publisher(PoseStamped, 'init_pose/nocov', qos.qos_profile_system_default)
        self.initpose_cov_pub = self.create_publisher(PoseWithCovarianceStamped, 'init_pose/cov', qos.qos_profile_system_default)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        localiser = self.declare_parameter('localiser', 'amcl').get_parameter_value().string_value
        localiser_state_srv = self.create_client(GetState, f'/{localiser}/get_state')
        while not localiser_state_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'waiting for localiser node {localiser} to show up')
        while True:
            future = localiser_state_srv.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future) # we can ONLY do this outside of callbacks!
            if future.result() is None:
                self.get_logger().error(f'localiser node {localiser} returned null for state retrieval request')
            else:
                state = future.result().current_state.label
                self.get_logger().info(f'localiser node {localiser} status: {state}')
                if state == 'active': break
            time.sleep(0.5)

        self.publish_pose()
        
        self.create_timer(0.5, self.timer_cb, callback_group=self.callback_group)

    def publish_pose(self):
        self.get_logger().info(f'publishing initial pose')
        header = Header(
            stamp = self.get_clock().now().to_msg(),
            frame_id = self.map_frame
        )
        self.initpose_pub.publish(PoseStamped(
            header = header,
            pose = self.pose
        ))
        self.initpose_cov_pub.publish(PoseWithCovarianceStamped(
            header = header,
            pose = PoseWithCovariance(
                pose = self.pose,
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

    def timer_cb(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time())
        except TransformException:
            self.get_logger().warn(f'cannot obtain {self.map_frame} -> {self.robot_frame} transform')
            return
        
        # verify transform is within error margin
        resend_pose = False
        if abs(tf.transform.translation.x - self.x) > self.pos_err or abs(tf.transform.translation.y - self.y) > self.pos_err:
            self.get_logger().info(f'current position ({tf.transform.translation.x}, {tf.transform.translation.y}) is off from desired position ({self.x}, {self.y})')
            resend_pose = True
        else:
            _, _, d_yaw = (Rotation.from_euler('z', -self.yaw) * Rotation.from_quat([
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w
            ])).as_euler('xyz')
            if abs(d_yaw) > self.yaw_err:
                self.get_logger().info(f'current yaw {d_yaw} is off from desired yaw {self.yaw}')
                resend_pose = True
        
        if resend_pose:
            self.publish_pose()
        else:
            self.get_logger().info(f'current pose is within error margin')

            if len(self.clear_costmap_srvs) > 0: # clear costmaps
                event = Event()
                def done_cb(future):
                    nonlocal event
                    event.set()

                for node in self.clear_costmap_srvs:
                    srv = self.clear_costmap_srvs[node]
                    while not srv.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info(f'waiting until costmap node {node} is available')
                    self.get_logger().info(f'clearing costmap of node {node}')
                    srv.call_async(ClearEntireCostmap.Request()).add_done_callback(done_cb)
                    event.wait()
                    event.clear() # done!

            rclpy.shutdown() # raise SystemExit doesn't work here

def main():
    rclpy.init()
    node = LocalisationInit()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
