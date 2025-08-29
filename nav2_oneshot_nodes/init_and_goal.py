import rclpy
from rclpy import qos
from rclpy.node import Node

import rclpy.time
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped, PoseWithCovariance
from action_msgs.msg import GoalStatusArray, GoalStatus
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

class InitAndGoal(Node):
    def __init__(self):
        super().__init__('init_and_goal')

        self.map_frame = self.declare_parameter('map_frame', 'map').get_parameter_value().string_value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_link').get_parameter_value().string_value
        
        self.init_x = self.declare_parameter('init_x', 0.0).get_parameter_value().double_value
        self.init_y = self.declare_parameter('init_y', 0.0).get_parameter_value().double_value
        self.init_yaw = self.declare_parameter('init_yaw', 0.0).get_parameter_value().double_value # in radians
        self.init_pose = Pose()
        self.init_pose.position.x = self.init_x
        self.init_pose.position.y = self.init_y
        self.init_pose.orientation.x, self.init_pose.orientation.y, self.init_pose.orientation.z, self.init_pose.orientation.w = Rotation.from_euler('z', self.init_yaw).as_quat()

        clear_costmaps = self.declare_parameter('clear_costmaps', False).get_parameter_value().bool_value
        self.clear_costmap_srvs = dict()
        if clear_costmaps:
            self.clear_costmap_srvs = {
                node: self.create_client(ClearEntireCostmap, f'{node}/clear_entirely_{node}')
                for node in ['local_costmap', 'global_costmap']
            }

        self.init_pos_err = self.declare_parameter('init_pos_err', 0.05).get_parameter_value().double_value
        self.init_yaw_err = self.declare_parameter('init_yaw_err', 0.1).get_parameter_value().double_value

        self.initpose_pub = self.create_publisher(PoseStamped, 'init_pose/nocov', qos.qos_profile_system_default)
        self.initpose_cov_pub = self.create_publisher(PoseWithCovarianceStamped, 'init_pose/cov', qos.qos_profile_system_default)

        self.goal_x = self.declare_parameter('goal_x', 0.0).get_parameter_value().double_value
        self.goal_y = self.declare_parameter('goal_y', 0.0).get_parameter_value().double_value
        self.goal_yaw = self.declare_parameter('goal_yaw', 0.0).get_parameter_value().double_value # in radians

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        localiser = self.declare_parameter('localiser', 'amcl').get_parameter_value().string_value
        localiser_state_srv = self.create_client(GetState, f'{localiser}/get_state')
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
        
        self.navigator = self.declare_parameter('navigator', 'bt_navigator').get_parameter_value().string_value
        self.navigator_state_srv = self.create_client(GetState, f'{self.navigator}/get_state')
        while not self.navigator_state_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'waiting for navigator node {self.navigator} to show up')
        
        qos_profile = qos.QoSProfile(
            reliability=qos.QoSReliabilityPolicy.RELIABLE,
            depth=1,
            durability=qos.QoSDurabilityPolicy.VOLATILE
        ) # same as bt_navigator
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', qos_profile)

        latched_qos = qos.QoSProfile(
            reliability=qos.QoSReliabilityPolicy.RELIABLE,
            depth=1,
            durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(GoalStatusArray, 'goal_status', self.status_cb, latched_qos)

        self.goal_published = False
        self.callback_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(0.5, self.init_timer_cb, callback_group=self.callback_group)

    def publish_pose(self):
        self.get_logger().info(f'publishing initial pose')
        header = Header(
            stamp = self.get_clock().now().to_msg(),
            frame_id = self.map_frame
        )
        self.initpose_pub.publish(PoseStamped(
            header = header,
            pose = self.init_pose
        ))
        self.initpose_cov_pub.publish(PoseWithCovarianceStamped(
            header = header,
            pose = PoseWithCovariance(
                pose = self.init_pose,
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

    def init_timer_cb(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time())
        except TransformException:
            self.get_logger().warn(f'cannot obtain {self.map_frame} -> {self.robot_frame} transform')
            return
        
        # verify transform is within error margin
        resend_pose = False
        if abs(tf.transform.translation.x - self.init_x) > self.init_pos_err or abs(tf.transform.translation.y - self.init_y) > self.init_pos_err:
            self.get_logger().info(f'current position ({tf.transform.translation.x}, {tf.transform.translation.y}) is off from desired position ({self.init_x}, {self.init_y})')
            resend_pose = True
        else:
            _, _, d_yaw = (Rotation.from_euler('z', -self.init_yaw) * Rotation.from_quat([
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w
            ])).as_euler('xyz')
            if abs(d_yaw) > self.init_yaw_err:
                self.get_logger().info(f'current yaw {d_yaw} is off from desired yaw {self.init_yaw}')
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

            self.timer.cancel(); self.destroy_timer(self.timer)
            self.timer = self.create_timer(0.5, self.goal_timer_cb, callback_group=self.callback_group)

    def goal_timer_cb(self):
        if self.goal_pub.get_subscription_count() == 0:
            self.get_logger().info('waiting for goal pose subscriber')
            return # no subscriptions - navigator not running yet?

        self.get_logger().info(f'sending out goal pose: (({self.goal_x}, {self.goal_y}), {self.goal_yaw})')
        
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.goal_x
        msg.pose.position.y = self.goal_y
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = Rotation.from_euler('z', self.goal_yaw).as_quat()
        self.goal_pub.publish(msg)
        self.goal_published = True

    def status_cb(self, data: GoalStatusArray):
        if not self.goal_published or len(data.status_list) == 0: return # nothing to do

        latest_status: GoalStatus = data.status_list[-1] # newest goal is last
        self.get_logger().info(f'latest goal status: {latest_status.status}')
        if latest_status.status == 1 or latest_status.status == 2: # accepted/executing
            self.get_logger().info(f'goal started execution, exiting')
            self.timer.cancel(); self.destroy_timer(self.timer)
            rclpy.shutdown() # raise SystemExit doesn't work here

def main():
    rclpy.init()
    node = InitAndGoal()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
