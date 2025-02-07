import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

from scipy.spatial.transform import Rotation

def main():
    rclpy.init()
    navigator = BasicNavigator()

    map_frame = navigator.declare_parameter('map_frame', 'map').get_parameter_value().string_value
    x = navigator.declare_parameter('x', 0.0).get_parameter_value().double_value
    y = navigator.declare_parameter('y', 0.0).get_parameter_value().double_value
    yaw = navigator.declare_parameter('yaw', 0.0).get_parameter_value().double_value # in radians

    navigator.get_logger().info(f'waiting until Nav2 becomes active')
    navigator.waitUntilNav2Active()

    # make goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = map_frame
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = Rotation.from_euler('z', yaw).as_quat()
    navigator.goToPose(goal_pose)

    rclpy.shutdown()

if __name__ == '__main__':
    main()