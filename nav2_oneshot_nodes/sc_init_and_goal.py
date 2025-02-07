import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

from scipy.spatial.transform import Rotation

def main():
    rclpy.init()
    navigator = BasicNavigator()

    map_frame = navigator.declare_parameter('map_frame', 'map').get_parameter_value().string_value
    init_x = navigator.declare_parameter('init_x', 0.0).get_parameter_value().double_value
    init_y = navigator.declare_parameter('init_y', 0.0).get_parameter_value().double_value
    init_yaw = navigator.declare_parameter('init_yaw', 0.0).get_parameter_value().double_value # in radians
    goal_x = navigator.declare_parameter('goal_x', 0.0).get_parameter_value().double_value
    goal_y = navigator.declare_parameter('goal_y', 0.0).get_parameter_value().double_value
    goal_yaw = navigator.declare_parameter('goal_yaw', 0.0).get_parameter_value().double_value # in radians

    init_pose = PoseStamped()
    init_pose.header.frame_id = map_frame
    init_pose.header.stamp = navigator.get_clock().now().to_msg()
    init_pose.pose.position.x = init_x
    init_pose.pose.position.y = init_y
    init_pose.pose.orientation.x, init_pose.pose.orientation.y, init_pose.pose.orientation.z, init_pose.pose.orientation.w = Rotation.from_euler('z', init_yaw).as_quat()
    navigator.setInitialPose(init_pose)

    navigator.get_logger().info(f'waiting until Nav2 becomes active')
    navigator.waitUntilNav2Active()

    # make goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = map_frame
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = Rotation.from_euler('z', goal_yaw).as_quat()
    navigator.goToPose(goal_pose)

    rclpy.shutdown()

if __name__ == '__main__':
    main()