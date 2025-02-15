import rclpy
from rclpy.node import Node

from nav2_msgs.srv import ClearEntireCostmap

def main():
    rclpy.init()
    node = Node('clear_costmaps')

    for node_name in ['local_costmap', 'global_costmap']:
        service = node.create_client(ClearEntireCostmap, f'/{node_name}/clear_entirely_{node_name}')
        while not service.wait_for_service(timeout_sec=1.0):
            node.get_logger().info(f'waiting until costmap node {node_name} is available')
        node.get_logger().info(f'clearing costmap provided by node {node_name}')
        future = service.call_async(ClearEntireCostmap.Request())
        rclpy.spin_until_future_complete(node, future)
        node.get_logger().info(f'{node_name} costmap clearing completed')
    
    rclpy.shutdown()
