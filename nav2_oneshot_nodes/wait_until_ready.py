import rclpy
from rclpy import qos
from rclpy.node import Node

import rclpy.time
from lifecycle_msgs.srv import GetState

import time

def main():
    rclpy.init()
    node = Node('wait_until_ready')
        
    node_name = node.declare_parameter('node', 'bt_navigator').get_parameter_value().string_value
    localiser_state_srv = node.create_client(GetState, f'{node_name}/get_state')
    while not localiser_state_srv.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(f'waiting for node {node_name} to show up')
    while True:
        future = localiser_state_srv.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node, future) # we can ONLY do this outside of callbacks!
        if future.result() is None:
            node.get_logger().error(f'node {node_name} returned null for state retrieval request')
        else:
            state = future.result().current_state.label
            node.get_logger().info(f'node {node_name} status: {state}')
            if state == 'active': break
        time.sleep(0.5)
    
    rclpy.shutdown()
