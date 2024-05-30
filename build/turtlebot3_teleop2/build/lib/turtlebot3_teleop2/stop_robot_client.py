import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

def main(args=None):
    rclpy.init(args=args)
    node = Node('stop_robot_client')
    client = node.create_client(Empty, 'stop_robot')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')
    
    req = Empty.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        node.get_logger().info('Robot stopped successfully.')
    else:
        node.get_logger().error('Failed to call service stop_robot.')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

