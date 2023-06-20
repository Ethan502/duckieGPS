import rclpy
from rclpy.node import Node
from custom_interfaces.srv import DuckieLocation

class DuckieLocatorClientNode(Node):
    def __init__(self):
        super().__init__('DuckieGPS_Client')
        self.client = self.create_client(DuckieLocation, 'duckielocation')
        self.request = DuckieLocation.Request()

    def send_request(self,n):
        self.request.botnum = n
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
def main():
    rclpy.init()
    client = DuckieLocatorClientNode()
    response = client.send_request(2)
    client.get_logger().info('x: %d y: %d' % (response.x, response.y))
    client.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()
