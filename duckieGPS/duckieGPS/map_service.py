import rclpy
from rclpy.node import Node
from .triangulate import triangulate
from custom_interfaces.srv import DuckieLocation

class DuckieLocatorServiceNode(Node):
    def __init__(self):
        super().__init__('DuckieGPS_Service')
        self.service = self.create_service(DuckieLocation, 'duckielocation', self.locator_callback)

    def locator_callback(self, request, response):
        self.num = request.botnum
        self.get_logger().info("Getting a request to the DuckieGPS servce")
        response.x, response.y = triangulate(self.num)
        return response
    
def main():
    rclpy.init()

    service = DuckieLocatorServiceNode()
    rclpy.spin(service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()