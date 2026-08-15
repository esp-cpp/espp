# rclpy add_two_ints service server for the espp interop matrix: the espp
# service CLIENT (rtps_service_interop_client) calls this and checks the sum.
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__("add_two_ints_server")
        self.create_service(AddTwoInts, "add_two_ints", self.cb)

    def cb(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f"{request.a} + {request.b} = {response.sum}")
        return response


def main():
    rclpy.init()
    rclpy.spin(AddTwoIntsServer())


main()
