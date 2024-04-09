import rclpy
from rclpy.node import Node
from custom_msgs.srv import YourService  # Replace 'YourService' with your actual service type

class ServiceSubscriber(Node):
    def __init__(self):
        super().__init__('service_subscriber')
        self.client = self.create_client(YourService, 'your_service_name')  # Replace 'YourService' and 'your_service_name'

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = YourService.Request()
        self.client.call_async(self.req).add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            # Process and save the response data to a file or storage
            if response:
                with open('service_response_data.txt', 'w') as file:
                    file.write(str(response))  # Example: Saving response as a string to a text file
                self.get_logger().info('Service response recorded.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    service_subscriber = ServiceSubscriber()
    rclpy.spin(service_subscriber)
    service_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
