import rclpy

from flexbe_core.proxy import ProxySubscriberCached
from flexbe_onboard.flexbe_onboard import FlexbeOnboard

def main(args=None):
    rclpy.init(args=args)

    onboard = FlexbeOnboard()

    # Wait for ctrl-c to stop the application
    rclpy.spin(onboard)

    onboard.get_logger().info("Onboard shutdown ...")
    ProxySubscriberCached().shutdown()
    onboard.get_logger().info("Done!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
