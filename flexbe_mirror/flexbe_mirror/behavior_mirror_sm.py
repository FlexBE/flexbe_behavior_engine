import rclpy

from flexbe_core.proxy import ProxySubscriberCached

from flexbe_mirror.flexbe_mirror import FlexbeMirror


def main(args=None):
    rclpy.init(args=args)

    mirror = FlexbeMirror()
    # Wait for ctrl-c to stop the application
    rclpy.spin(mirror)

    mirror.get_logger().info("Mirror shutdown ...")
    ProxySubscriberCached().shutdown()
    mirror.get_logger().info("finished!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
