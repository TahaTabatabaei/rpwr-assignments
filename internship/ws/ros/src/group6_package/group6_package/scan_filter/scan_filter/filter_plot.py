import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

class ScanPlotter(Node):
    def __init__(self):
        super().__init__('scan_plotter')
        self.orig_scan = None
        self.filt_scan = None

        self.create_subscription(LaserScan, '/scan', self.orig_callback, 10)
        self.create_subscription(LaserScan, '/scan_filtered', self.filt_callback, 10)

    def orig_callback(self, msg):
        if self.orig_scan is None:
            self.orig_scan = msg
            self.try_plot()

    def filt_callback(self, msg):
        if self.filt_scan is None:
            self.filt_scan = msg
            self.try_plot()

    def try_plot(self):
        if self.orig_scan and self.filt_scan:
            plt.figure(figsize=(10, 5))
            plt.plot(self.orig_scan.ranges, label='Original /scan', alpha=0.7)
            plt.plot(self.filt_scan.ranges, label='Filtered /scan_filtered', alpha=0.7)
            plt.xlabel('Index')
            plt.ylabel('Distance (m)')
            plt.legend()
            plt.title('Comparison of Original and Filtered LaserScan')
            plt.show()
            rclpy.shutdown()  # Stop after plotting

def main(args=None):
    rclpy.init(args=args)
    node = ScanPlotter()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
