import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt

class LiveScanPlot(Node):
    def __init__(self):
        super().__init__('live_scan_plot')
        self.sub_orig = self.create_subscription(LaserScan, '/scan', self.orig_cb, 1)
        self.sub_filt = self.create_subscription(LaserScan, '/scan_filtered', self.filt_cb, 1)
        self.sub_outl = self.create_subscription(LaserScan, '/scan_filtered_outliers', self.outl_cb, 1)
        self.orig = None
        self.filt = None
        self.outl = None

        plt.ion()
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})

    def orig_cb(self, msg):
        self.orig = (msg.ranges, msg.angle_min, msg.angle_increment)
        self.update_plot()

    def filt_cb(self, msg):
        self.filt = (msg.ranges, msg.angle_min, msg.angle_increment)
        self.update_plot()

    def outl_cb(self, msg):
        self.outl = (msg.ranges, msg.angle_min, msg.angle_increment)
        self.update_plot()

    def update_plot(self):
        if self.orig is None or self.filt is None:
            return
        self.ax.clear()
        # Original
        r, a_min, a_inc = self.orig
        th = np.arange(len(r)) * a_inc + a_min
        self.ax.plot(th, r, '.', label='Original', color='blue', alpha=0.3)
        # Filtered
        r, a_min, a_inc = self.filt
        th = np.arange(len(r)) * a_inc + a_min
        self.ax.plot(th, r, '.', label='Filtered', color='red', alpha=0.8)
        # Outliers (optional)
        if self.outl is not None:
            r, a_min, a_inc = self.outl
            th = np.arange(len(r)) * a_inc + a_min
            self.ax.plot(th, r, '.', label='Outliers', color='green', alpha=0.8)
        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main():
    rclpy.init()
    node = LiveScanPlot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    plt.ioff()
    plt.show()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
