import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

frames = []

class ScanRecorder(Node):
    def __init__(self):
        super().__init__('scan_recorder')
        self.sub_filt = self.create_subscription(LaserScan, '/scan_filtered', self.filt_cb, 1)
        self.count = 0
        self.max_frames = 100  # Record 100 frames

    def filt_cb(self, msg):
        global frames
        if self.count >= self.max_frames:
            return
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        th = np.arange(len(ranges)) * angle_inc + angle_min
        frames.append((th, ranges.copy()))
        self.count += 1
        if self.count == self.max_frames:
            self.get_logger().info("Recording finished. Saving animation...")

def main():
    rclpy.init()
    node = ScanRecorder()
    while rclpy.ok() and node.count < node.max_frames:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    # --- Animation ---
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    ln, = ax.plot([], [], 'r.')

    def init():
        ax.set_ylim(0, 5)
        return ln,

    def update(frame):
        th, r = frame
        ln.set_data(th, r)
        return ln,

    ani = animation.FuncAnimation(
        fig, update, frames=frames, init_func=init, blit=True, interval=50)
    ani.save('filtered_scan.gif', writer='imagemagick')
    plt.show()

if __name__ == '__main__':
    main()
