#!/usr/bin/env python3
"""
Simple script to subscribe to /conveyor_speed and plot it live.
Run separately: python3 plot_speed.py
Requires: matplotlib, rclpy
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from collections import deque

class SpeedPlotter(Node):
    def __init__(self):
        super().__init__('speed_plotter')
        self.sub = self.create_subscription(
            Float32, '/conveyor_speed', self.callback, 10)
        self.speeds = deque(maxlen=200)

    def callback(self, msg):
        self.speeds.append(msg.data)

def main():
    rclpy.init()
    node = SpeedPlotter()
    plt.ion()
    fig, ax = plt.subplots()

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        ax.clear()
        ax.plot(list(node.speeds), color='blue')
        ax.set_title("Conveyor Speed")
        ax.set_ylabel("Speed")
        ax.set_xlabel("Samples")
        ax.set_ylim(-0.5, 2.0)
        plt.pause(0.05)

    rclpy.shutdown()

if __name__ == '__main__':
    main()