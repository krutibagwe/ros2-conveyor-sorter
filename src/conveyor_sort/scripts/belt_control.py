#!/usr/bin/env python3
"""
Belt Control CLI
Commands: start, stop, pause, resume, reset, jam, status, quit
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BeltControlCLI(Node):
    def __init__(self):
        super().__init__('belt_control_cli')

        self.cmd_pub   = self.create_publisher(String, '/belt_command', 10)
        self.reset_pub = self.create_publisher(String, '/belt_reset',   10)
        self.jam_pub   = self.create_publisher(String, '/simulate_jam', 10)

        self.status_sub = self.create_subscription(
            String, '/belt_status',
            self.status_callback, 10)

        self.current_status = "UNKNOWN"

    def status_callback(self, msg):
        self.current_status = msg.data

    def send(self, pub, data):
        msg = String()
        msg.data = data
        pub.publish(msg)

    def print_help(self):
        print("\n" + "="*50)
        print("  CONVEYOR BELT CONTROL CLI")
        print("="*50)
        print("  start   — start belt from STOPPED state")
        print("  stop    — emergency stop (RESET needed to restart)")
        print("  pause   — freeze belt in place")
        print("  resume  — continue from PAUSE or clear a JAM")
        print("  reset   — clear all boxes + counts, belt → STOPPED")
        print("  jam     — simulate a jam (stays until resume/reset)")
        print("  status  — show current belt status")
        print("  help    — show this menu")
        print("  quit    — exit CLI")
        print("="*50)
        print()

    def run(self):
        self.print_help()
        while rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.1)
                raw = input("belt> ").strip().lower()

                if not raw:
                    continue

                if raw == "start":
                    self.send(self.cmd_pub, "START")
                    print("  → START sent.")

                elif raw == "stop":
                    self.send(self.cmd_pub, "STOP")
                    print("  → STOP sent. Send RESET then START to restart.")

                elif raw == "pause":
                    self.send(self.cmd_pub, "PAUSE")
                    print("  → PAUSE sent. Send RESUME to continue.")

                elif raw == "resume":
                    self.send(self.cmd_pub, "RESUME")
                    print("  → RESUME sent.")

                elif raw == "reset":
                    confirm = input(
                        "  Reset all boxes and counts? (y/n): "
                    ).strip().lower()
                    if confirm == "y":
                        self.send(self.reset_pub, "RESET")
                        print("  → RESET sent. Send START to begin again.")
                    else:
                        print("  Reset cancelled.")

                elif raw == "jam":
                    self.send(self.jam_pub, "jam")
                    print("  → JAM triggered! Send RESUME or RESET to clear.")

                elif raw == "status":
                    rclpy.spin_once(self, timeout_sec=0.3)
                    print(f"  Current status: {self.current_status}")

                elif raw in ("help", "h", "?"):
                    self.print_help()

                elif raw in ("quit", "exit", "q"):
                    print("  Exiting belt control CLI.")
                    break

                else:
                    print(f"  Unknown command '{raw}'. Type 'help' for list.")

            except (KeyboardInterrupt, EOFError):
                print("\n  Exiting.")
                break


def main():
    rclpy.init()
    node = BeltControlCLI()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()