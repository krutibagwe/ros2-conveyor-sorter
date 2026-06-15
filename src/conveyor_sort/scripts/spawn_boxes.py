#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import random

BOX_SDF = """
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="box_link">
      <visual name="box_vis">
        <geometry><box><size>0.07 0.07 0.07</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

KNOWN_COLORS = {
    "red":    (0.9, 0.1, 0.1),
    "blue":   (0.1, 0.1, 0.9),
    "green":  (0.1, 0.8, 0.1),
}
MISC_COLORS = {
    "purple": (0.6, 0.1, 0.8),
    "yellow": (0.9, 0.9, 0.1),
    "orange": (0.9, 0.5, 0.1),
    "white":  (0.9, 0.9, 0.9),
}
ALL_COLORS  = {**KNOWN_COLORS, **MISC_COLORS}
COLOR_POOL  = (["red"]*3 + ["blue"]*3 + ["green"]*3
               + ["purple", "yellow", "orange", "white"])

BELT_SPEED    = 0.2
BELT_START_X  = -1.35
BELT_Y        =  0.05
BELT_Z        =  0.57
PUSH_SPEED    =  0.35
BIN_X         = {"red": -0.8, "blue": 0.2, "green": 1.2}
MISC_BIN_X    =  1.7
BIN_Y_TARGET  = -0.65
PUSH_TRIGGER  =  0.05
DT            =  0.1


class BoxSpawner(Node):
    def __init__(self):
        super().__init__('box_spawner')

        self.spawn_client  = self.create_client(SpawnEntity,  '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

        self.count_pub   = self.create_publisher(String, '/bin_counts',      10)
        self.color_pub   = self.create_publisher(String, '/active_box',      10)
        self.jam_sim_pub = self.create_publisher(String, '/jam_sim_status',  10)
        self.status_pub  = self.create_publisher(String, '/spawner_status',  10)

        self.cmd_sub = self.create_subscription(
            String, '/belt_command',  self.cmd_callback,     10)
        self.rst_sub = self.create_subscription(
            String, '/belt_reset',    self.reset_callback,   10)
        self.jam_sub = self.create_subscription(
            String, '/simulate_jam',  self.trigger_jam,      10)

        self.get_logger().info("Waiting for Gazebo services...")
        self.spawn_client.wait_for_service(timeout_sec=20.0)
        self.delete_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("Gazebo ready.")

        self.boxes       = []
        self.box_count   = 0
        self.belt_busy   = False
        self.jam_active  = False
        # STOPPED | RUNNING | PAUSED
        self.belt_state  = "STOPPED"

        self.bin_counts  = {
            "red": 0, "blue": 0, "green": 0, "misc": 0}

        self.move_timer  = self.create_timer(DT,   self.move_boxes)
        self.count_timer = self.create_timer(0.3,  self.publish_counts)
        self.status_timer= self.create_timer(0.5,  self.publish_status)

        self.get_logger().info(
            "Box spawner ready. Belt STOPPED. Send START to begin.")

    # ── Belt command handler ───────────────────────────────────────────────

    def cmd_callback(self, msg):
        cmd = msg.data.upper()

        if cmd == "START":
            if self.belt_state == "STOPPED":
                self.belt_state = "RUNNING"
                self.get_logger().info("Belt STARTED.")
                if not self.belt_busy and not self.boxes:
                    self.try_spawn()
            else:
                self.get_logger().warn(
                    f"Cannot START from state {self.belt_state}. "
                    f"Use RESUME if paused, or RESET first.")

        elif cmd == "STOP":
            if self.belt_state in ("RUNNING", "PAUSED"):
                self.belt_state = "STOPPED"
                self.get_logger().info(
                    "Belt STOPPED. Current box frozen on belt. "
                    "Send START after RESET, or RESUME is not valid from STOP.")
            else:
                self.get_logger().warn("Belt already stopped.")

        elif cmd == "PAUSE":
            if self.belt_state == "RUNNING":
                self.belt_state = "PAUSED"
                self.get_logger().info("Belt PAUSED — frozen in place.")
            else:
                self.get_logger().warn(
                    "Belt not running — cannot pause.")

        elif cmd == "RESUME":
            if self.belt_state == "PAUSED":
                self.belt_state = "RUNNING"
                self.get_logger().info("Belt RESUMED from pause.")
            elif self.jam_active:
                self.jam_active = False
                self.belt_state = "RUNNING"
                self.get_logger().info("Jam cleared by RESUME — belt running.")
                status = String()
                status.data = "JAM_CLEARED"
                self.jam_sim_pub.publish(status)
            else:
                self.get_logger().warn(
                    "Belt not paused/jammed — RESUME has no effect.")

        else:
            self.get_logger().warn(f"Unknown command: {cmd}")

    # ── Reset handler ──────────────────────────────────────────────────────

    def reset_callback(self, msg):
        self.get_logger().warn("RESET received — clearing everything.")

        self.belt_state = "STOPPED"
        self.belt_busy  = False
        self.jam_active = False

        status = String()
        status.data = "JAM_CLEARED"
        self.jam_sim_pub.publish(status)

        # Delete all active boxes from Gazebo
        for box in self.boxes:
            self._delete_box(box['name'])
        self.boxes = []

        # Reset counters
        self.bin_counts = {"red": 0, "blue": 0, "green": 0, "misc": 0}
        self.box_count  = 0

        # Clear RViz active box marker
        clear = String()
        clear.data = "none,0.0,0.0,idle"
        self.color_pub.publish(clear)

        self.get_logger().info(
            "Reset complete. Belt STOPPED. Send START to begin again.")

    # ── Jam simulation ─────────────────────────────────────────────────────

    def trigger_jam(self, msg):
        if self.jam_active:
            self.get_logger().warn("Already jammed.")
            return
        if self.belt_state != "RUNNING":
            self.get_logger().warn("Cannot jam — belt not running.")
            return

        self.jam_active = True
        self.get_logger().warn("SIMULATED JAM TRIGGERED! "
                                "Belt frozen until RESUME or RESET.")

        status = String()
        status.data = "JAM_ACTIVE"
        self.jam_sim_pub.publish(status)

    # ── Spawn ──────────────────────────────────────────────────────────────

    def try_spawn(self):
        if self.belt_busy or self.jam_active:
            return
        if self.belt_state != "RUNNING":
            return

        self.belt_busy = True
        color_name     = random.choice(COLOR_POOL)
        r, g, b        = ALL_COLORS[color_name]
        name           = f"box_{self.box_count}_{color_name}"
        self.box_count += 1

        sdf = BOX_SDF.format(name=name, r=r, g=g, b=b)
        pose = Pose()
        pose.position.x  = BELT_START_X
        pose.position.y  = BELT_Y
        pose.position.z  = BELT_Z
        pose.orientation.w = 1.0

        req = SpawnEntity.Request()
        req.name         = name
        req.xml          = sdf
        req.initial_pose = pose

        future = self.spawn_client.call_async(req)
        future.add_done_callback(
            lambda f, n=name, c=color_name: self._on_spawned(f, n, c))

        is_misc = color_name not in KNOWN_COLORS
        dest = "MISC bin" if is_misc else f"bin X={BIN_X[color_name]:.1f}"
        self.get_logger().info(f"Spawning [{color_name}] → {dest}")

    def _on_spawned(self, future, name, color):
        try:
            result = future.result()
            if result.success:
                is_misc = color not in KNOWN_COLORS
                self.boxes.append({
                    'name':    name,
                    'color':   color,
                    'is_misc': is_misc,
                    'x':       float(BELT_START_X),
                    'y':       float(BELT_Y),
                    'state':   'moving',
                    'counted': False,
                })
                self.get_logger().info(f"[{name}] on belt.")
            else:
                self.belt_busy = False
        except Exception as e:
            self.get_logger().error(f"Spawn error: {e}")
            self.belt_busy = False

    # ── Move loop ──────────────────────────────────────────────────────────

    def move_boxes(self):
        # Freeze everything if not actively running
        if self.belt_state != "RUNNING" or self.jam_active:
            self._publish_active_box()
            return

        to_remove = []

        for box in self.boxes:
            if box['state'] == 'moving':
                box['x'] += BELT_SPEED * DT

                if box['is_misc']:
                    if box['x'] >= MISC_BIN_X:
                        box['x']    = MISC_BIN_X
                        box['state'] = 'pushing'
                else:
                    target_x = BIN_X[box['color']]
                    if box['x'] >= target_x - PUSH_TRIGGER:
                        box['x']    = target_x
                        box['state'] = 'pushing'
                        self.get_logger().info(
                            f"PUSH [{box['color']}] → bin!")

            elif box['state'] == 'pushing':
                box['y'] -= PUSH_SPEED * DT

                if not box['counted'] and box['y'] <= BIN_Y_TARGET + 0.08:
                    box['counted'] = True
                    if box['is_misc']:
                        self.bin_counts['misc'] += 1
                        self.get_logger().info(
                            f"MISC [{box['color']}] in misc bin! "
                            f"Misc={self.bin_counts['misc']}")
                    else:
                        self.bin_counts[box['color']] += 1
                        self.get_logger().info(
                            f"SORTED [{box['color']}]! "
                            f"R={self.bin_counts['red']} "
                            f"B={self.bin_counts['blue']} "
                            f"G={self.bin_counts['green']} "
                            f"Misc={self.bin_counts['misc']}")

                if box['y'] <= BIN_Y_TARGET - 0.1:
                    box['state'] = 'done'
                    self._delete_box(box['name'])
                    to_remove.append(box['name'])
                    self.belt_busy = False
                    self.create_timer(3.0, self.try_spawn)
                    continue

        self.boxes = [b for b in self.boxes
                      if b['name'] not in to_remove]

        self._publish_active_box()

    def _publish_active_box(self):
        if self.boxes:
            box = self.boxes[0]
            msg = String()

            if self.jam_active:
                state = "jammed"
            elif self.belt_state == "PAUSED":
                state = "paused"
            elif self.belt_state == "STOPPED":
                state = "stopped"
            else:
                state = box['state']

            msg.data = (
                f"{box['color']},"
                f"{box['x']:.3f},"
                f"{box['y']:.3f},"
                f"{state}"
            )
            self.color_pub.publish(msg)

    def _delete_box(self, name):
        req = DeleteEntity.Request()
        req.name = name
        self.delete_client.call_async(req)

    # ── Status + counts ────────────────────────────────────────────────────

    def publish_counts(self):
        msg = String()
        msg.data = (
            f"RED:{self.bin_counts['red']},"
            f"BLUE:{self.bin_counts['blue']},"
            f"GREEN:{self.bin_counts['green']},"
            f"MISC:{self.bin_counts['misc']}"
        )
        self.count_pub.publish(msg)

    def publish_status(self):
        msg = String()
        if self.jam_active:
            msg.data = "JAMMED"
        else:
            msg.data = self.belt_state
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = BoxSpawner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()