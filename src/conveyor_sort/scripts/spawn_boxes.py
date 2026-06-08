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

# Known colors + misc colors (purple, yellow, orange, white)
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

ALL_COLORS = {**KNOWN_COLORS, **MISC_COLORS}

# Spawn weights — misc appears ~25% of the time
COLOR_POOL = (
    ["red"] * 3 + ["blue"] * 3 + ["green"] * 3 +
    ["purple", "yellow", "orange", "white"]
)

BELT_SPEED   = 0.2
BELT_START_X = -1.35
BELT_Y       =  0.05
BELT_Z       =  0.57
PUSH_SPEED   =  0.35

# Bin X positions for known colors
# Misc bin is at end of belt — no pusher needed
BIN_X = {
    "red":    -0.8,
    "blue":    0.2,
    "green":   1.2,
}
MISC_BIN_X   =  1.7   # end of belt — box rides here naturally
BIN_Y_TARGET = -0.65
PUSH_TRIGGER =  0.05
DT           =  0.1

# Jam simulation
JAM_DURATION =  8.0   # seconds a simulated jam lasts


class BoxSpawner(Node):
    def __init__(self):
        super().__init__('box_spawner')

        self.spawn_client  = self.create_client(SpawnEntity,  '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

        self.count_pub     = self.create_publisher(String, '/bin_counts',  10)
        self.color_pub     = self.create_publisher(String, '/active_box',  10)
        self.jam_sim_pub   = self.create_publisher(String, '/jam_sim_status', 10)

        # Subscribe to jam trigger topic
        self.jam_trigger_sub = self.create_subscription(
            String, '/simulate_jam',
            self.trigger_jam, 10)

        self.get_logger().info("Waiting for Gazebo services...")
        self.spawn_client.wait_for_service(timeout_sec=20.0)
        self.delete_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("Ready!")

        self.boxes      = []
        self.box_count  = 0
        self.belt_busy  = False
        self.jam_active = False

        self.bin_counts = {
            "red": 0, "blue": 0, "green": 0, "misc": 0
        }

        self.move_timer  = self.create_timer(DT,   self.move_boxes)
        self.count_timer = self.create_timer(0.3,  self.publish_counts)

        # First box after 2s
        self.create_timer(2.0, self._first_spawn)

    def _first_spawn(self):
        self.try_spawn()

    # ── Jam simulation ─────────────────────────────────────────────────────

    def trigger_jam(self, msg):
        if self.jam_active:
            self.get_logger().warn("Jam already active — ignoring.")
            return

        self.jam_active = True
        self.get_logger().warn("=" * 40)
        self.get_logger().warn("SIMULATED JAM TRIGGERED!")
        self.get_logger().warn("Belt frozen for %.1fs" % JAM_DURATION)
        self.get_logger().warn("=" * 40)

        # Stop the move timer — freezes all boxes
        self.move_timer.cancel()

        # Publish jam status
        status = String()
        status.data = "JAM_ACTIVE"
        self.jam_sim_pub.publish(status)

        # Auto-clear after JAM_DURATION seconds
        self.create_timer(JAM_DURATION, self._clear_jam)

    def _clear_jam(self):
        if not self.jam_active:
            return
        self.jam_active = False
        self.get_logger().info("Jam cleared — resuming belt.")

        # Restart move timer
        self.move_timer = self.create_timer(DT, self.move_boxes)

        # Publish clear status
        status = String()
        status.data = "JAM_CLEARED"
        self.jam_sim_pub.publish(status)

    # ── Spawn ──────────────────────────────────────────────────────────────

    def try_spawn(self):
        if self.belt_busy or self.jam_active:
            return
        self.belt_busy = True

        color_name = random.choice(COLOR_POOL)
        r, g, b    = ALL_COLORS[color_name]
        name       = f"box_{self.box_count}_{color_name}"
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
        dest = "MISC bin (end)" if is_misc else f"bin at X={BIN_X[color_name]:.1f}"
        self.get_logger().info(
            f"Spawning [{color_name}] → {dest}")

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
        to_remove = []

        for box in self.boxes:
            if box['state'] == 'moving':
                box['x'] += BELT_SPEED * DT

                if box['is_misc']:
                    # Misc — ride to end of belt, no pusher
                    if box['x'] >= MISC_BIN_X:
                        box['x']    = MISC_BIN_X
                        box['state'] = 'pushing'
                else:
                    # Known color — push sideways at correct bin
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
                            f"MISC box [{box['color']}] in misc bin! "
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

        self.boxes = [b for b in self.boxes if b['name'] not in to_remove]

        # Publish active box info for RViz + camera
        if self.boxes:
            box = self.boxes[0]
            msg = String()
            msg.data = (
                f"{box['color']},"
                f"{box['x']:.3f},"
                f"{box['y']:.3f},"
                f"{box['state']}"
            )
            self.color_pub.publish(msg)

    def _delete_box(self, name):
        req = DeleteEntity.Request()
        req.name = name
        self.delete_client.call_async(req)

    # ── Counts ─────────────────────────────────────────────────────────────

    def publish_counts(self):
        msg = String()
        msg.data = (
            f"RED:{self.bin_counts['red']},"
            f"BLUE:{self.bin_counts['blue']},"
            f"GREEN:{self.bin_counts['green']},"
            f"MISC:{self.bin_counts['misc']}"
        )
        self.count_pub.publish(msg)


def main():
    rclpy.init()
    node = BoxSpawner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()