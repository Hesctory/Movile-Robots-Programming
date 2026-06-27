#!/usr/bin/env python3
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String
from tf2_ros import StaticTransformBroadcaster


class RoboMapper(Node):

    # Grid covers 20 x 20 m centred at world origin — fits the full arena
    GRID_SIZE = 100     # cells
    RESOLUTION = 0.2    # metres per cell

    # Carried-flag masking (must match the controller's _sector_min mask). The grabbed
    # flag is rigidly held in a short forward cone and crosses the 2-D LiDAR plane, so
    # its returns must not be mapped as a wall that follows the robot.
    CARGO_MASK_ANGLE = 0.7   # rad (~±40°) — forward half-cone occupied by the flag
    CARGO_MASK_RANGE = 0.6   # m — returns closer than this in the cone are the cargo

    def __init__(self):
        super().__init__('robo_mapper')

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Pose, '/model/prm_robot/pose', self.odom_callback, 10)
        self.create_subscription(String, '/robot_state', self.state_callback, 10)

        self.map_pub = self.create_publisher(OccupancyGrid, '/grid_map', 10)
        self.timer = self.create_timer(0.5, self.publish_occupancy_grid)

        # Robot pose (world frame)
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0

        # True while the flag is grabbed and held — enables cargo masking below.
        self.carrying = False

        # -1 = unknown, 0 = free, 100 = occupied
        self.grid_map = -np.ones((self.GRID_SIZE, self.GRID_SIZE), dtype=np.int8)

        # Broadcast static map → odom_gt transform so RViz can display the grid
        self.tf_static = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom_gt'
        tf.transform.rotation.w = 1.0
        self.tf_static.sendTransform(tf)

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------

    def state_callback(self, msg: String):
        # The flag is clamped from GRABBING onward and held until base is reached.
        self.carrying = msg.data in ('GRABBING', 'LIFTING', 'RETURNING_TO_BASE')

    def odom_callback(self, msg: Pose):
        self.x = msg.position.x
        self.y = msg.position.y

        q = msg.orientation
        # yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.heading = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg: LaserScan):
        robot_gx, robot_gy = self.world_to_grid(self.x, self.y)
        if not self._in_bounds(robot_gx, robot_gy):
            return

        angle = msg.angle_min
        for r in msg.ranges:
            in_range = math.isfinite(r) and msg.range_min < r < msg.range_max
            # Skip the carried flag: a return in the short forward cone is our cargo, not
            # the world (beam angle is already robot-relative; 0 = straight ahead).
            beam = (angle + math.pi) % (2 * math.pi) - math.pi
            cargo = (self.carrying
                     and abs(beam) < self.CARGO_MASK_ANGLE
                     and r < self.CARGO_MASK_RANGE)
            if in_range and not cargo:
                world_angle = self.heading + angle
                ex = self.x + r * math.cos(world_angle)
                ey = self.y + r * math.sin(world_angle)
                end_gx, end_gy = self.world_to_grid(ex, ey)
                self._bresenham(robot_gx, robot_gy, end_gx, end_gy, mark_end=True)
            angle += msg.angle_increment

    # ------------------------------------------------------------------
    # Publisher
    # ------------------------------------------------------------------

    def publish_occupancy_grid(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.RESOLUTION
        msg.info.width = self.GRID_SIZE
        msg.info.height = self.GRID_SIZE

        origin = Pose()
        half = (self.GRID_SIZE * self.RESOLUTION) / 2.0
        origin.position.x = -half
        origin.position.y = -half
        origin.orientation.w = 1.0
        msg.info.origin = origin

        msg.data = self.grid_map.flatten().tolist()
        self.map_pub.publish(msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def world_to_grid(self, x: float, y: float):
        half = self.GRID_SIZE * self.RESOLUTION / 2.0
        gx = int((x + half) / self.RESOLUTION)
        gy = int((y + half) / self.RESOLUTION)
        return gx, gy

    def _in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.GRID_SIZE and 0 <= gy < self.GRID_SIZE

    def _bresenham(self, x0: int, y0: int, x1: int, y1: int, mark_end: bool):
        """Walk cells from (x0,y0) to (x1,y1), marking free; optionally mark end occupied."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        err = dx - dy

        while True:
            at_end = (x0 == x1 and y0 == y1)

            if self._in_bounds(x0, y0):
                if at_end and mark_end:
                    self.grid_map[y0, x0] = 100
                elif not at_end and self.grid_map[y0, x0] != 100:
                    self.grid_map[y0, x0] = 0

            if at_end:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy


def main(args=None):
    rclpy.init(args=args)
    node = RoboMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
