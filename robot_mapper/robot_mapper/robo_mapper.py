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

    # --- log-odds occupancy model (hit/miss with active free-space clearing) ---
    # The grid is accumulated as log-odds, not a single sticky write, so (a) single spurious
    # returns no longer stick as permanent obstacles, and (b) free space is actively cleared.
    # Crucially, a beam that returns NOTHING clears free out to max range — this is what finally
    # maps the wide open zones the 3.5 m LiDAR can't bounce a return across.
    L_OCC  = 0.7      # log-odds added to a hit (endpoint) cell
    L_FREE = -0.4     # log-odds added to each cell a beam passes through / clears
    L_MIN  = -2.0     # clamp range — keeps cells correctable (recovers from error/dynamic objects)
    L_MAX  = 3.0
    OCC_THRESH  = 1.0   # log-odds > this ⇒ published occupied (needs ~2 consistent hits)
    FREE_THRESH = -0.7  # log-odds < this ⇒ published free     (needs ~2 consistent clears)

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

        # Log-odds occupancy (0.0 = unknown). The published -1/0/100 grid is thresholded
        # from this each publish cycle.
        self.logodds = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=np.float32)

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
            beam = (angle + math.pi) % (2 * math.pi) - math.pi
            # Skip the carried flag entirely: a finite return in the short forward cone is
            # our cargo, not the world — don't let it clear OR mark anything.
            cargo = (self.carrying
                     and abs(beam) < self.CARGO_MASK_ANGLE
                     and math.isfinite(r) and r < self.CARGO_MASK_RANGE)
            if not cargo:
                world_angle = self.heading + angle
                hit = math.isfinite(r) and msg.range_min < r < msg.range_max
                # No return ⇒ nothing within max range this way: clear free out to max range
                # (no occupied endpoint). This is what finally fills the wide open zones.
                rng = r if hit else msg.range_max
                ex = self.x + rng * math.cos(world_angle)
                ey = self.y + rng * math.sin(world_angle)
                end_gx, end_gy = self.world_to_grid(ex, ey)
                self._ray_update(robot_gx, robot_gy, end_gx, end_gy, occupied_end=hit)
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

        # Threshold log-odds into the ROS occupancy convention (-1 unknown / 0 free / 100 occ).
        grid = np.full((self.GRID_SIZE, self.GRID_SIZE), -1, dtype=np.int8)
        grid[self.logodds > self.OCC_THRESH] = 100
        grid[self.logodds < self.FREE_THRESH] = 0
        msg.data = grid.flatten().tolist()
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

    def _bump(self, gx: int, gy: int, delta: float):
        """Add to a cell's log-odds, clamped to [L_MIN, L_MAX]."""
        v = self.logodds[gy, gx] + delta
        self.logodds[gy, gx] = min(self.L_MAX, max(self.L_MIN, v))

    def _ray_update(self, x0: int, y0: int, x1: int, y1: int, occupied_end: bool):
        """Bresenham from robot (x0,y0) to endpoint (x1,y1): clear every cell the beam
        passes through with L_FREE; if occupied_end, mark the endpoint a hit with L_OCC."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        err = dx - dy

        while True:
            at_end = (x0 == x1 and y0 == y1)

            if self._in_bounds(x0, y0):
                if at_end and occupied_end:
                    self._bump(x0, y0, self.L_OCC)
                else:
                    self._bump(x0, y0, self.L_FREE)

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
