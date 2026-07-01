#!/usr/bin/env python3
import heapq
import math
import random
from enum import Enum

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Float64MultiArray, String


class State(Enum):
    EXPLORING = 0
    FLAG_DETECTED = 1
    NAVIGATING_TO_FLAG = 2
    POSITION_TO_COLLECT = 3
    GRABBING = 4
    LIFTING = 5
    RETURNING_TO_BASE = 6


class ControleRobo(Node):

    # --- control loop ---
    # Single source of truth for the timer period. All tick-based durations below
    # are derived from seconds via this value, so the loop rate can be changed here
    # without altering any real-world timing.
    CONTROL_PERIOD = 0.1    # seconds (10 Hz)

    # --- grid (must match robot_mapper constants) ---
    GRID_SIZE = 100
    GRID_RESOLUTION = 0.2
    OBSTACLE_INFLATE_RADIUS = 3   # cells of safety margin around obstacles

    # --- distances (metres) ---
    OBSTACLE_FRONT = 0.6   # open-space reaction: engage avoidance when the front is this close
    # Block-check distance while FOLLOWING an A* path. A valid planned path legitimately
    # threads gaps close to obstacles, so using the larger open-space OBSTACLE_FRONT here
    # would reject A*'s own corridors and deadlock (turn-away → re-plan → identical path).
    # This smaller value only flags an obstacle CLOSER than A* planned for — a real surprise.
    PATH_BLOCKED_DIST = 0.5
    CLOSE_ENOUGH = 0.7
    TARGET_DIST = 0.3

    # --- camera / detection ---
    FLAG_LABEL = 25
    MIN_FLAG_PIXELS = 25       # blob area to ACQUIRE the flag (Schmitt high threshold) — sized for the thin pole (label 25); ~5 m range
    MIN_FLAG_PIXELS_LOW = 14   # blob area below which we DROP it (Schmitt low threshold)
    FLAG_LOST_GRACE = 3        # camera frames a dropout is tolerated before flag_visible→False
    CONFIRM_FRAMES = 5
    FLAG_LOST_TICKS_MAX = int(0.8 / CONTROL_PERIOD)    # control ticks flag must stay lost before FLAG_DETECTED→EXPLORING (~0.8 s)
    CAMERA_FOV = 1.57
    # Calibrate: drive to exactly CLOSE_ENOUGH metres from the flag and read
    # flag_pixel_height, then set FLAG_CAM_K = CLOSE_ENOUGH * pixel_height.
    FLAG_CAM_K = 84.0   # pixel_height × distance (m) — tune in simulation

    # --- re-acquisition after losing flag ---
    SEARCH_TICKS_MAX = int(6.0 / CONTROL_PERIOD)   # ~6 s of re-acquisition search

    # --- "obstacle ahead is actually the flag" guard (prevents circumnavigating our own target) ---
    FLAG_AHEAD_ALIGN = 0.2   # |normalised bearing| within which a front blockage may be the flag pole
    FLAG_AHEAD_MATCH = 0.4   # m — front-LiDAR vs camera-distance agreement to treat the blockage as the flag
    FLAG_AHEAD_TICKS_MAX = int(2.5 / CONTROL_PERIOD)  # ticks the front may stay "flag ahead" without collecting before we allow circumnavigation (~2.5 s)

    # --- alignment ---
    ANGULAR_ALIGN_THRESH = 0.05
    DIST_TOLERANCE = 0.05
    DONE_TICKS = int(1.0 / CONTROL_PERIOD)   # ~1 s held aligned to declare "done"

    # --- waypoint following ---
    WAYPOINT_TOLERANCE = 0.20   # metres — distance to consider a waypoint reached
    WAYPOINT_STRIDE = 2         # keep every Nth cell from A* path
    REPLAN_INTERVAL = int(5.0 / CONTROL_PERIOD)        # ticks between periodic re-plans (~5 s)
    ASTAR_RETRY_INTERVAL = int(2.0 / CONTROL_PERIOD)   # ticks — retry A* every 2 s when it previously failed

    # --- POSITION_TO_COLLECT guards ---
    FLAG_CLOSE_CONFIRM = int(0.4 / CONTROL_PERIOD)      # consecutive ticks flag must be close+aligned before switching (~0.4 s)
    POSITION_MAX_TICKS = int(20.0 / CONTROL_PERIOD)    # max ticks in POSITION_TO_COLLECT before escaping (~20 s)

    # --- tilt recovery ---
    # Recovery speeds are deliberately DECOUPLED from the racing speeds below: this is an
    # open-loop manoeuvre (fixed-time reverse + spin), so scaling it with TURN_SPEED/
    # CREEP_SPEED would over-rotate the robot and swing it back into the same obstacle.
    # Keep these at the gentle values the recovery was originally tuned for.
    RECOVERY_TURN_SPEED = 0.4     # rad/s — spin rate while backing away from the obstacle
    RECOVERY_REVERSE_SPEED = 0.06 # m/s — reverse (and forward-advance) speed during recovery
    TILT_THRESHOLD = 0.25      # radians (~14°) — back up if pitched beyond this
    TILT_LIDAR_CLOSE = 0.8     # metres — LiDAR side reading below this counts as "seeing" the cause
    TILT_ROLL_THRESH = 0.10    # radians — minimum roll to trust as IMU direction signal
    RECOVERY_TICKS = int(2.5 / CONTROL_PERIOD)        # ticks to keep reversing after tilt clears (~2.5 s)
    TILT_ADVANCE_TICKS = int(0.1 / (RECOVERY_REVERSE_SPEED * CONTROL_PERIOD))  # ticks of forward creep after tilt recovery (~0.1 m)
    TIPPED_CELL_PENALTY = 25.0 # A* cost added to cells where tipping occurred (high but passable)
    TIPPED_INFLATE_RADIUS = 3  # inflate tipped area by this many cells in every direction

    # --- speeds ---
    EXPLORE_SPEED = 0.25
    NAV_SPEED = 0.12
    CREEP_SPEED = 0.06
    TURN_SPEED = 0.4
    ALIGN_SPEED = 0.5

    # --- gains ---
    KP_ANGULAR_NAV = 1.5
    KP_ANGULAR_ALIGN = 2.0
    KP_ANGULAR_WAYPOINT = 2.0

    # --- exploration ---
    SPIN_TICKS = int(2 * math.pi / (TURN_SPEED * CONTROL_PERIOD)) + 5
    OBSTACLE_CLEAR = 0.8   # hysteresis: stop avoiding only when front opens to this distance

    # --- circumnavigation ---
    CIRCUM_WALL_DIST = 0.45   # target gap to keep from the wall (m)
    CIRCUM_KP        = 1.8    # P-gain for wall-distance controller
    CIRCUM_MIN_TICKS = int(1.5 / CONTROL_PERIOD)     # minimum ticks before exit is allowed (~1.5 s)
    CIRCUM_MAX_TICKS = int(30.0 / CONTROL_PERIOD)    # timeout before aborting (~30 s)

    # --- tipped cells ---
    TIPPED_CELLS_TTL = int(60.0 / CONTROL_PERIOD)    # ticks before tipped cells are forgotten (~60 s)

    # --- gripper / claw ---
    # Joint order expected by gripper_controller (JointGroupPositionController):
    # [gripper_extension, right_gripper_joint, left_gripper_joint].
    # Open = fingers spread apart (left → +max, right → -max), arm held level.
    GRIPPER_OPEN  = (0.0, -0.06, 0.06)
    GRIPPER_CLOSE = (0.0,  0.0,  0.0)
    # Lift: arm pitched up (gripper_extension < 0 raises it; limit -1.57) with fingers
    # held closed. Tune the angle toward -1.57 if the flag base doesn't clear the floor.
    GRIPPER_LIFT  = (-0.6, 0.0,  0.0)

    # --- grab / lift sequence (timed, open-loop — node has no joint-state feedback) ---
    GRAB_TICKS = int(1.5 / CONTROL_PERIOD)   # ticks to let the fingers clamp before lifting (~1.5 s)
    LIFT_TICKS = int(3.0 / CONTROL_PERIOD)   # ticks to let the arm raise before declaring done (~3 s)

    # --- return to base (carrying the flag) ---
    RETURN_SPEED = 0.10     # m/s — slower than NAV_SPEED for stability with the load
    HOME_TOLERANCE = 0.30   # m — distance to home that counts as arrived
    FINAL_APPROACH_DIST = 0.7  # m — within this of home, abandon A*/obstacle-turn and creep straight in
    #   (home sits against the arena wall, so its grid cell is inside wall inflation: A* keeps
    #    returning a blocked/trivial path and the obstacle-turn recoils from the wall — an endless
    #    loop. The final-approach branch docks the robot the way POSITION_TO_COLLECT docks the flag.)
    # The carried flag is rigidly held ~0.3-0.45 m ahead and crosses the 2-D LiDAR plane,
    # so it reads as a permanent front obstacle. Any return inside this forward cone +
    # range band is our own cargo, not the world — masked out of the scan while carrying.
    CARGO_MASK_ANGLE = 0.7  # rad (~±40°) — forward half-cone occupied by the flag
    CARGO_MASK_RANGE = 0.6  # m — returns closer than this in the cone are the cargo

    def __init__(self):
        super().__init__('controle_robo')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/astar_path', 10)
        self.state_pub = self.create_publisher(String, '/robot_state', 1)
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Image, '/robot_cam/labels_map', self.camera_callback, 10)
        self.create_subscription(Pose, '/model/prm_robot/pose', self.pose_callback, 10)
        self.create_subscription(OccupancyGrid, '/grid_map', self._map_callback, 10)
        self.create_subscription(Imu, '/imu', self._imu_callback, 10)

        self.bridge = CvBridge()
        self.timer = self.create_timer(self.CONTROL_PERIOD, self.move_robot)

        # Tilt recovery
        self.robot_pitch = 0.0
        self.robot_roll = 0.0
        self._tilt_recovery_ticks = 0
        self._post_recovery_ticks: int = 0
        self._tilt_turn_dir = 0.0   # angular velocity chosen at first detection, held during recovery
        self._tipped_cells = set()  # grid cells where tipping occurred — penalised in A*

        # LiDAR sectors + full scan (needed to estimate flag distance)
        self.front_dist = float('inf')
        self.front_left_dist = float('inf')
        self.front_right_dist = float('inf')
        self._left_dist = float('inf')
        self._right_dist = float('inf')
        self.last_scan = None

        # Robot pose (full, from ground truth)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading = 0.0

        # Occupancy grid received from robot_mapper (locally inflated)
        self.grid_map = None   # np.ndarray shape (GRID_SIZE, GRID_SIZE)
        self._map_frame = 'map'

        # Flag detection
        self.flag_visible = False
        self.flag_angular_error = 0.0
        self.flag_confidence = 0
        self._flag_lost_frames = 0   # camera frames since last solid detection (debounce)
        self._flag_lost_ticks = 0    # control ticks flag has been lost while in FLAG_DETECTED
        self.last_angular_error = 0.0
        self.last_flag_bearing = None
        self.flag_pixel_height = 0       # bounding-box height of flag in camera frame (pixels)
        self.last_cam_dist_pos = float('inf')  # last reliable cam-derived distance, used in POSITION_TO_COLLECT

        # Flag world position (estimated at detection confirmation)
        self.flag_world_pos = None   # (x, y) in metres

        # A* waypoints (world coordinates)
        self.waypoints = []
        self.waypoint_idx = 0
        self._replan_ticks = 0
        self._replan_requested = False   # set when obstacle is hit; re-plan on next map update
        self._astar_retry_ticks: int = 0

        # Exploration obstacle avoidance (hysteresis)
        self._explore_avoiding = False
        self._explore_avoid_dir = 1.0

        # State machine
        self.state = State.EXPLORING

        # Exploration sub-state
        self._spin_ticks = 0
        self._spinning = True

        # Circumnavigation
        self._circumnavigating  = False
        self._circum_side       = -1     # -1 = follow right wall, +1 = follow left wall
        self._circum_start_dist = 0.0
        self._circum_ticks      = 0

        # A* / tipped cells
        self._astar_failed       = False
        self._tipped_cells_age   = 0     # ticks since last tipping event

        # Re-acquisition search
        self._search_ticks = 0
        self._flag_ahead_ticks = 0   # ticks the front blockage has been attributed to the flag

        # Position-to-collect
        self._done_ticks = 0
        self._pos_ticks = 0          # total ticks spent in POSITION_TO_COLLECT
        self._flag_close_ticks = 0   # consecutive ticks where flag is close+aligned

        # Grab / lift
        self._grab_ticks = 0
        self._lift_ticks = 0

        # Return to base
        self.home_pos = None     # (x, y) captured from the first pose reading
        self._carrying = False   # True once the flag is grabbed — enables cargo masking

        self.get_logger().info('ControleRobo ready — EXPLORING')
        self.state_pub.publish(String(data=self.state.name))

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg
        self.front_dist       = self._sector_min(msg,  -30,  30)
        self.front_left_dist  = self._sector_min(msg,   30,  90)
        self.front_right_dist = self._sector_min(msg,  -90, -30)
        self._left_dist       = self._sector_min(msg,   60, 120)
        self._right_dist      = self._sector_min(msg, -120, -60)

    def pose_callback(self, msg: Pose):
        self.robot_x = msg.position.x
        self.robot_y = msg.position.y
        if self.home_pos is None:
            self.home_pos = (self.robot_x, self.robot_y)
            self.get_logger().info(
                f'Home position recorded: ({self.robot_x:.2f}, {self.robot_y:.2f})'
            )
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_heading = math.atan2(siny, cosy)

    def camera_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if frame.ndim == 3:
            frame = frame[:, :, 0]
        h, w = frame.shape
        mask = (frame.astype(np.int32) == self.FLAG_LABEL)
        area = int(np.count_nonzero(mask))

        # Schmitt-trigger + grace-window debounce so a single noisy frame near the
        # threshold does not flip flag_visible (which would reset the confirmation FSM).
        # Acquire at MIN_FLAG_PIXELS; treat as a valid frame down to MIN_FLAG_PIXELS_LOW;
        # below that, tolerate FLAG_LOST_GRACE dropped frames before declaring loss.
        solid = area >= self.MIN_FLAG_PIXELS or (self.flag_visible and area >= self.MIN_FLAG_PIXELS_LOW)
        if solid:
            ys, xs = np.where(mask)
            cx = float(np.mean(xs))
            self._flag_lost_frames = 0
            self.flag_visible = True
            self.flag_pixel_height = int(np.max(ys) - np.min(ys)) + 1
            self.flag_angular_error = (cx - w / 2.0) / w
            angular_error_rad = self.flag_angular_error * self.CAMERA_FOV
            self.last_flag_bearing = self.robot_heading - angular_error_rad
        elif self.flag_visible and self._flag_lost_frames < self.FLAG_LOST_GRACE:
            # Brief dropout — keep the last good bearing/height and stay "visible"
            self._flag_lost_frames += 1
        else:
            self.flag_visible = False
            self.flag_pixel_height = 0

    def _imu_callback(self, msg: Imu):
        q = msg.orientation
        self.robot_pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.robot_roll = math.atan2(sinr, cosr)

    def _map_callback(self, msg: OccupancyGrid):
        self._map_frame = msg.header.frame_id
        raw = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.grid_map = self._inflate_obstacles(raw, self.OBSTACLE_INFLATE_RADIUS)
        # If a waypoint obstacle was detected, re-plan now that the map is fresh
        if self._replan_requested and (self.flag_world_pos is not None or self._carrying):
            self._replan_requested = False
            self._replan_current_goal()

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------

    def move_robot(self):
        if self._tipped_cells:
            self._tipped_cells_age += 1
            if self._tipped_cells_age >= self.TIPPED_CELLS_TTL:
                self._tipped_cells.clear()
                self._tipped_cells_age = 0
                self.get_logger().info('Tipped cells expired — memory cleared')

        # Tilt recovery is suppressed during the grab/lift sequence: the robot is
        # stationary clamping the flag, and a spurious pitch reading must not make it
        # reverse and abandon the grab.
        grabbing_or_lifting = self.state in (State.GRABBING, State.LIFTING)

        if abs(self.robot_pitch) > self.TILT_THRESHOLD and not grabbing_or_lifting:
            if self._tilt_recovery_ticks == 0:   # first detection this event — decide direction once
                self._tilt_turn_dir = self._tilt_turn_direction()
                self._record_tipped_location()
                self._tipped_cells_age = 0   # reset TTL on new tipping event
                self.get_logger().warn(
                    f'Tilt detected ({math.degrees(self.robot_pitch):.1f}°) — reversing '
                    f'with turn={self._tilt_turn_dir:+.2f}'
                )
            self._tilt_recovery_ticks = self.RECOVERY_TICKS   # keep resetting while still tilted
        if self._tilt_recovery_ticks > 0:
            self._tilt_recovery_ticks -= 1
            self._publish(-self.RECOVERY_REVERSE_SPEED, self._tilt_turn_dir)
            if self._tilt_recovery_ticks == 0:
                self._post_recovery_ticks = self.TILT_ADVANCE_TICKS
            return

        if self._post_recovery_ticks > 0:
            self._post_recovery_ticks -= 1
            self._publish(self.RECOVERY_REVERSE_SPEED, 0.0)
            if self._post_recovery_ticks == 0:
                if self.state == State.NAVIGATING_TO_FLAG and self.flag_world_pos is not None:
                    self.get_logger().info('Tilt advance done — re-planning path from new position')
                    self._plan_path_to_flag()
                elif self.state == State.RETURNING_TO_BASE:
                    self.get_logger().info('Tilt advance done — re-planning path to base from new position')
                    self._plan_path_to_base()
            return

        if self.state == State.EXPLORING:
            self._exploring()
        elif self.state == State.FLAG_DETECTED:
            self._flag_detected()
        elif self.state == State.NAVIGATING_TO_FLAG:
            self._navigating()
        elif self.state == State.POSITION_TO_COLLECT:
            self._position_to_collect()
        elif self.state == State.GRABBING:
            self._grabbing()
        elif self.state == State.LIFTING:
            self._lifting()
        elif self.state == State.RETURNING_TO_BASE:
            self._returning_to_base()

    # ------------------------------------------------------------------

    def _exploring(self):
        if self.flag_visible:
            self._go_to(State.FLAG_DETECTED)
            return

        if self._spinning:
            self._publish(0.0, self.TURN_SPEED)
            self._spin_ticks += 1
            if self._spin_ticks >= self.SPIN_TICKS:
                self._spinning = False
                self.get_logger().info('Initial spin done')
            return

        # Obstacle avoidance runs first — takes priority over bearing pursuit.
        # Hysteresis: engage when front_dist < OBSTACLE_FRONT, release only when > OBSTACLE_CLEAR.
        # Direction is decided once on first engagement and held until the front fully opens.
        if self.front_dist < self.OBSTACLE_FRONT:
            if not self._explore_avoiding:
                self._explore_avoiding = True
                self._explore_avoid_dir = self.TURN_SPEED if self.front_left_dist >= self.front_right_dist else -self.TURN_SPEED
                self.get_logger().info(
                    f'Explore: obstacle avoidance engaged — turning {"left" if self._explore_avoid_dir > 0 else "right"} '
                    f'(front={self.front_dist:.2f} m)'
                )
            self._publish(0.0, self._explore_avoid_dir)
            return
        if self._explore_avoiding:
            if self.front_dist < self.OBSTACLE_CLEAR:
                self._publish(0.0, self._explore_avoid_dir)   # still in hysteresis band
                return
            self._explore_avoiding = False   # front is fully clear — exit avoidance
            self.get_logger().info('Explore: obstacle avoidance cleared — resuming')

        # Only pursue flag bearing when not avoiding an obstacle
        if self.last_flag_bearing is not None:
            heading_err = self._angle_diff(self.last_flag_bearing, self.robot_heading)
            if abs(heading_err) > 0.05:
                angular = max(-self.TURN_SPEED, min(self.TURN_SPEED, 2.0 * heading_err))
                self._publish(0.0, angular)
                return

        self._publish(self.EXPLORE_SPEED, 0.0)

    # ------------------------------------------------------------------

    def _flag_detected(self):
        if self.flag_visible:
            self._circumnavigating = False
            self._flag_lost_ticks = 0
            # Approach while confirming: align to the flag and creep forward so the
            # blob grows above threshold and the detection stabilises, instead of
            # parking at the flicker distance. Confirmation logic is unchanged.
            err = self.flag_angular_error
            angular = max(-self.TURN_SPEED, min(self.TURN_SPEED, -self.KP_ANGULAR_ALIGN * err))
            linear = self.CREEP_SPEED if abs(err) < 0.1 else 0.0
            self._publish(linear, angular)
            self.flag_confidence += 1
            self.get_logger().debug(f'Flag confidence: {self.flag_confidence}/{self.CONFIRM_FRAMES}')
            if self.flag_confidence >= self.CONFIRM_FRAMES:
                self._astar_retry_ticks = 0
                self._go_to(State.NAVIGATING_TO_FLAG)
        elif self.front_dist < self.OBSTACLE_FRONT or self._circumnavigating:
            self._circumnavigate()
        else:
            # Sustained-loss guard: tolerate brief gaps (the debounce already smooths
            # the camera) so confidence accumulated across glimpses is not thrown away
            # on the first missed frame. Only give up after FLAG_LOST_TICKS_MAX ticks.
            self._stop()
            self._flag_lost_ticks += 1
            if self._flag_lost_ticks >= self.FLAG_LOST_TICKS_MAX:
                self.flag_confidence = max(0, self.flag_confidence - 1)
                self._flag_lost_ticks = 0
                self.get_logger().debug(f'Flag lost — confidence decaying: {self.flag_confidence}')
                if self.flag_confidence == 0:
                    self._go_to(State.EXPLORING)

    # ------------------------------------------------------------------

    def _navigating(self):
        if self.flag_visible:
            self.last_angular_error = self.flag_angular_error
            # Refine the flag world position estimate while it's visible.
            # Prefer the camera-derived distance (label-specific, not confused by
            # walls or other obstacles in the same LiDAR direction); fall back to
            # LiDAR only when pixel height is unavailable.
            if self.last_flag_bearing is not None:
                if self.flag_pixel_height > 0:
                    dist = self.FLAG_CAM_K / self.flag_pixel_height
                else:
                    dist = self._range_at_world_bearing(self.last_flag_bearing)
                if math.isfinite(dist) and dist > 0.2:
                    self.flag_world_pos = (
                        self.robot_x + dist * math.cos(self.last_flag_bearing),
                        self.robot_y + dist * math.sin(self.last_flag_bearing),
                    )
                    self.get_logger().debug(
                        f'Flag pos refined: ({self.flag_world_pos[0]:.2f}, {self.flag_world_pos[1]:.2f}), '
                        f'dist={dist:.2f} m ({"cam" if self.flag_pixel_height > 0 else "lidar"})'
                    )

        # PRIORITY 1: flag is close and aligned — hand off to fine approach.
        # Distance is estimated from the camera bounding-box height (pinhole model:
        # dist = FLAG_CAM_K / pixel_height) so it is flag-specific and immune to
        # walls or other obstacles that happen to be in the same LiDAR sector.
        cam_dist = (self.FLAG_CAM_K / self.flag_pixel_height
                    if self.flag_visible and self.flag_pixel_height > 0
                    else float('inf'))
        if (self.flag_visible
                and cam_dist < self.CLOSE_ENOUGH
                and abs(self.last_angular_error) < 0.1):
            self._flag_close_ticks += 1
            self.get_logger().debug(
                f'Flag close+aligned: {self._flag_close_ticks}/{self.FLAG_CLOSE_CONFIRM}'
            )
            if self._flag_close_ticks >= self.FLAG_CLOSE_CONFIRM:
                self._go_to(State.POSITION_TO_COLLECT)
                return
        else:
            self._flag_close_ticks = 0

        # PRIORITY 2: follow A* waypoints (if a path was computed)
        if self.waypoints and self.waypoint_idx < len(self.waypoints):
            # Periodic re-plan to incorporate new map data
            self._replan_ticks += 1 
            if self._replan_ticks >= self.REPLAN_INTERVAL and self.flag_world_pos is not None:
                self._replan_ticks = 0
                self.get_logger().info('Periodic re-plan triggered')
                self._plan_path_to_flag()
                if not self.waypoints:
                    return
            self._follow_waypoints()
            return

        # PRIORITY 3: no waypoints — fall back to visual servoing.
        # Two distinct situations:
        #   _astar_failed=True  → A* found no path; robot is still far; servo toward flag
        #                         while retrying A* periodically so it can switch to planned
        #                         navigation as soon as a path becomes available.
        #   _astar_failed=False → waypoints were consumed normally; robot is close to the
        #                         flag and just needs to close the final gap. Servo as-is.
        if self._astar_failed:
            self._astar_retry_ticks += 1
            if self._astar_retry_ticks >= self.ASTAR_RETRY_INTERVAL:
                self._astar_retry_ticks = 0
                self._plan_path_to_flag()
                if not self._astar_failed:
                    return  # path found — follow waypoints next tick
        self._visual_servo_to_flag()

    def _follow_waypoints(self):
        wx, wy = self.waypoints[self.waypoint_idx]
        dx = wx - self.robot_x
        dy = wy - self.robot_y
        dist_to_wp = math.sqrt(dx * dx + dy * dy)

        if dist_to_wp < self.WAYPOINT_TOLERANCE:
            self.waypoint_idx += 1
            self.get_logger().debug(
                f'Waypoint reached — advancing to {self.waypoint_idx}/{len(self.waypoints)}'
            )
            if self.waypoint_idx >= len(self.waypoints):
                if self.state == State.RETURNING_TO_BASE:
                    # Arrival is decided by _returning_to_base() (HOME_TOLERANCE); if the
                    # path ran out before we got there, re-plan to home from here.
                    self.get_logger().info('Return waypoints consumed — re-planning to base')
                    self._plan_path_to_base()
                    return
                # Sanity-check: only hand off when the flag is genuinely close.
                # If the A* goal was placed on a wrong LiDAR hit (wall in the flag
                # direction), waypoints end far from the real flag — re-plan instead.
                if self.flag_visible and self.flag_pixel_height > 0:
                    wp_cam_dist = self.FLAG_CAM_K / self.flag_pixel_height
                else:
                    wp_cam_dist = self.front_dist
                if wp_cam_dist < self.CLOSE_ENOUGH:
                    self.get_logger().info('All waypoints reached — switching to fine approach')
                    self._go_to(State.POSITION_TO_COLLECT)
                else:
                    # Check if the robot is already at the A* goal position.
                    # If so, re-planning produces the same trivial path — switch to visual servo instead.
                    astar_goal_close = False
                    if self.flag_world_pos is not None:
                        flag_x, flag_y = self.flag_world_pos
                        bearing_to_flag = math.atan2(flag_y - self.robot_y, flag_x - self.robot_x)
                        goal_wx = flag_x - self.CLOSE_ENOUGH * math.cos(bearing_to_flag)
                        goal_wy = flag_y - self.CLOSE_ENOUGH * math.sin(bearing_to_flag)
                        dist_to_goal = math.sqrt((self.robot_x - goal_wx) ** 2 + (self.robot_y - goal_wy) ** 2)
                        astar_goal_close = dist_to_goal < self.WAYPOINT_TOLERANCE
                    trivial_path = len(self.waypoints) <= 2
                    if astar_goal_close or trivial_path:
                        self.get_logger().warn(
                            f'All waypoints reached but flag still {wp_cam_dist:.2f} m away — '
                            f'{"trivial path" if trivial_path else "already at A* goal"}, '
                            f'switching to visual servo'
                        )
                        self.waypoints = []
                    else:
                        self.get_logger().warn(
                            f'All waypoints reached but flag still {wp_cam_dist:.2f} m away — re-planning'
                        )
                        self._plan_path_to_flag()
            return

        target_bearing = math.atan2(dy, dx)
        heading_err = self._angle_diff(target_bearing, self.robot_heading)

        # Obstacle CLOSER than A* planned for in the direction we are heading: request a
        # re-plan on the next map update. Uses PATH_BLOCKED_DIST (not OBSTACLE_FRONT) so we
        # don't reject A*'s own pillar-threading corridors — see PATH_BLOCKED_DIST note.
        if self.front_dist < self.PATH_BLOCKED_DIST and abs(heading_err) < 0.5:
            self.get_logger().info('Obstacle on waypoint path — turning toward free space while waiting for re-plan')
            self._replan_requested = True
            turn_dir = 1.0 if self.front_left_dist >= self.front_right_dist else -1.0
            self._publish(0.0, turn_dir * self.TURN_SPEED)
            return

        angular = max(-self.TURN_SPEED, min(self.TURN_SPEED, self.KP_ANGULAR_WAYPOINT * heading_err))
        # Move forward only when roughly aligned with the waypoint; slower under load.
        cruise = self.RETURN_SPEED if self.state == State.RETURNING_TO_BASE else self.NAV_SPEED
        linear = cruise if abs(heading_err) < 0.3 else 0.0
        self._publish(linear, angular)

    def _visual_servo_to_flag(self):
        """Fallback navigation used when A* has no path (map not ready or flag unreachable)."""
        if self.flag_visible:
            self._search_ticks = 0

        # The flag pole reads as a LiDAR obstacle once we close in. Don't circumnavigate
        # our own target: if the flag is visible, roughly centred (so it's inside the front
        # LiDAR sector), and the camera-derived distance agrees with the front reading, the
        # thing ahead IS the flag — keep servoing straight so PRIORITY 1 hands off to collect.
        # A genuine obstacle nearer than the flag reads front_dist << cam_dist and still triggers.
        cam_dist = (self.FLAG_CAM_K / self.flag_pixel_height
                    if self.flag_visible and self.flag_pixel_height > 0 else float('inf'))
        flag_ahead = (self.flag_visible
                      and abs(self.last_angular_error) < self.FLAG_AHEAD_ALIGN
                      and abs(self.front_dist - cam_dist) < self.FLAG_AHEAD_MATCH)

        # Stall watchdog: an obstacle sitting at the flag's range, beside the pole, would
        # also satisfy flag_ahead. If we keep suppressing circumnavigation but never reach
        # the collect handoff, something is really in the way — release the suppression.
        if flag_ahead:
            self._flag_ahead_ticks += 1
        else:
            self._flag_ahead_ticks = 0
        stalled = self._flag_ahead_ticks > self.FLAG_AHEAD_TICKS_MAX

        if (self.front_dist < self.OBSTACLE_FRONT or self._circumnavigating) and (not flag_ahead or stalled):
            self._circumnavigate()
            return

        self._circumnavigating = False

        if not self.flag_visible:
            self._reacquire_flag()
            return

        self._search_ticks = 0
        err = self.last_angular_error
        angular = max(-self.TURN_SPEED, min(self.TURN_SPEED, -self.KP_ANGULAR_NAV * err))
        linear = self.NAV_SPEED if abs(err) < 0.15 else 0.0
        self._publish(linear, angular)

    def _circumnavigate(self):
        if not self._circumnavigating:
            # More open on left  → turn left → obstacle on RIGHT → follow right wall (side=-1)
            # More open on right → turn right → obstacle on LEFT → follow left wall  (side=+1)
            self._circum_side = -1 if self.front_left_dist >= self.front_right_dist else +1
            if self.flag_world_pos is not None:
                dx = self.flag_world_pos[0] - self.robot_x
                dy = self.flag_world_pos[1] - self.robot_y
                self._circum_start_dist = math.sqrt(dx * dx + dy * dy)
            else:
                self._circum_start_dist = float('inf')
            self._circum_ticks = 0
            self._circumnavigating = True
            self.get_logger().info(
                f'Circumnavigation started — following '
                f'{"right" if self._circum_side == -1 else "left"} wall, '
                f'dist_to_flag={self._circum_start_dist:.2f} m'
            )

        self._circum_ticks += 1

        if self._circum_ticks > self.CIRCUM_MAX_TICKS:
            self.get_logger().warn('Circumnavigation timeout — aborting')
            self._circumnavigating = False
            return

        # Exit: front clear, min time elapsed, and closer to flag than at start
        if self._circum_ticks >= self.CIRCUM_MIN_TICKS and self.front_dist > self.OBSTACLE_CLEAR:
            if self.flag_world_pos is not None:
                dx = self.flag_world_pos[0] - self.robot_x
                dy = self.flag_world_pos[1] - self.robot_y
                if math.sqrt(dx * dx + dy * dy) < self._circum_start_dist:
                    self.get_logger().info('Circumnavigation complete — resumed approach')
                    self._circumnavigating = False
                    return
            else:
                self.get_logger().info('Circumnavigation complete — front clear')
                self._circumnavigating = False
                return

        # Front still blocked: turn sharply in circumnavigation direction
        if self.front_dist < self.OBSTACLE_FRONT:
            # side=-1 (right wall): turn left (+); side=+1 (left wall): turn right (-)
            self._publish(0.0, -self._circum_side * self.TURN_SPEED)
            return

        # Wall-following P-controller
        wall_dist = self._right_dist if self._circum_side == -1 else self._left_dist
        angular = -self._circum_side * self.CIRCUM_KP * (self.CIRCUM_WALL_DIST - wall_dist)
        angular = max(-self.TURN_SPEED, min(self.TURN_SPEED, angular))
        self._publish(self.NAV_SPEED * 0.5, angular)

    def _reacquire_flag(self):
        if self.last_flag_bearing is None:
            self._go_to(State.EXPLORING)
            return
        self._search_ticks += 1
        if self._search_ticks == 1:
            self.get_logger().info(
                f'Re-acquisition started — sweeping toward bearing {math.degrees(self.last_flag_bearing):.1f}°'
            )
        if self._search_ticks > self.SEARCH_TICKS_MAX:
            self.get_logger().info('Re-acquisition failed — back to EXPLORING')
            self.last_flag_bearing = None
            self._go_to(State.EXPLORING)
            return
        heading_err = self._angle_diff(self.last_flag_bearing, self.robot_heading)
        if abs(heading_err) > 0.05:
            angular = max(-self.TURN_SPEED, min(self.TURN_SPEED, 2.0 * heading_err))
            self._publish(0.0, angular)
        else:
            sweep = 1.57 * math.sin(self._search_ticks * 0.2)
            target = self.last_flag_bearing + sweep
            err = self._angle_diff(target, self.robot_heading)
            angular = max(-self.TURN_SPEED, min(self.TURN_SPEED, 2.0 * err))
            self._publish(0.0, angular)

    # ------------------------------------------------------------------

    def _position_to_collect(self):
        self._pos_ticks += 1
        if self._pos_ticks >= self.POSITION_MAX_TICKS:
            self.get_logger().warn('POSITION_TO_COLLECT timeout — clearing flag estimate and resuming navigation')
            self.flag_world_pos = None   # force fresh LiDAR-based re-estimation on next entry
            self._astar_retry_ticks = 0
            self._go_to(State.NAVIGATING_TO_FLAG)
            return

        if self.flag_visible:
            self.last_angular_error = self.flag_angular_error
            if self.flag_pixel_height > 0:
                self.last_cam_dist_pos = self.FLAG_CAM_K / self.flag_pixel_height
        err = self.last_angular_error
        if abs(err) > self.ANGULAR_ALIGN_THRESH:
            angular = max(-self.TURN_SPEED, min(self.TURN_SPEED, -self.KP_ANGULAR_ALIGN * err))
            self._publish(0.0, angular)
            self._done_ticks = 0
            return
        dist_err = self.last_cam_dist_pos - self.TARGET_DIST
        if abs(dist_err) > self.DIST_TOLERANCE:
            self._publish(self.CREEP_SPEED if dist_err > 0 else -self.CREEP_SPEED, 0.0)
            self._done_ticks = 0
            return
        self._stop()
        self._done_ticks += 1
        if self._done_ticks >= self.DONE_TICKS:
            if not self.flag_visible:
                # Distance and angle are correct but the flag is not in view —
                # this is an obstacle, not the flag. Back out and re-navigate.
                self.get_logger().warn('Stopped but flag not visible — obstacle, not flag. Re-navigating.')
                self._go_to(State.NAVIGATING_TO_FLAG)
                return
            self.get_logger().info(
                f'FLAG REACHED — distance={self.front_dist:.2f} m, '
                f'angle_error={err:.3f} rad'
            )
            self._go_to(State.GRABBING)

    # ------------------------------------------------------------------

    def _grabbing(self):
        """Hold position and let the claw clamp the pole, then lift."""
        self._stop()
        self._grab_ticks += 1
        if self._grab_ticks >= self.GRAB_TICKS:
            self._go_to(State.LIFTING)

    def _lifting(self):
        """Hold position and raise the arm, then the task is complete."""
        self._stop()
        self._lift_ticks += 1
        if self._lift_ticks >= self.LIFT_TICKS:
            self.get_logger().info('FLAG LIFTED — returning to base')
            self._go_to(State.RETURNING_TO_BASE)

    # ------------------------------------------------------------------

    def _returning_to_base(self):
        """Drive the explored map back to the recorded home position, carrying the flag.

        Mirrors _navigating() but the goal is home and there is no camera servoing.
        The carried flag is masked out of the LiDAR (see _sector_min), so A* is the
        primary navigator and the front sector reflects real obstacles only.
        """
        if self.home_pos is None:
            self._stop()
            return

        hx, hy = self.home_pos
        dist_home = math.hypot(hx - self.robot_x, hy - self.robot_y)

        # PRIORITY 1: arrived home — stop, drop the flag, end the mission.
        if dist_home < self.HOME_TOLERANCE:
            self._stop()
            self._open_gripper()        # release the flag at the base
            self._carrying = False      # flag delivered — disable cargo masking
            self.get_logger().info(
                f'BASE REACHED — flag delivered at ({self.robot_x:.2f}, {self.robot_y:.2f})'
            )
            self.timer.cancel()
            return

        # PRIORITY 1.5: final approach. Close to home the goal cell is usually inside wall
        # inflation, so A* returns a blocked/trivial path and _follow_waypoints recoils from
        # the base wall forever. Within FINAL_APPROACH_DIST, bypass A* and obstacle avoidance:
        # align to home and creep straight in until HOME_TOLERANCE (accepting the nearby wall,
        # exactly as the flag approach accepts the flag pole as the target, not an obstacle).
        if dist_home < self.FINAL_APPROACH_DIST:
            self.waypoints = []   # abandon the stale/blocked path
            bearing_home = math.atan2(hy - self.robot_y, hx - self.robot_x)
            heading_err = self._angle_diff(bearing_home, self.robot_heading)
            angular = max(-self.TURN_SPEED, min(self.TURN_SPEED, self.KP_ANGULAR_WAYPOINT * heading_err))
            linear = self.CREEP_SPEED if abs(heading_err) < 0.3 else 0.0
            self._publish(linear, angular)
            return

        # PRIORITY 2: follow A* waypoints, re-planning periodically
        if self.waypoints and self.waypoint_idx < len(self.waypoints):
            self._replan_ticks += 1
            if self._replan_ticks >= self.REPLAN_INTERVAL:
                self._replan_ticks = 0
                self.get_logger().info('Periodic re-plan (return) triggered')
                self._plan_path_to_base()
                if not self.waypoints:
                    return
            self._follow_waypoints()
            return

        # PRIORITY 3: no path — retry A* periodically while creeping toward home.
        if self._astar_failed:
            self._astar_retry_ticks += 1
            if self._astar_retry_ticks >= self.ASTAR_RETRY_INTERVAL:
                self._astar_retry_ticks = 0
                self._plan_path_to_base()
                if not self._astar_failed:
                    return  # path found — follow it next tick
        self._reactive_to_home(hx, hy)

    def _reactive_to_home(self, hx: float, hy: float):
        """Fallback used only when A* has no path: head toward home, masked-LiDAR avoidance."""
        # Obstacle ahead (cargo already masked): turn toward the more open side.
        if self.front_dist < self.OBSTACLE_FRONT:
            turn_dir = 1.0 if self.front_left_dist >= self.front_right_dist else -1.0
            self._publish(0.0, turn_dir * self.TURN_SPEED)
            return

        bearing_home = math.atan2(hy - self.robot_y, hx - self.robot_x)
        heading_err = self._angle_diff(bearing_home, self.robot_heading)
        angular = max(-self.TURN_SPEED, min(self.TURN_SPEED, self.KP_ANGULAR_WAYPOINT * heading_err))
        linear = self.RETURN_SPEED if abs(heading_err) < 0.3 else 0.0
        self._publish(linear, angular)

    # ------------------------------------------------------------------
    # A* path planning
    # ------------------------------------------------------------------

    def _plan_path_to_flag(self):
        if self.grid_map is None or self.flag_world_pos is None:
            return

        # Place the A* goal CLOSE_ENOUGH metres in front of the flag (toward the
        # robot), not on the flag pole itself.  The flag pole is marked as an
        # obstacle in the map and the inflation radius extends OBSTACLE_INFLATE_RADIUS
        # cells (0.4 m) beyond it, so a goal placed on or inside that zone is always
        # unreachable.  CLOSE_ENOUGH (0.7 m) safely clears the inflation zone and
        # matches the distance at which _navigating() hands off to POSITION_TO_COLLECT.
        flag_x, flag_y = self.flag_world_pos
        bearing_to_flag = math.atan2(flag_y - self.robot_y, flag_x - self.robot_x)
        goal_wx = flag_x - self.CLOSE_ENOUGH * math.cos(bearing_to_flag)
        goal_wy = flag_y - self.CLOSE_ENOUGH * math.sin(bearing_to_flag)
        goal = self._world_to_grid(goal_wx, goal_wy)
        self.get_logger().debug(
            f'A* goal set to ({goal_wx:.2f}, {goal_wy:.2f}) — {self.CLOSE_ENOUGH} m '
            f'in front of flag at ({flag_x:.2f}, {flag_y:.2f})'
        )
        self._plan_path_to(goal)

    def _plan_path_to_base(self):
        if self.grid_map is None or self.home_pos is None:
            return
        goal = self._world_to_grid(*self.home_pos)
        self.get_logger().debug(
            f'A* goal set to home ({self.home_pos[0]:.2f}, {self.home_pos[1]:.2f})'
        )
        self._plan_path_to(goal)

    def _replan_current_goal(self):
        """Re-plan to whichever goal the current state is pursuing."""
        if self.state == State.RETURNING_TO_BASE:
            self._plan_path_to_base()
        else:
            self._plan_path_to_flag()

    def _plan_path_to(self, goal: tuple):
        """Plan an A* path from the robot's cell to grid cell `goal` and publish it."""
        if self.grid_map is None:
            return

        start = self._world_to_grid(self.robot_x, self.robot_y)

        # Safety net: if the goal cell is blocked (e.g. it sits in an inflation zone or
        # on an obstacle), search outward for the nearest free cell.
        g_gx, g_gy = goal
        if self._grid_in_bounds(g_gx, g_gy) and self.grid_map[g_gy, g_gx] == 100:
            found = False
            for radius in range(1, 6):
                for dy in range(-radius, radius + 1):
                    for dx in range(-radius, radius + 1):
                        nx, ny = g_gx + dx, g_gy + dy
                        if self._grid_in_bounds(nx, ny) and self.grid_map[ny, nx] != 100:
                            goal = (nx, ny)
                            found = True
                            break
                    if found:
                        break
                if found:
                    break
            if found:
                self.get_logger().info(
                    f'Goal cell ({g_gx},{g_gy}) blocked — shifted to nearest free cell {goal}'
                )
            else:
                self.get_logger().warn(
                    f'Goal cell ({g_gx},{g_gy}) blocked and no free cell found within 5-cell radius'
                )

        path_cells = self._astar(start, goal)
        if not path_cells:
            self.get_logger().warn('A* found no path — falling back to visual servoing')
            self.waypoints = []
            self._astar_failed = True
            return

        self._astar_failed = False
        # Thin the path: keep every WAYPOINT_STRIDE-th cell plus the final goal
        sampled = path_cells[::self.WAYPOINT_STRIDE]
        if sampled[-1] != path_cells[-1]:
            sampled.append(path_cells[-1])
        self.waypoints = [self._grid_to_world(gx, gy) for gx, gy in sampled]
        self.waypoint_idx = 0
        self.get_logger().info(
            f'A* path: {len(path_cells)} cells → {len(self.waypoints)} waypoints'
        )

        now = self.get_clock().now().to_msg()
        path_msg = Path()
        path_msg.header.frame_id = self._map_frame
        path_msg.header.stamp = now
        for gx, gy in path_cells:
            wx, wy = self._grid_to_world(gx, gy)
            ps = PoseStamped()
            ps.header.frame_id = self._map_frame
            ps.header.stamp = now
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.position.z = 0.05   # raised above map plane to prevent Z-fighting in RViz
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)

    def _record_tipped_location(self):
        """Mark the current grid cell (inflated) as a costly area for future A* plans."""
        cx, cy = self._world_to_grid(self.robot_x, self.robot_y)
        r = self.TIPPED_INFLATE_RADIUS
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                nx, ny = cx + dx, cy + dy
                if self._grid_in_bounds(nx, ny):
                    self._tipped_cells.add((nx, ny))
        self.get_logger().warn(
            f'Tipped at grid ({cx},{cy}) — {len(self._tipped_cells)} penalised cells total'
        )

    def _tilt_turn_direction(self) -> float:
        """
        Decide which way to turn during tilt recovery.
        Tier 1 — LiDAR: if either side sector sees something close, turn away from it.
        Tier 2 — IMU roll: if nothing visible but robot is leaning, turn away from the lean.
        Tier 3 — random: symmetric situation; pick a random direction to escape the spot.
        Returns an angular velocity value (positive = left, negative = right).
        """
        left = self.front_left_dist
        right = self.front_right_dist

        # Tier 1: LiDAR can see the cause
        if left < self.TILT_LIDAR_CLOSE or right < self.TILT_LIDAR_CLOSE:
            direction = 1 if right < left else -1   # turn away from the closer side
            self.get_logger().info('Tilt direction: LiDAR')
            return direction * self.RECOVERY_TURN_SPEED

        # Tier 2: IMU roll — positive roll = leaning right (ROS: X-fwd, Y-left, Z-up)
        # → obstacle likely on right → turn left; flip sign if robot uses a different frame
        if abs(self.robot_roll) > self.TILT_ROLL_THRESH:
            direction = 1 if self.robot_roll > 0 else -1
            self.get_logger().info('Tilt direction: IMU roll')
            return direction * self.RECOVERY_TURN_SPEED

        # Tier 3: nothing visible and no clear lean — random escape
        self.get_logger().info('Tilt direction: random')
        return random.choice([-1, 1]) * self.RECOVERY_TURN_SPEED

    def _astar(self, start: tuple, goal: tuple) -> list:
        """Returns a list of (gx, gy) grid cells from start to goal, or [] if no path found."""
        s_gx, s_gy = start
        g_gx, g_gy = goal

        if not self._grid_in_bounds(s_gx, s_gy) or not self._grid_in_bounds(g_gx, g_gy):
            return []

        open_set = []
        heapq.heappush(open_set, (0.0, s_gx, s_gy))
        came_from = {}
        g_score = {(s_gx, s_gy): 0.0}

        def h(gx, gy):
            return math.sqrt((gx - g_gx) ** 2 + (gy - g_gy) ** 2)

        while open_set:
            _, cx, cy = heapq.heappop(open_set)

            if cx == g_gx and cy == g_gy:
                path = []
                pos = (cx, cy)
                while pos in came_from:
                    path.append(pos)
                    pos = came_from[pos]
                path.append((s_gx, s_gy))
                path.reverse()
                return path

            for dx, dy in [(-1, -1), (-1, 0), (-1, 1), (0, -1),
                           (0, 1), (1, -1), (1, 0), (1, 1)]:
                nx, ny = cx + dx, cy + dy
                if not self._grid_in_bounds(nx, ny):
                    continue
                cell_val = self.grid_map[ny, nx]
                if cell_val == 100:
                    continue
                step = math.sqrt(2) if dx != 0 and dy != 0 else 1.0
                penalty = 0.3 if cell_val < 0 else 0.0
                penalty += self.TIPPED_CELL_PENALTY if (nx, ny) in self._tipped_cells else 0.0
                tentative_g = g_score[(cx, cy)] + step + penalty
                if tentative_g < g_score.get((nx, ny), float('inf')):
                    came_from[(nx, ny)] = (cx, cy)
                    g_score[(nx, ny)] = tentative_g
                    heapq.heappush(open_set, (tentative_g + h(nx, ny), nx, ny))

        return []

    # ------------------------------------------------------------------
    # Grid helpers
    # ------------------------------------------------------------------

    def _world_to_grid(self, wx: float, wy: float) -> tuple:
        half = self.GRID_SIZE * self.GRID_RESOLUTION / 2.0
        gx = int((wx + half) / self.GRID_RESOLUTION)
        gy = int((wy + half) / self.GRID_RESOLUTION)
        return gx, gy

    def _grid_to_world(self, gx: int, gy: int) -> tuple:
        half = self.GRID_SIZE * self.GRID_RESOLUTION / 2.0
        wx = gx * self.GRID_RESOLUTION - half + self.GRID_RESOLUTION / 2.0
        wy = gy * self.GRID_RESOLUTION - half + self.GRID_RESOLUTION / 2.0
        return wx, wy

    def _grid_in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.GRID_SIZE and 0 <= gy < self.GRID_SIZE

    def _inflate_obstacles(self, raw: np.ndarray, radius: int) -> np.ndarray:
        """Mark cells within `radius` of any occupied cell as occupied (safety margin)."""
        result = raw.copy()
        for gy, gx in np.argwhere(raw == 100):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    ny, nx = int(gy) + dy, int(gx) + dx
                    if 0 <= ny < raw.shape[0] and 0 <= nx < raw.shape[1]:
                        result[ny, nx] = 100
        return result

    def _range_at_world_bearing(self, world_bearing: float) -> float:
        """Return the LiDAR range in the direction of world_bearing (metres)."""
        if self.last_scan is None:
            return float('inf')
        msg = self.last_scan
        robot_angle = self._angle_diff(world_bearing, self.robot_heading)
        # _angle_diff returns [-pi, pi]. Scans that start at 0 (0-to-2pi) have no
        # negative indices, so wrap negative angles into the [0, 2pi] range.
        if robot_angle < msg.angle_min:
            robot_angle += 2 * math.pi
        idx = round((robot_angle - msg.angle_min) / msg.angle_increment)
        idx = max(0, min(len(msg.ranges) - 1, idx))
        # Average a small window for noise robustness
        window = 3
        valid = []
        for i in range(idx - window, idx + window + 1):
            i_c = max(0, min(len(msg.ranges) - 1, i))
            r = msg.ranges[i_c]
            if math.isfinite(r) and r > 0.05:
                valid.append(r)
        return min(valid) if valid else float('inf')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _sector_min(self, msg: LaserScan, start_deg: float, end_deg: float) -> float:
        start = math.radians(start_deg)
        end = math.radians(end_deg)
        vals = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.05:
                continue
            a = msg.angle_min + i * msg.angle_increment
            a = (a + math.pi) % (2 * math.pi) - math.pi
            # Mask the carried flag: forward cone, short range — it's our cargo, not the world.
            if self._carrying and abs(a) < self.CARGO_MASK_ANGLE and r < self.CARGO_MASK_RANGE:
                continue
            if start <= a <= end:
                vals.append(r)
        return min(vals) if vals else float('inf')

    def _angle_diff(self, target: float, current: float) -> float:
        """Signed shortest-path angle from current to target, in [-pi, pi]."""
        diff = target - current
        return (diff + math.pi) % (2 * math.pi) - math.pi

    def _publish(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def _stop(self):
        self._publish(0.0, 0.0)

    def _set_gripper(self, positions):
        """Send an absolute position command to the claw's three joints."""
        self.gripper_pub.publish(Float64MultiArray(data=[float(p) for p in positions]))

    def _open_gripper(self):
        self._set_gripper(self.GRIPPER_OPEN)

    def _close_gripper(self):
        self._set_gripper(self.GRIPPER_CLOSE)

    def _go_to(self, new_state: State):
        self.get_logger().info(f'{self.state.name} → {new_state.name}')
        self.state = new_state
        self.state_pub.publish(String(data=new_state.name))
        if new_state == State.EXPLORING:
            self.flag_confidence = 0
            self._search_ticks = 0
        elif new_state == State.FLAG_DETECTED:
            # Do NOT reset flag_confidence here: glimpses must persist across
            # EXPLORING↔FLAG_DETECTED bounces so the confirmation vote can build.
            # (EXPLORING entry above is the genuine "gave up" reset.)
            self._flag_lost_ticks = 0
            self._circumnavigating = False
        elif new_state == State.NAVIGATING_TO_FLAG:
            self._search_ticks = 0
            self._circumnavigating = False
            self._replan_ticks = 0
            self._flag_close_ticks = 0
            self._flag_ahead_ticks = 0
            self._replan_requested = False
            self.waypoints = []
            self.waypoint_idx = 0
            if self.flag_world_pos is not None:
                # Already estimated the flag position — just re-plan A*
                self.get_logger().info('Re-planning A* to known flag position')
                self._plan_path_to_flag()
            elif self.last_flag_bearing is not None:
                # First time entering: estimate flag position from camera (label-specific)
                # or fall back to LiDAR if camera height is unavailable.
                if self.flag_pixel_height > 0:
                    dist = self.FLAG_CAM_K / self.flag_pixel_height
                    self.get_logger().info(
                        f'Flag distance from camera: {dist:.2f} m (pixel_height={self.flag_pixel_height})'
                    )
                else:
                    dist = self._range_at_world_bearing(self.last_flag_bearing)
                if math.isfinite(dist) and dist > 0.2:
                    fx = self.robot_x + dist * math.cos(self.last_flag_bearing)
                    fy = self.robot_y + dist * math.sin(self.last_flag_bearing)
                    self.get_logger().info(
                        f'Flag at ({fx:.2f}, {fy:.2f}), dist={dist:.2f} m'
                    )
                else:
                    far = 8.0
                    fx = self.robot_x + far * math.cos(self.last_flag_bearing)
                    fy = self.robot_y + far * math.sin(self.last_flag_bearing)
                    self.get_logger().warn(
                        f'Flag distance unknown — using projected goal ({fx:.1f}, {fy:.1f})'
                    )
                self.flag_world_pos = (fx, fy)
                self._plan_path_to_flag()
        elif new_state == State.POSITION_TO_COLLECT:
            self._done_ticks = 0
            self._pos_ticks = 0
            self.last_cam_dist_pos = float('inf')
            # Open the claw for the final approach so it's ready to capture the flag.
            self._open_gripper()
            self.get_logger().info('Claw opened for collection')
        elif new_state == State.GRABBING:
            self._stop()
            self._grab_ticks = 0
            self._carrying = True   # cargo masking active from here on
            self._close_gripper()
            self.get_logger().info('Closing claw on flag pole')
        elif new_state == State.LIFTING:
            self._stop()
            self._lift_ticks = 0
            self._set_gripper(self.GRIPPER_LIFT)
            self.get_logger().info('Lifting flag')
        elif new_state == State.RETURNING_TO_BASE:
            self._carrying = True
            self._circumnavigating = False
            self._replan_ticks = 0
            self._replan_requested = False
            self._astar_failed = False
            self._astar_retry_ticks = 0
            self.waypoints = []
            self.waypoint_idx = 0
            self.get_logger().info('Returning to base with flag')
            self._plan_path_to_base()


def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
