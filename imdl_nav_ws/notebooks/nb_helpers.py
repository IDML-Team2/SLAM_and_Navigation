"""Helper functions for the imdl_nav notebooks.

The notebooks import everything from here, so they can focus on the ideas.
Read this file if you want to know how something works in detail.

Sections:
    files and paths
    ROS: receiving messages, robot pose, sending goals
    laser scans
    maps (occupancy grids)
    plotting
"""

import math
import os
import sys
import time

import numpy as np
import yaml

# ---------------------------------------------------------------------------
# Files and paths
# ---------------------------------------------------------------------------

NOTEBOOK_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_DIR = os.path.dirname(NOTEBOOK_DIR)
PACKAGE_DIR = os.path.join(WORKSPACE_DIR, 'src', 'imdl_nav')
ROBOT_YAML = os.path.join(PACKAGE_DIR, 'config', 'robot.yaml')
SAMPLE_DIR = os.path.join(NOTEBOOK_DIR, 'sample_data')
SAMPLE_MAP = os.path.join(SAMPLE_DIR, 'sample_map.yaml')
SAMPLE_SCAN = os.path.join(SAMPLE_DIR, 'sample_scan.yaml')
MY_MAP = os.path.expanduser('~/maps/my_map.yaml')


def load_robot_config(path=ROBOT_YAML):
    """Read config/robot.yaml (lidar position and robot size in cm / degrees)."""
    with open(path) as f:
        return yaml.safe_load(f)


def robot_outer_radius(config):
    """Largest distance (m) from the robot center to its outline, incl. safety margin.

    Same rule as navigation.launch.py: radius_cm, or the farthest corner of
    footprint_cm if an outline is given.
    """
    robot = config['robot']
    footprint = robot.get('footprint_cm') or []
    if footprint:
        size_cm = max(math.hypot(x, y) for x, y in footprint)
    else:
        size_cm = robot['radius_cm']
    return (size_cm + robot['safety_margin_cm']) / 100.0


# ---------------------------------------------------------------------------
# ROS: receiving messages, robot pose, sending goals
# ---------------------------------------------------------------------------

_node = None
_tf_buffer = None
_tf_listener = None


def ros_node():
    """Start ROS (once) and return the notebook's node."""
    global _node
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    if not rclpy.ok():
        # Without rclpy's SIGINT handler, the notebook's "stop" button raises
        # KeyboardInterrupt but ROS keeps running (e.g. to cancel a goal).
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    if _node is None:
        _node = rclpy.create_node('notebook')
    return _node


def receive_one(topic, msg_type, qos=10, timeout_sec=5.0):
    """Subscribe to a topic, wait for ONE message and return it."""
    import rclpy
    node = ros_node()
    received = []
    subscription = node.create_subscription(msg_type, topic, received.append, qos)
    end = time.monotonic() + timeout_sec
    while not received and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(subscription)
    if not received:
        raise TimeoutError(
            f'No message on {topic} within {timeout_sec} s. Is the robot running '
            '(robot.launch.py or mapping.launch.py)? Same ROS_DOMAIN_ID?')
    return received[0]


def lookup_pose(target='map', source='base_link', timeout_sec=5.0):
    """Where is `source` in `target`? Returns (x, y, yaw_deg) from TF."""
    import rclpy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    global _tf_buffer, _tf_listener
    node = ros_node()
    if _tf_buffer is None:
        _tf_buffer = Buffer()
        _tf_listener = TransformListener(_tf_buffer, node, spin_thread=False)
    start = time.monotonic()
    while time.monotonic() < start + timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.05)
        # Listen for a moment first, so the buffer holds fresh data and not
        # what was received the last time this function ran.
        if time.monotonic() - start > 0.5 and _tf_buffer.can_transform(target, source, Time()):
            t = _tf_buffer.lookup_transform(target, source, Time()).transform
            return t.translation.x, t.translation.y, math.degrees(quaternion_to_yaw(t.rotation))
    raise TimeoutError(f'No transform {target} -> {source}. Is localization (or SLAM) running?')


def nav_client_class():
    """Return the NavClient class from the imdl_nav package (src/imdl_nav/imdl_nav/nav_client.py)."""
    try:
        from imdl_nav.nav_client import NavClient
    except ImportError:          # workspace not sourced: use the source folder directly
        sys.path.insert(0, PACKAGE_DIR)
        from imdl_nav.nav_client import NavClient
    return NavClient


def yaw_to_quaternion(yaw):
    """Rotation about the vertical axis (rad) -> quaternion (x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_to_yaw(q):
    """Quaternion (object with .x .y .z .w) -> rotation about the vertical axis (rad)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ---------------------------------------------------------------------------
# Laser scans
# ---------------------------------------------------------------------------

def scan_from_msg(msg):
    """sensor_msgs/msg/LaserScan -> plain dict (easy to save and to compute with)."""
    return {
        'frame_id': msg.header.frame_id,
        'angle_min': msg.angle_min,
        'angle_increment': msg.angle_increment,
        'range_min': msg.range_min,
        'range_max': msg.range_max,
        'ranges': np.array(msg.ranges, dtype=float),
    }


def save_scan(scan, path):
    """Save a scan dict to a YAML file (e.g. to work on it at home)."""
    data = dict(scan)
    data['ranges'] = [float(r) for r in scan['ranges']]
    with open(path, 'w') as f:
        yaml.safe_dump(data, f)


def load_scan(path=SAMPLE_SCAN):
    """Load a scan dict saved with save_scan()."""
    with open(path) as f:
        data = yaml.safe_load(f)
    data['ranges'] = np.array(data['ranges'], dtype=float)
    return data


def scan_angles(scan):
    """Angle (rad) of every reading."""
    return scan['angle_min'] + np.arange(len(scan['ranges'])) * scan['angle_increment']


def scan_to_points(scan):
    """Valid readings as points (x, y) in the lidar's own frame ('laser')."""
    r = scan['ranges']
    a = scan_angles(scan)
    valid = np.isfinite(r) & (r >= scan['range_min']) & (r <= scan['range_max'])
    return r[valid] * np.cos(a[valid]), r[valid] * np.sin(a[valid])


def transform_points(x, y, tx, ty, yaw):
    """Rotate points by yaw (rad), then shift them by (tx, ty)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return tx + c * x - s * y, ty + s * x + c * y


# ---------------------------------------------------------------------------
# Maps (occupancy grids, as written by map_saver_cli)
# ---------------------------------------------------------------------------

FREE, UNKNOWN, OCCUPIED = 0, 1, 2


def read_pgm(path):
    """Read a binary PGM image (P5) as a 2D array; row 0 is the TOP of the image."""
    with open(path, 'rb') as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:                       # magic, width, height, max value
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':            # comment line
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    if tokens[0] != b'P5':
        raise ValueError(f'{path}: only binary PGM (P5) is supported')
    width, height = int(tokens[1]), int(tokens[2])
    pixels = np.frombuffer(data[pos + 1:pos + 1 + width * height], dtype=np.uint8)
    return pixels.reshape(height, width)


class GridMap:
    """A saved map: the image plus the information from the .yaml file."""

    def __init__(self, yaml_path):
        with open(yaml_path) as f:
            self.info = yaml.safe_load(f)
        image_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), self.info['image'])
        self.image = read_pgm(image_path)
        self.resolution = float(self.info['resolution'])
        self.origin_x, self.origin_y = float(self.info['origin'][0]), float(self.info['origin'][1])
        self.height, self.width = self.image.shape
        self.cells = self._classify()

    def _classify(self):
        """Same rule as map_server: darker than occupied_thresh -> occupied, etc."""
        p = (255.0 - self.image) / 255.0          # 0 = white, 1 = black
        if self.info.get('negate', 0):
            p = 1.0 - p
        cells = np.full(self.image.shape, UNKNOWN, dtype=np.uint8)
        cells[p > self.info['occupied_thresh']] = OCCUPIED
        cells[p < self.info['free_thresh']] = FREE
        return cells

    def extent(self):
        """[left, right, bottom, top] in meters, for matplotlib's imshow."""
        return [self.origin_x, self.origin_x + self.width * self.resolution,
                self.origin_y, self.origin_y + self.height * self.resolution]

    def world_to_pixel(self, x, y):
        """Map coordinates (m) -> (row, column) in the image. Row 0 is the top!"""
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = self.height - 1 - int(math.floor((y - self.origin_y) / self.resolution))
        return row, col

    def pixel_to_world(self, row, col):
        """(row, column) -> map coordinates (m) of the cell CENTER."""
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (self.height - 1 - row + 0.5) * self.resolution
        return x, y

    def inside(self, row, col):
        return 0 <= row < self.height and 0 <= col < self.width

    def cell_at(self, x, y):
        """FREE, UNKNOWN or OCCUPIED at map coordinates (x, y); UNKNOWN outside the image."""
        row, col = self.world_to_pixel(x, y)
        return self.cells[row, col] if self.inside(row, col) else UNKNOWN

    def distance_to_obstacle(self, max_distance=1.0):
        """For every cell: distance (m) to the nearest occupied cell, capped at max_distance."""
        occupied = self.cells == OCCUPIED
        r = int(math.ceil(max_distance / self.resolution))
        dist = np.full(self.cells.shape, max_distance)
        h, w = occupied.shape
        for dr in range(-r, r + 1):              # look at every offset within the radius
            for dc in range(-r, r + 1):
                d = math.hypot(dr, dc) * self.resolution
                if d > max_distance:
                    continue
                shifted = np.zeros_like(occupied)
                shifted[max(dr, 0):h + min(dr, 0), max(dc, 0):w + min(dc, 0)] = \
                    occupied[max(-dr, 0):h + min(-dr, 0), max(-dc, 0):w + min(-dc, 0)]
                dist[shifted] = np.minimum(dist[shifted], d)
        return dist

    def raycast(self, x, y, angles, max_range=12.0):
        """Simulate a lidar at (x, y): distance to the first occupied cell for each angle."""
        steps = np.arange(0.0, max_range, self.resolution / 2.0)
        ranges = np.full(len(angles), np.inf)
        for i, a in enumerate(angles):
            px, py = x + steps * math.cos(a), y + steps * math.sin(a)
            cols = np.floor((px - self.origin_x) / self.resolution).astype(int)
            rows = self.height - 1 - np.floor((py - self.origin_y) / self.resolution).astype(int)
            ok = (rows >= 0) & (rows < self.height) & (cols >= 0) & (cols < self.width)
            hit = np.zeros(len(steps), dtype=bool)
            hit[ok] = self.cells[rows[ok], cols[ok]] == OCCUPIED
            if hit.any():
                ranges[i] = steps[np.argmax(hit)]
        return ranges


def goal_check(grid, distances, x, y, config):
    """Can the robot CENTER stand at (x, y)? Returns (ok, explanation).

    distances: result of grid.distance_to_obstacle(...)
    """
    outer = robot_outer_radius(config)
    inflation = config['navigation']['inflation_cm'] / 100.0
    row, col = grid.world_to_pixel(x, y)
    if not grid.inside(row, col):
        return False, f'({x:.2f}, {y:.2f}): outside the map'
    if grid.cells[row, col] == OCCUPIED:
        return False, f'({x:.2f}, {y:.2f}): inside an obstacle'
    if grid.cells[row, col] == UNKNOWN:
        return False, f'({x:.2f}, {y:.2f}): unknown area (never seen by the lidar)'
    d = distances[row, col]
    if d <= outer:
        return False, (f'({x:.2f}, {y:.2f}): {d * 100:.0f} cm from the nearest obstacle '
                       f'<= robot size {outer * 100:.0f} cm -> robot would touch it (lethal)')
    if d <= inflation:
        return True, (f'({x:.2f}, {y:.2f}): {d * 100:.0f} cm from the nearest obstacle -> '
                      f'reachable, but inside the inflation zone ({inflation * 100:.0f} cm): '
                      'robot drives slowly / may stop a bit off')
    return True, f'({x:.2f}, {y:.2f}): at least {d * 100:.0f} cm from any obstacle -> fine'


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def show_map(ax, grid, title=None):
    """Draw a GridMap like RViz: white = free, gray = unknown, black = occupied; axes in meters."""
    colors = np.array([[255, 255, 255], [180, 180, 180], [0, 0, 0]], dtype=np.uint8)
    ax.imshow(colors[grid.cells], extent=grid.extent(), origin='upper', interpolation='nearest')
    ax.plot(0, 0, 'g^', markersize=10, label='start pose (0, 0)')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_aspect('equal')
    if title:
        ax.set_title(title)


def draw_robot(ax, x, y, yaw, radius, color='tab:blue', label=None):
    """Draw the robot as a circle with an arrow for its heading (yaw in rad)."""
    import matplotlib.pyplot as plt
    ax.add_patch(plt.Circle((x, y), radius, fill=False, color=color, linewidth=2, label=label))
    ax.arrow(x, y, radius * math.cos(yaw), radius * math.sin(yaw),
             width=radius * 0.08, color=color, length_includes_head=True)
