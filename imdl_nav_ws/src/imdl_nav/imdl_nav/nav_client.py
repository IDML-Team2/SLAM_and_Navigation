"""Small Nav2 action client: "drive to (x, y, yaw)".

Used by go_to.py and waypoints.py. Runs on the laptop.

Nav2 offers the action /navigate_to_pose (type nav2_msgs/action/NavigateToPose).
An action has three parts:
    Goal      -> what we want:        the target pose
    Feedback  -> while driving:       e.g. remaining distance
    Result    -> at the end:          succeeded / canceled / aborted
"""

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'reached',
    GoalStatus.STATUS_ABORTED: 'failed (ABORTED)',
    GoalStatus.STATUS_CANCELED: 'canceled (CANCELED)',
}


def make_pose(x, y, yaw_deg, stamp):
    """Build a pose in the 'map' frame. The angle is converted to a quaternion."""
    yaw = math.radians(yaw_deg)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = stamp
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    # Rotation about the vertical axis only: x = y = 0, z = sin(yaw/2), w = cos(yaw/2)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class NavClient(Node):

    def __init__(self):
        super().__init__('nav_client')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_handle = None

    def drive_to(self, x, y, yaw_deg=0.0):
        """Send a goal and wait until Nav2 is done. Returns True on success."""
        self.get_logger().info('Waiting for Nav2 (/navigate_to_pose) ...')
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('Nav2 does not answer. Is navigation.launch.py running?')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = make_pose(x, y, yaw_deg, self.get_clock().now().to_msg())
        self.get_logger().info(f'Driving to x={x:.2f} m, y={y:.2f} m, yaw={yaw_deg:.0f} deg')

        # 1. Send the goal and wait until Nav2 accepts it
        send_future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        self.goal_handle = send_future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the goal.')
            return False

        # 2. Wait for the result. Feedback arrives while we wait.
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.goal_handle = None
        status = result_future.result().status
        self.get_logger().info(f'Result: {STATUS_NAMES.get(status, status)}')
        return status == GoalStatus.STATUS_SUCCEEDED

    def on_feedback(self, msg):
        remaining = msg.feedback.distance_remaining
        self.get_logger().info(f'  {remaining:.2f} m to go', throttle_duration_sec=1.0)

    def cancel(self):
        """Cancel the running goal (e.g. after Ctrl-C)."""
        if self.goal_handle is not None:
            self.get_logger().warn('Canceling the drive ...')
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
