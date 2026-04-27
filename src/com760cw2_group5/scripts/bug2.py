#!/usr/bin/env python3

import rospy
import math
import os
import rospkg
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty, Trigger, TriggerResponse
from tf import transformations
from com760cw2_group5.msg import RobotStatus
from com760cw2_group5.srv import (SwitchBehavior, SetGoal, SetGoalResponse,
                                   SetBugStatus, SetBugStatusResponse)


# Bug 2 algorithm states
STATE_GO_TO_POINT = 0
STATE_FOLLOW_WALL = 1
STATE_GOAL_REACHED = 2


class Bug2:

    def __init__(self):
        # Current pose
        self.position = Point()
        self.yaw = 0.0

        # Goal
        self.goal = Point()
        self.goal.x = rospy.get_param('~goal_x', 5.0)
        self.goal.y = rospy.get_param('~goal_y', 0.0)

        # Start position (recorded on first model_states callback)
        self.start = None

        # State
        self.state = STATE_GO_TO_POINT
        self.wall_hits = 0

        # Metrics tracking
        self.run_start_time = None
        self.start_recorded = None
        self.distance_traveled = 0.0
        self.last_position = None
        self.run_count = 0
        self.metrics_file = os.path.join(
            rospkg.RosPack().get_path('com760cw2_group5'), 'metrics.csv')

        # Configurable parameters (can be changed via SetBugStatus service)
        self.linear_speed = 0.3
        self.wall_follow_direction = 'right'  # 'right' or 'left' hand rule

        # Laser data
        self.front_obstacle = False
        self.obstacle_threshold = 0.5

        # M-line leave point (where the robot left the M-line to follow a wall)
        self.leave_point = None
        self.leave_time = None          # time when wall-follow started
        self.dist_threshold = 0.5       # close enough to goal
        self.mline_threshold = 0.3      # close enough to M-line
        self.leave_point_buffer = 0.15  # min distance from leave point before re-leaving
        self.leave_cooldown = 3.0       # seconds before robot can leave wall after hitting

        # Publishers
        self.pub_status = rospy.Publisher('/group5Bot/status', RobotStatus, queue_size=1)
        self.pub_vel = rospy.Publisher('/group5Bot/cmd_vel', Twist, queue_size=1)

        # Robot model name in Gazebo
        self.model_name = 'group5Bot'

        # Subscribers — use Gazebo ground-truth pose instead of /odom
        # to avoid odometry drift when wheels slip against walls
        self.sub_model_states = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback_model_states)
        self.sub_laser = rospy.Subscriber('/group5Bot/laser/scan', LaserScan, self.callback_laser)

        # Services
        self.srv_set_goal = rospy.Service('set_goal', SetGoal, self.callback_set_goal)
        self.srv_set_bug_status = rospy.Service('set_bug_status', SetBugStatus, self.callback_set_bug_status)
        self.srv_reset = rospy.Service('reset_bug2', Trigger, self.callback_reset)

        # Wait for behavior services to be available
        rospy.loginfo('[Bug2] Waiting for go_to_point_switch service...')
        rospy.wait_for_service('go_to_point_switch')
        rospy.loginfo('[Bug2] Waiting for wall_follower_switch service...')
        rospy.wait_for_service('wall_follower_switch')

        self.srv_go_to_point = rospy.ServiceProxy('go_to_point_switch', SwitchBehavior)
        self.srv_wall_follower = rospy.ServiceProxy('wall_follower_switch', SwitchBehavior)

        # Publish initial parameters so other nodes can read them
        rospy.set_param('/bug2/linear_speed', float(self.linear_speed))
        rospy.set_param('/bug2/wall_follow_direction', self.wall_follow_direction)

        # Start with go-to-point active
        self.activate_go_to_point()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.run()
            self.publish_status()
            rate.sleep()

    # ── Callbacks ──────────────────────────────────────────────────────────

    def callback_model_states(self, msg):
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            return  # model not yet spawned
        self.position = msg.pose[idx].position
        quaternion = (
            msg.pose[idx].orientation.x,
            msg.pose[idx].orientation.y,
            msg.pose[idx].orientation.z,
            msg.pose[idx].orientation.w,
        )
        euler = transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]

        # Track distance traveled
        if self.last_position is not None and self.state != STATE_GOAL_REACHED:
            dx = self.position.x - self.last_position.x
            dy = self.position.y - self.last_position.y
            self.distance_traveled += math.sqrt(dx * dx + dy * dy)
        self.last_position = Point()
        self.last_position.x = self.position.x
        self.last_position.y = self.position.y

        if self.start is None:
            self.start = Point()
            self.start.x = self.position.x
            self.start.y = self.position.y

    def callback_laser(self, msg):
        # Check if there is an obstacle directly in front
        # Scan runs from -π (index 0 = behind) to +π (index n-1 = behind)
        # Forward is at index n//2 (180 for 360 samples)
        n = len(msg.ranges)
        center = n // 2
        front_ranges = msg.ranges[center - 36:center + 36]
        self.front_obstacle = min(front_ranges) < self.obstacle_threshold

    def callback_set_goal(self, req):
        self.goal.x = req.x
        self.goal.y = req.y
        self.state = STATE_GO_TO_POINT
        self.wall_hits = 0
        self.leave_point = None
        self.activate_go_to_point()
        rospy.loginfo('[Bug2] New goal set: (%.2f, %.2f)', req.x, req.y)
        return SetGoalResponse(success=True, message='Goal set')

    def callback_set_bug_status(self, req):
        """Control bug behaviour via SetBugStatus service.

        flag:      True to enable/resume, False to pause the algorithm
        speed:     Linear speed for GoToPoint (m/s, 0.1 - 1.0)
        direction: Wall-follow hand rule — 'right' or 'left'
        message:   Logged for debugging
        """
        if req.message:
            rospy.loginfo('[Bug2] SetBugStatus: %s', req.message)

        if req.speed > 0:
            self.linear_speed = max(0.1, min(1.0, req.speed))
            rospy.set_param('/bug2/linear_speed', float(self.linear_speed))
            rospy.loginfo('[Bug2] Speed set to %.2f m/s', self.linear_speed)

        if req.direction in ('right', 'left'):
            self.wall_follow_direction = req.direction
            rospy.set_param('/bug2/wall_follow_direction', req.direction)
            rospy.loginfo('[Bug2] Wall-follow direction set to %s', req.direction)

        if req.flag:
            if self.state == STATE_GOAL_REACHED:
                self.state = STATE_GO_TO_POINT
                self.activate_go_to_point()
        else:
            self.deactivate_all()

        return SetBugStatusResponse(
            success=True,
            response='Bug status updated: speed=%.2f dir=%s flag=%s' % (
                self.linear_speed, self.wall_follow_direction, req.flag))

    # ── State machine ─────────────────────────────────────────────────────

    def run(self):
        if self.start is None:
            return  # waiting for first pose update

        # Start metrics timer on first active cycle
        if self.run_start_time is None and self.state == STATE_GO_TO_POINT:
            self.run_start_time = rospy.get_time()
            self.start_recorded = Point()
            self.start_recorded.x = self.start.x
            self.start_recorded.y = self.start.y

        dist_to_goal = self.distance(self.position, self.goal)

        if self.state == STATE_GOAL_REACHED:
            # Keep publishing zero velocity to prevent drift
            self.pub_vel.publish(Twist())
            return

        if dist_to_goal < self.dist_threshold:
            self.state = STATE_GOAL_REACHED
            self.deactivate_all()
            rospy.loginfo('[Bug2] Goal reached!')
            self.log_metrics()
            return

        if self.state == STATE_GO_TO_POINT:
            if self.front_obstacle:
                # Hit a wall — switch to wall following
                self.wall_hits += 1
                self.leave_point = Point()
                self.leave_point.x = self.position.x
                self.leave_point.y = self.position.y
                self.leave_time = rospy.get_time()
                self.state = STATE_FOLLOW_WALL
                self.activate_wall_follower()
                rospy.loginfo('[Bug2] Obstacle hit (#%d), switching to wall following', self.wall_hits)

        elif self.state == STATE_FOLLOW_WALL:
            # Check if we are back on the M-line and closer to goal than the leave point
            if self.leave_point is not None:
                dist_to_mline = self.distance_to_mline(self.position)
                dist_leave_to_goal = self.distance(self.leave_point, self.goal)
                dist_from_leave = self.distance(self.position, self.leave_point)

                # Log wall-follow state for monitoring
                rospy.loginfo_throttle(1.0,
                    '[Bug2] FOLLOW_WALL  d_mline=%.2f  d_goal=%.2f  d_leave_goal=%.2f  d_from_leave=%.2f  front_clear=%s',
                    dist_to_mline, dist_to_goal, dist_leave_to_goal, dist_from_leave,
                    not self.front_obstacle)

                # Only leave the wall if:
                #  - on the M-line, closer to goal than leave point, moved away from leave point
                #  - AND the front is currently clear (so we don't immediately re-hit the wall)
                time_since_leave = rospy.get_time() - self.leave_time if self.leave_time else 0
                if (dist_to_mline < self.mline_threshold and
                        dist_to_goal < dist_leave_to_goal and
                        dist_from_leave > self.leave_point_buffer and
                        time_since_leave > self.leave_cooldown and
                        not self.front_obstacle):
                    self.state = STATE_GO_TO_POINT
                    self.activate_go_to_point()
                    rospy.loginfo('[Bug2] M-line re-intersected, switching to go-to-point')

    # ── Reset service ─────────────────────────────────────────────────────

    def callback_reset(self, req):
        rospy.loginfo('[Bug2] Reset requested')
        try:
            # Pause physics
            rospy.wait_for_service('/gazebo/pause_physics', timeout=5)
            pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
            pause()

            # Reset robot pose
            rospy.wait_for_service('/gazebo/set_model_state', timeout=5)
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            model_state = ModelState()
            model_state.model_name = self.model_name
            model_state.pose.position.x = -5.0
            model_state.pose.position.y = 0.0
            model_state.pose.position.z = 0.1
            model_state.pose.orientation.x = 0.0
            model_state.pose.orientation.y = 0.0
            model_state.pose.orientation.z = 0.0
            model_state.pose.orientation.w = 1.0
            model_state.reference_frame = 'world'
            set_state(model_state)

            # Reset internal state — set position to known reset pose so
            # stale model_states data doesn't trigger a false goal_reached
            self.position = Point()
            self.position.x = -5.0
            self.position.y = 0.0
            self.state = STATE_GO_TO_POINT
            self.wall_hits = 0
            self.leave_point = None
            self.leave_time = None
            self.start = None
            self.distance_traveled = 0.0
            self.last_position = None
            self.run_start_time = None
            self.start_recorded = None
            self.front_obstacle = False

            # Activate go_to_point
            self.activate_go_to_point()

            # Unpause physics
            rospy.wait_for_service('/gazebo/unpause_physics', timeout=5)
            unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
            unpause()

            rospy.loginfo('[Bug2] Reset complete')
            return TriggerResponse(success=True, message='Reset complete')
        except Exception as e:
            rospy.logerr('[Bug2] Reset failed: %s', str(e))
            return TriggerResponse(success=False, message=str(e))

    # ── Metrics logging ──────────────────────────────────────────────────

    def log_metrics(self):
        if self.run_start_time is None or self.start_recorded is None:
            rospy.logwarn('[Bug2] Cannot log metrics: no run start recorded')
            return

        elapsed_time = rospy.get_time() - self.run_start_time
        straight_line = self.distance(self.start_recorded, self.goal)
        path_efficiency = straight_line / self.distance_traveled if self.distance_traveled > 0 else 0.0
        speed = self.distance_traveled / elapsed_time if elapsed_time > 0 else 0.0
        self.run_count += 1

        # Write CSV
        write_header = not os.path.exists(self.metrics_file)
        with open(self.metrics_file, 'a') as f:
            if write_header:
                f.write('run,algorithm,time_s,distance_m,straight_line_m,efficiency,wall_hits,speed\n')
            f.write('%d,bug2,%.2f,%.2f,%.2f,%.4f,%d,%.4f\n' % (
                self.run_count, elapsed_time, self.distance_traveled,
                straight_line, path_efficiency, self.wall_hits, speed))

        rospy.loginfo('[Bug2] Run %d metrics: time=%.2fs dist=%.2fm straight=%.2fm '
                      'efficiency=%.2f%% wall_hits=%d speed=%.2fm/s',
                      self.run_count, elapsed_time, self.distance_traveled,
                      straight_line, path_efficiency * 100, self.wall_hits, speed)

    # ── Behavior switching ────────────────────────────────────────────────

    def activate_go_to_point(self):
        self.srv_wall_follower(False)
        # Set the goal so GoToPoint reads it when activated
        rospy.set_param('/bug2/goal_x', float(self.goal.x))
        rospy.set_param('/bug2/goal_y', float(self.goal.y))
        self.srv_go_to_point(True)

    def activate_wall_follower(self):
        self.srv_go_to_point(False)
        self.srv_wall_follower(True)

    def deactivate_all(self):
        self.srv_go_to_point(False)
        self.srv_wall_follower(False)

    # ── Helpers ───────────────────────────────────────────────────────────

    @staticmethod
    def distance(p1, p2):
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

    def distance_to_mline(self, point):
        """Perpendicular distance from a point to the M-line (start→goal)."""
        if self.start is None:
            return float('inf')
        # Line from start to goal: ax + by + c = 0
        dx = self.goal.x - self.start.x
        dy = self.goal.y - self.start.y
        length = math.sqrt(dx ** 2 + dy ** 2)
        if length < 1e-6:
            return self.distance(point, self.start)
        # Coefficients of the line equation
        a = -dy
        b = dx
        c = dy * self.start.x - dx * self.start.y
        return abs(a * point.x + b * point.y + c) / length

    def publish_status(self):
        msg = RobotStatus()
        if self.state == STATE_GO_TO_POINT:
            msg.current_state = 'go_to_point'
        elif self.state == STATE_FOLLOW_WALL:
            msg.current_state = 'follow_wall'
        else:
            msg.current_state = 'goal_reached'
        msg.distance_to_goal = self.distance(self.position, self.goal)
        msg.x_position = self.position.x
        msg.y_position = self.position.y
        msg.yaw = self.yaw
        msg.wall_hits = self.wall_hits
        msg.wall_follow_direction = self.wall_follow_direction
        msg.linear_speed = self.linear_speed
        self.pub_status.publish(msg)


if __name__ == '__main__':
    rospy.init_node('bug2')
    Bug2()
    rospy.spin()
