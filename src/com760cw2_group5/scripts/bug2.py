#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from com760cw2_group5.msg import RobotStatus
from com760cw2_group5.srv import SwitchBehavior, SetGoal, SetGoalResponse


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
        self.goal.y = rospy.get_param('~goal_y', 5.0)

        # Start position (recorded on first odom callback)
        self.start = None

        # State
        self.state = STATE_GO_TO_POINT
        self.wall_hits = 0

        # Laser data
        self.front_obstacle = False
        self.obstacle_threshold = 0.5

        # M-line leave point (where the robot left the M-line to follow a wall)
        self.leave_point = None
        self.dist_threshold = 0.5       # close enough to goal
        self.mline_threshold = 0.3      # close enough to M-line
        self.leave_point_buffer = 0.5   # min distance from leave point before re-leaving

        # Publishers
        self.pub_status = rospy.Publisher('/group5Bot/status', RobotStatus, queue_size=1)

        # Subscribers
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.sub_laser = rospy.Subscriber('/group5Bot/laser/scan', LaserScan, self.callback_laser)

        # Service to set a new goal
        self.srv_set_goal = rospy.Service('set_goal', SetGoal, self.callback_set_goal)

        # Wait for behavior services to be available
        rospy.loginfo('[Bug2] Waiting for go_to_point_switch service...')
        rospy.wait_for_service('go_to_point_switch')
        rospy.loginfo('[Bug2] Waiting for wall_follower_switch service...')
        rospy.wait_for_service('wall_follower_switch')

        self.srv_go_to_point = rospy.ServiceProxy('go_to_point_switch', SwitchBehavior)
        self.srv_wall_follower = rospy.ServiceProxy('wall_follower_switch', SwitchBehavior)

        # Start with go-to-point active
        self.activate_go_to_point()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.run()
            self.publish_status()
            rate.sleep()

    # ── Callbacks ──────────────────────────────────────────────────────────

    def callback_odom(self, msg):
        self.position = msg.pose.pose.position
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        euler = transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]

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

    # ── State machine ─────────────────────────────────────────────────────

    def run(self):
        if self.start is None:
            return  # waiting for first odom

        dist_to_goal = self.distance(self.position, self.goal)

        if dist_to_goal < self.dist_threshold:
            self.state = STATE_GOAL_REACHED
            self.deactivate_all()
            rospy.loginfo('[Bug2] Goal reached!')
            return

        if self.state == STATE_GO_TO_POINT:
            if self.front_obstacle:
                # Hit a wall — switch to wall following
                self.wall_hits += 1
                self.leave_point = Point()
                self.leave_point.x = self.position.x
                self.leave_point.y = self.position.y
                self.state = STATE_FOLLOW_WALL
                self.activate_wall_follower()
                rospy.loginfo('[Bug2] Obstacle hit (#%d), switching to wall following', self.wall_hits)

        elif self.state == STATE_FOLLOW_WALL:
            # Check if we are back on the M-line and closer to goal than the leave point
            if self.leave_point is not None:
                dist_to_mline = self.distance_to_mline(self.position)
                dist_leave_to_goal = self.distance(self.leave_point, self.goal)
                dist_from_leave = self.distance(self.position, self.leave_point)

                # Periodic debug so we can see why it isn't leaving the wall
                rospy.loginfo_throttle(1.0,
                    '[Bug2] FOLLOW_WALL  d_mline=%.2f  d_goal=%.2f  d_leave_goal=%.2f  d_from_leave=%.2f',
                    dist_to_mline, dist_to_goal, dist_leave_to_goal, dist_from_leave)

                if (dist_to_mline < self.mline_threshold and
                        dist_to_goal < dist_leave_to_goal and
                        dist_from_leave > self.leave_point_buffer):
                    self.state = STATE_GO_TO_POINT
                    self.activate_go_to_point()
                    rospy.loginfo('[Bug2] M-line re-intersected, switching to go-to-point')

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
        self.pub_status.publish(msg)


if __name__ == '__main__':
    rospy.init_node('bug2')
    Bug2()
    rospy.spin()
