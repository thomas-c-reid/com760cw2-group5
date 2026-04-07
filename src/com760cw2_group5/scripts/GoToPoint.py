#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from com760cw2_group5.srv import SwitchBehavior, SwitchBehaviorResponse


class GoToPoint:

    def __init__(self):
        self.active = False

        # Robot state
        self.position = Point()
        self.yaw = 0
        # State machine: 0 - fix heading, 1 - go straight, 2 - reached destination
        self.state = 0

        # Destination — updated by bug2 via rosparam before activating
        self.desired_position = Point()
        self.desired_position.x = 0.0
        self.desired_position.y = 0.0

        # Threshold parameters
        self.yaw_threshold = math.radians(5)
        self.dist_threshold = 0.3

        # Publishers
        self.pub_vel = rospy.Publisher('/group5Bot/cmd_vel', Twist, queue_size=1)

        # Subscribers
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.callback_odom)

        # Service to activate/deactivate this behavior
        self.srv = rospy.Service('go_to_point_switch', SwitchBehavior, self.callback_switch)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if not self.active:
                rate.sleep()
                continue

            if self.state == 0:
                self.fix_heading(self.desired_position)
            elif self.state == 1:
                self.go_straight(self.desired_position)
            elif self.state == 2:
                self.done()
            else:
                rospy.logerr('Unknown state!')

            rate.sleep()

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

    def callback_switch(self, req):
        self.active = req.active
        if self.active:
            # Read the goal from rosparam (set by bug2 before calling this service)
            self.desired_position.x = rospy.get_param('/bug2/goal_x', 0.0)
            self.desired_position.y = rospy.get_param('/bug2/goal_y', 0.0)
            self.state = 0  # reset to fix heading
            rospy.loginfo('[GoToPoint] Activated — heading to (%.2f, %.2f)',
                          self.desired_position.x, self.desired_position.y)
        else:
            vel = Twist()
            self.pub_vel.publish(vel)
        return SwitchBehaviorResponse(success=True)

    def fix_heading(self, des_pos):
        desired_yaw = math.atan2(des_pos.y - self.position.y, des_pos.x - self.position.x)
        err_yaw = self.normalize_angle(desired_yaw - self.yaw)

        if abs(err_yaw) > self.yaw_threshold:
            vel = Twist()
            # Proportional control — slow down as heading approaches target
            vel.angular.z = max(-0.5, min(0.5, err_yaw * 1.0))
            self.pub_vel.publish(vel)
        else:
            self.state = 1

    def go_straight(self, des_pos):
        desired_yaw = math.atan2(des_pos.y - self.position.y, des_pos.x - self.position.x)
        err_yaw = self.normalize_angle(desired_yaw - self.yaw)
        err_pos = math.sqrt(
            (des_pos.x - self.position.x) ** 2 + (des_pos.y - self.position.y) ** 2
        )

        if err_pos <= self.dist_threshold:
            self.state = 2
        elif abs(err_yaw) > math.radians(30):
            # Only stop and re-align for large heading errors
            self.state = 0
        else:
            vel = Twist()
            vel.linear.x = 0.3
            # Gentle course correction while moving
            vel.angular.z = err_yaw * 0.5
            self.pub_vel.publish(vel)

    def done(self):
        vel = Twist()
        self.pub_vel.publish(vel)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


if __name__ == '__main__':
    rospy.init_node('go_to_point')
    GoToPoint()
    rospy.spin()
