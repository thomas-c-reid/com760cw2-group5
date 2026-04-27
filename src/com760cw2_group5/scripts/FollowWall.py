#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from com760cw2_group5.srv import SwitchBehavior, SwitchBehaviorResponse


class FollowWall:

    def __init__(self):
        self.active = False

        # Laser scan regions (populated by callback)
        self.regions = {
            'right': float('inf'),
            'front_right': float('inf'),
            'front': float('inf'),
            'front_left': float('inf'),
            'left': float('inf'),
        }

        self.d = 0.5    # distance to consider a wall "close"
        self.d_too_close = 0.25  # distance at which robot must back up

        # Publishers
        self.pub_vel = rospy.Publisher('/group5Bot/cmd_vel', Twist, queue_size=1)

        # Subscribers
        self.sub_laser = rospy.Subscriber('/group5Bot/laser/scan', LaserScan, self.callback_laser)

        # Service to activate/deactivate this behavior
        self.srv = rospy.Service('wall_follower_switch', SwitchBehavior, self.callback_switch)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if not self.active:
                rate.sleep()
                continue

            msg = self.decide_action()
            self.pub_vel.publish(msg)
            rate.sleep()

    def callback_laser(self, msg):
        # 360 samples from -π to +π.  Index 0 = -180° (behind),
        # 90 = -90° (right), 180 = 0° (front), 270 = +90° (left).
        self.regions = {
            'right':       min(min(msg.ranges[54:126]), 3.5),    # -126° to -54°  (right)
            'front_right': min(min(msg.ranges[126:162]), 3.5),   # -54°  to -18°  (front-right)
            'front':       min(min(msg.ranges[162:198]), 3.5),   # -18°  to +18°  (front)
            'front_left':  min(min(msg.ranges[198:234]), 3.5),   # +18°  to +54°  (front-left)
            'left':        min(min(msg.ranges[234:306]), 3.5),   # +54°  to +126° (left)
        }

    def callback_switch(self, req):
        self.active = req.active
        if not self.active:
            vel = Twist()
            self.pub_vel.publish(vel)
        return SwitchBehaviorResponse(success=True)

    def decide_action(self):
        """Right-hand rule wall following with backup when too close."""
        d = self.d
        r = self.regions

        msg = Twist()

        # If too close to a wall in front, back up straight first
        if r['front'] < self.d_too_close:
            msg.linear.x = -0.15
            msg.angular.z = 0.3   # back up AND gently rotate to free the robot
            return msg

        front_clear = r['front'] > d
        front_right_clear = r['front_right'] > d
        right_clear = r['right'] > d

        if front_clear and front_right_clear and right_clear:
            # Case 1: Nothing nearby — turn right to find a wall
            msg.linear.x = 0.2
            msg.angular.z = -0.3

        elif front_clear and front_right_clear and not right_clear:
            # Case 2: Wall on right only — ideal, follow it
            msg.linear.x = 0.3
            msg.angular.z = 0.0

        elif front_clear and not front_right_clear and right_clear:
            # Case 3: Wall on front-right — drift left slightly
            msg.linear.x = 0.2
            msg.angular.z = 0.15

        elif front_clear and not front_right_clear and not right_clear:
            # Case 4: Wall on front-right and right — go straight
            msg.linear.x = 0.3
            msg.angular.z = 0.0

        elif not front_clear and front_right_clear and right_clear:
            # Case 5: Wall ahead only — turn left
            msg.linear.x = 0.0
            msg.angular.z = 0.5

        elif not front_clear and front_right_clear and not right_clear:
            # Case 6: Wall ahead and right — turn left
            msg.linear.x = 0.0
            msg.angular.z = 0.5

        elif not front_clear and not front_right_clear and right_clear:
            # Case 7: Wall ahead and front-right — turn left
            msg.linear.x = 0.0
            msg.angular.z = 0.5

        elif not front_clear and not front_right_clear and not right_clear:
            # Case 8: Boxed in on front/front-right/right — turn left hard
            msg.linear.x = 0.0
            msg.angular.z = 0.5

        return msg


if __name__ == '__main__':
    rospy.init_node('follow_wall')
    FollowWall()
    rospy.spin()
