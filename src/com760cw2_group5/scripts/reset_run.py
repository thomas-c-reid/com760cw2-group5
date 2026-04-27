#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger

rospy.init_node('reset_run', anonymous=True)
rospy.loginfo('Waiting for reset_bug2 service...')
rospy.wait_for_service('reset_bug2')
reset = rospy.ServiceProxy('reset_bug2', Trigger)
resp = reset()
print(resp.message)
