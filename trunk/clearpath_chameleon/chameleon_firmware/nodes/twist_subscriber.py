#!/usr/bin/python

import roslib; roslib.load_manifest('chameleon_firmware')
import rospy

from geometry_msgs.msg import Twist
from chameleon_msgs.msg import ChameleonCmdSpeed
from math import fabs, copysign


class TwistSubscriber:
    def __init__(self):
        rospy.init_node('chameleon_twist')

	# diameter of circle traced by wheels, in meters.
        self.track = rospy.get_param('~track', 0.26)
        self.max_speed = rospy.get_param('~max_speed', 0.5)

        self.cmd_pub = rospy.Publisher('cmd_speed', ChameleonCmdSpeed)
        rospy.Subscriber("cmd_vel", Twist, self.callback)    
        rospy.spin()
	    

    def callback(self, twist):
        """ Receive twist message, formulate and send Chameleon speed msg. """
        cmd = ChameleonCmdSpeed()
        cmd.left = twist.linear.x - (twist.angular.z * self.track / 2)
        cmd.right = twist.linear.x + (twist.angular.z * self.track / 2) 

        # When a max speed is exceeded, maintain ratio of left/right
        if fabs(cmd.left) > self.max_speed:
            cmd.right = cmd.right * self.max_speed / fabs(cmd.left)
            cmd.left = copysign(self.max_speed, cmd.left)
        if fabs(cmd.right) > self.max_speed:
            cmd.left = cmd.left * self.max_speed / fabs(cmd.right)
            cmd.right = copysign(self.max_speed, cmd.right)

	self.cmd_pub.publish(cmd)


if __name__ == "__main__": TwistSubscriber()
