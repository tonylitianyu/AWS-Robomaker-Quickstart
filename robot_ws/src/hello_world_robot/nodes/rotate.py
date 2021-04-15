#!/usr/bin/env python
"""
Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy of this
software and associated documentation files (the "Software"), to deal in the Software
without restriction, including without limitation the rights to use, copy, modify,
merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""


from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import rospy


class Rotator():

    def __init__(self):
        self._cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.laserscan_callback)
        self.front_range = 100.0
        self.rear_range = 100.0
        self.forward = True
        
    
    def laserscan_callback(self, msg):
        self.front_range = msg.ranges[0]
        self.rear_range = msg.ranges[len(msg.ranges)/2]
        rospy.loginfo('Rotating robot: %s', self.front_range)
        

    def rotate_forever(self):
        self.twist = Twist()

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.front_range < 0.50:
                self.forward = False
            elif self.rear_range < 0.50:
                self.forward = True
            
            if self.forward:
                self.twist.linear.x = 0.3
            else:
                self.twist.linear.x = -0.3
            self._cmd_pub.publish(self.twist)

            r.sleep()


def main():
    rospy.init_node('rotate')
    try:
        rotator = Rotator()
        rotator.rotate_forever()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
