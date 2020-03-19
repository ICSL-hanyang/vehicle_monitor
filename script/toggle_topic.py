#! /usr/bin/env python

import rospy
from std_msgs.msg import String

rospy.init_node("toggle")
pub = rospy.Publisher("test", String, queue_size=10)
rate = rospy.Rate(10)

rospy.set_param("button", 0)

while not rospy.is_shutdown():
    var = rospy.get_param("button");
    if var == 1:
        msg = "start"
    else:
        msg = "stop"
    pub.publish(msg)
    rate.sleep()

rospy.spin()