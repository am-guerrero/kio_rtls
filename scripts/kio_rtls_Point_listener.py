#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String

def callback_point(data):
    rospy.loginfo(rospy.get_caller_id() + ' I heard [%f, %f, %f] on stdout', data.x, data.y, data.z)

def callback_string(data):
    rospy.logerr(rospy.get_caller_id() + ' I heard "%s" on stderr', data.data)
    
def listener():
    rospy.init_node('kio_rtls_Point_listener', anonymous=True)

    if len(sys.argv) != 2:
        rospy.logerr("Usage: rosrun kio_rtls kio_rtls_Point_listener.py TAG")
        sys.exit(0)

    tag = str(sys.argv[1]) # Tag to listen
    rospy.loginfo("Listening to tag %s" % tag)

    rospy.Subscriber('/kio/Point/' + tag + '/out', Point, callback_point)
    rospy.Subscriber('/kio/String/' + tag + '/err', String, callback_string)

    rospy.spin() # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    listener()
