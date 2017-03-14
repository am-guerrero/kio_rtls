#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' I heard "%s"', data.data)

def listener():
    rospy.init_node('kio_rtls_listener', anonymous=True)

    if len(sys.argv) != 2:
        rospy.logerr("Usage: rosrun kio_rtls kio_rtls_String_listener.py TAG")
        sys.exit(0)

    tag = str(sys.argv[1]) # Tag to listen
    rospy.loginfo("Listening to tag %s" % tag)

    rospy.Subscriber('/kio/String/' + tag + '/out', String, callback)
    rospy.Subscriber('/kio/String/' + tag + '/err', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
