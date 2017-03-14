#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

def callback_pointStamped(data):
    rospy.loginfo(rospy.get_caller_id() + ' I heard: frame_id = %s, seq = %d, stamp = %d secs %d nsecs, point = [%f, %f, %f]', data.header.frame_id, data.header.seq, data.header.stamp.secs, data.header.stamp.nsecs, data.point.x, data.point.y, data.point.z)

def callback_string(data):
    rospy.logerr(rospy.get_caller_id() + ' I heard "%s" on stderr', data.data)

def listener():
    rospy.init_node('kio_rtls_PointStamped_listener', anonymous=True)

    if len(sys.argv) != 2:
        rospy.logerr("Usage: rosrun kio_rtls kio_rtls_PointStamped_listener.py TAG")
        sys.exit(0)
        
    tag = str(sys.argv[1]) # Tag to listen
    rospy.loginfo("Listening to tag %s" % tag)
    
    rospy.Subscriber('/kio/PointStamped/' + tag + '/out', PointStamped, callback_pointStamped)
    rospy.Subscriber('/kio/String/' + tag + '/err', String, callback_string)

    rospy.spin() # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    listener()
