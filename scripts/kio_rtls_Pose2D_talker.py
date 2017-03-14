#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import subprocess
import fcntl
import math
import rospy, rospkg
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Pose2D

def talker():
    rospy.init_node('kio_rtls_Pose2D_talker', anonymous=True)
       
    rospack = rospkg.RosPack() # get an instance of RosPack with the default search paths
    rate = rospy.Rate(4) # 4 hz
    
    host        = rospy.get_param('~host', "127.0.0.1")
    port1       = rospy.get_param('~port1', "3016")
    port2       = rospy.get_param('~port2', "3017")
    device1     = rospy.get_param('~device1', "/dev/ttyACM0")
    device2     = rospy.get_param('~device2', "/dev/ttyACM1")
    baudrate    = rospy.get_param('~baudrate', "230400")
    path        = rospy.get_param('~path', rospack.get_path('kio_rtls') + "/bin") # rangingEngine path
    
    os.chdir(path)
    
    cmd_user = []
    cmd_user.append("./rangingEngine %s %s %s %s" % (host, port1, device1, baudrate))
    cmd_user.append("./rangingEngine %s %s %s %s" % (host, port2, device2, baudrate))
    
    rospy.loginfo("Current working directory: " + os.getcwd())
    rospy.loginfo("execute commands: " + str(cmd_user))

    # change the buffer mode of the subprocess-shell to linebufferd
    cmd_stdbuf = "stdbuf -o L"

    processes = []
    for item in cmd_user:
        cmd = cmd_stdbuf + " " + item
        process = subprocess.Popen(cmd.split(" "), stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.PIPE, bufsize=1)
        rospy.loginfo("started process with pid " + str(process.pid))
        processes.append(process)

    pubOUT = rospy.Publisher("/kio/Pose2D/out", Pose2D, queue_size=10)
    pubERR = rospy.Publisher("/kio/err", String, queue_size=10)

    # putting readline of stdout and stderr into non-blocking mode
    for process in processes:
        fd_out = process.stdout.fileno()
        fl = fcntl.fcntl(fd_out, fcntl.F_GETFL)
        fcntl.fcntl(fd_out, fcntl.F_SETFL, fl | os.O_NONBLOCK)

        fd_err = process.stderr.fileno()
        fl = fcntl.fcntl(fd_err, fcntl.F_GETFL)
        fcntl.fcntl(fd_err, fcntl.F_SETFL, fl | os.O_NONBLOCK)
    
    while not rospy.is_shutdown():
        # Working with STDOUT
        try:
            stdouts = {}
            for process in processes:
                stdouts[str(process.pid)] = process.stdout.readline().lstrip('\r').rstrip('\n')
    
            points = []
            for pid, stdout in stdouts.items():
                if len(stdout) > 0:
                    data = stdout.split(' ')
                    if data[0] == "003c":
                        tagId = data[9]
                        point = PointStamped()
                        point.point.x = float(data[15])
                        point.point.y = float(data[16])
                        point.point.z = float(data[17])
                        point.header.stamp = rospy.get_rostime()
                        point.header.frame_id = "kio_frame"
                        
                        points.append(point)
                        
                        rospy.loginfo("OUT[%s]: Tag=%s: %s" % (pid, tagId, stdout))
                    else:
                        rospy.loginfo("OUT[%s]: %s" % (pid, stdout))
            
            if len(points) == 2:
                rospy.loginfo("Origin:      x=%5.2f, y=%5.2f, timestamp=%s" % (points[0].point.x, points[0].point.y, str(points[0].header.stamp)))
                rospy.loginfo("Destination: x=%5.2f, y=%5.2f, timestamp=%s" % (points[1].point.x, points[1].point.y, str(points[1].header.stamp)))
                rospy.loginfo("Dif=%s" % (str(points[0].header.stamp -points[1].header.stamp)))
                
                pose2d = Pose2D()
                pose2d.x = points[1].point.x
                pose2d.y = points[1].point.y
                
                pose2d.theta = math.atan((points[1].point.y - points[0].point.y) / (points[1].point.x - points[0].point.x))                
                rospy.loginfo("Pose2D:      x=%5.2f, y=%5.2f, theta=%s" % (pose2d.x, pose2d.y, str(pose2d.theta)))
                
                #pose2d.theta = math.atan2(points[1].point.y - points[0].point.y, points[1].point.x - points[0].point.x)
                #rospy.loginfo("Pose2D:      x=%5.2f, y=%5.2f, theta2=%s" % (pose2d.x, pose2d.y, str(pose2d.theta)))

                try:
                    pubOUT.publish(pose2d)
                except Exception as e:
                    rospy.logerr("Error publishing Pose2D: " + str(e))
                    pass
                                
        except Exception as e:
            if "Errno 11" not in str(e): # Avoid "[Errno 11] Resource temporarily unavailable" error messages when stdout is empty
                rospy.logerr("OUT: " + str(e))
            pass
        
        # Working with STDERR
        try:
            stderrs = {}
            for process in processes:
                stderrs[str(process.pid)] = process.stderr.readline().lstrip('\r').rstrip('\n')

            for pid, stderr in stderrs.items():
                if len(stderr) > 0:
                    try:
                        pubERR.publish(String(stderr))
                    except Exception as e:
                        rospy.logerr("Error publishing stderr: " + str(e))
                        pass
                    
        except Exception as e:
            if "Errno 11" not in str(e): # Avoid "[Errno 11] Resource temporarily unavailable" error messages when stderr is empty
                rospy.logerr("ERR: " + str(e))
            pass

        rate.sleep()

    ret = process.returncode
    rospy.signal_shutdown('finish')
    sys.exit(ret)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
