#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import subprocess
import fcntl
import rospy, rospkg
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

def talker():
    rospy.init_node('kio_rtls_PointStamped_talker', anonymous=True)
       
    rospack = rospkg.RosPack() # get an instance of RosPack with the default search paths
    rate = rospy.Rate(10) # 10hz
    
    host        = rospy.get_param('~host', "127.0.0.1")
    port        = rospy.get_param('~port', "3016")
    device      = rospy.get_param('~device', "/dev/ttyACM0")
    baudrate    = rospy.get_param('~baudrate', "230400")
    path        = rospy.get_param('~path', rospack.get_path('kio_rtls') + "/bin") # rangingEngine path
    map_x       = rospy.get_param('~map_x', 0.0)
    map_y       = rospy.get_param('~map_y', 0.0)
    map_z       = rospy.get_param('~map_z', 0.0) 

    cmd_user    = "./rangingEngine %s %s %s %s" % (host, port, device, baudrate) # rangingEngine looks for anchor.config and tag.config at current directory
   
    os.chdir(path)
    
    tags = []
    lines = open("tag.config", "r")
    for line in lines:
        data = line.split()
        tags.append(data[0])
    lines.close()

    rospy.loginfo("Current working directory: " + os.getcwd())
    rospy.loginfo("execute command: " + cmd_user)

    # change the buffer mode of the subprocess-shell to linebufferd
    cmd_stdbuf = "stdbuf -o L"
    cmd = cmd_stdbuf + " " + cmd_user

    process = subprocess.Popen(cmd.split(" "), stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.PIPE, bufsize=1)
    rospy.loginfo("started process with pid " + str(process.pid))

    pubOUTs = {}
    for tag in tags:
        pubOUTs[tag] = rospy.Publisher("/kio/PointStamped/"+tag+"/out", PointStamped, queue_size=10)
    
    pubERR = rospy.Publisher("/kio/err", String, queue_size=10)

    # putting readline of stdout and stderr into non-blocking mode
    fd_out = process.stdout.fileno()
    fl = fcntl.fcntl(fd_out, fcntl.F_GETFL)
    fcntl.fcntl(fd_out, fcntl.F_SETFL, fl | os.O_NONBLOCK)

    fd_err = process.stderr.fileno()
    fl = fcntl.fcntl(fd_err, fcntl.F_GETFL)
    fcntl.fcntl(fd_err, fcntl.F_SETFL, fl | os.O_NONBLOCK)
    
    br = tf.TransformBroadcaster()
    
    while not rospy.is_shutdown():
        try:
            stdout = process.stdout.readline().lstrip('\r').rstrip('\n')
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
                    
                    try:
                        pubOUTs[tagId].publish(point)
                    except Exception as e:
                        rospy.logerr("OUT: Error publishing. " + str(e))
                        pass

                    br.sendTransform((map_x, map_y, map_z), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "kio_frame", "map")
              
                rospy.loginfo("OUT: " + stdout)
        except Exception as e:
            if "Errno 11" not in str(e): # Avoid "[Errno 11] Resource temporarily unavailable" error messages when stdout is empty
                rospy.logerr("OUT: " + str(e))
            pass

        try:
            stderr = process.stderr.readline().lstrip('\r').rstrip('\n')
            if len(stderr) > 0:
                pubERR.publish(String(stderr))
                rospy.logerr("ERR: " + stderr)
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
