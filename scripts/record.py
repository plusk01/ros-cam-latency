#!/usr/bin/env python
import sys
import time

import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rospy.init_node('record', anonymous=False)

    # OpenCV to ROS stuff
    img_pub = rospy.Publisher('img', Image, queue_size=10)
    bridge = CvBridge()

    # Select the camera to measure latency of
    cap = cv2.VideoCapture(0)

    # Load first frame to get 'init done' out of the way
    ret, frame = cap.read()
    cv2.imshow('frame', frame)

    # Bool to toggle pausing
    pause = False

    # Frame counter
    i = 0
    times = []

    while(True):
        now = rospy.Time.now().to_sec()

        # Print out the current time for the camera to record
        if not pause:
            sys.stdout.write("\r")
            sys.stdout.write("{:2f}".format(now)) 
            sys.stdout.flush()

        # Capture frame-by-frame
        ret, frame = cap.read()

        # Capture frame count to calculate stats with
        times.append(rospy.Time.now().to_sec() - now)
        i += 1

        # Publish
        img = bridge.cv2_to_imgmsg(frame, 'bgr8')
        img.header.stamp = rospy.Time.now()
        img_pub.publish(img)

        # Display the resulting frame, if not paused
        if not pause:
            cv2.imshow('frame',frame)

        # Let user interact
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        elif k == ord('p'):
            pause = not pause

    print("\n\nAverage camera fps: {}".format((1.0*i)/np.sum(times)))
    print("Num frames received: {}".format(i))

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()