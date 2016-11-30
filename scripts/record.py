#!/usr/bin/env python
import sys
import time

import rospy

import numpy as np
import cv2

def main():
    rospy.init_node('record', anonymous=False)

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
        # time now
        now = rospy.Time.now().to_sec()

        # Print out the current time for the camera to record
        if not pause:
            sys.stdout.write("\r")
            sys.stdout.write("{:2f}".format(now)) 
            sys.stdout.flush()

        # Capture frame-by-frame
        ret, frame = cap.read()

        # Capture frame count to calculate stats with
        i += 1
        times.append(rospy.Time.now().to_sec() - now)

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

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()