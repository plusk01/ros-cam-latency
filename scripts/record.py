#!/usr/bin/env python
import sys
import time

import rospy

import numpy as np
import cv2

FILM_DURATION_SECS = 15

def main():
    rospy.init_node('record', anonymous=False)

    # Select the camera to measure latency of
    cap = cv2.VideoCapture(0)

    # Create an output writer to record video
    # fourcc = cv2.VideoWriter_fourcc('H','2','6','4');
    out = cv2.VideoWriter('output.avi', 0, 30.0, (640,480))

    # Load first frame to get 'init done' out of the way
    ret, frame = cap.read()
    cv2.imshow('frame', frame)

    print('Please setup of windows so that the camera '
            'feed shows both its own output and the terminal timer.')
    print('Press \'c\' when windows are situated.')

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Display the resulting frame
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('c'):
            break

    # Frame counter
    i = 0
    times = []

    # Loop for 15 seconds
    start_time = rospy.Time.now()
    duration = rospy.rostime.Duration(FILM_DURATION_SECS)

    print('\n\n')
    print('Press \'q\' in window to quit.\n')
    print("Start time: {}\nPlease record video for {} seconds".format(
                                start_time.to_sec(), FILM_DURATION_SECS))

    while(start_time+duration > rospy.Time.now()):
        # time now
        now = rospy.Time.now().to_sec()

        # Print out the current time for the camera to record
        sys.stdout.write("\r")
        sys.stdout.write("{:2f}".format(now)) 
        sys.stdout.flush()

        # Capture frame-by-frame
        ret, frame = cap.read()
        out.write(frame)

        i += 1
        times.append(rospy.Time.now().to_sec() - now)

        # Display the resulting frame
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print("\n\nAverage camera fps: {}".format((1.0*i)/np.sum(times)))

    # When everything done, release the capture
    cap.release()
    out.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()