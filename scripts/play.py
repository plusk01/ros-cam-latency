#!/usr/bin/env python
import sys
import time

import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from sensor_msgs.msg import Image

# Globals
line = None
diff_ms = []

def _handle_img(msg):
    global diff_ms

    # Calculate difference in recvd vs trans time
    msg_time = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
    diff = rospy.Time.now().to_sec() - msg_time.to_sec()
    diff_ms.append(diff*1e3)

    print(diff_ms[-1])

def _animate(i):
    global diff_ms

    if len(diff_ms) > 0:
        xdata = line.get_xdata()
        ydata = line.get_ydata()

        N = len(xdata)+1

        xdata.append(N)
        ydata.append(diff_ms[-1])

        line.set_data(xdata, ydata)

        ax = plt.gca()
        ax.set_xlim(0, N)

        diff_ms = []

    return [line]

def _init():
    line.set_data([],[])
    return [line]
    

def main():
    rospy.init_node('play', anonymous=False)

    # Open an interactive plot
    fig, ax = plt.subplots()
    plt.title('Transmission latency [ms]')
    ax.set_ylim(0,10)

    global line
    line = ax.plot([],[], linewidth=0.6, animated=False)[0]
    animate = animation.FuncAnimation(fig, _animate, init_func=_init, frames=None,
                            interval=10, blit=True)

    # OpenCV to ROS stuff
    img_sub = rospy.Subscriber('img', Image, _handle_img)

    plt.show()
    rospy.spin()

if __name__ == '__main__':
    main()