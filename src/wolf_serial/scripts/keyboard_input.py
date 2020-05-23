#!/usr/bin/env python
import rospy
import sys
import termios
import tty
from std_msgs.msg import Float64


def getch(allowed='', print_char=True):
    """Get a single character of input from the user.

    Optional keyword arguments:
    - allowed -- string; characters to permit (default all)
    - print_char -- bool; whether to print the character (default True)
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        if allowed:
            ch = None
            while not (ch and ch in allowed + '\x03'):
                ch = sys.stdin.read(1)
        else:
            ch = sys.stdin.read(1)
        if print_char:
            sys.stdout.write(ch)
            sys.stdout.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    if ch == '\x03':
        raise KeyboardInterrupt
    return ch



def keys():
    rospy.init_node('keyboard', anonymous=False)

    vert_pub = rospy.Publisher('wolf_vertical', Float64, queue_size=10)
    lat_pub = rospy.Publisher('wolf_lateral', Float64, queue_size=10)
    rot_pub = rospy.Publisher('wolf_rotation', Float64, queue_size=10)

    while not rospy.is_shutdown():
        char = getch()

        if char == "q":
            vert_pub.publish(0.2)
        elif char == "e":
            vert_pub.publish(-0.2)

        if char == "a":
            rot_pub.publish(0.2)
        elif char == "d":
            rot_pub.publish(-0.2)

        if char == "w":
            lat_pub.publish(0.2)
        elif char == "s":
            lat_pub.publish(-0.2)

        if char == 'f':
            lat_pub.publish(0)
            vert_pub.publish(0)
            rot_pub.publish(0)
if __name__ == '__main__':
    keys()