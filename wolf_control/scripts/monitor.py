#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from curses import wrapper
import curses
import curses.panel
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String, Bool

class Monitor:
    voltage = 16.4
    simulated = True
    set_pose = False
    is_kill = True
    rcrates = Twist()
    goal = Twist()
    state = "MISSION DISABLED"

    def battery_callback(self, data: BatteryState):
        self.voltage = data.voltage

    def vel_callback(self, data: Twist):
        self.rcrates = data

    def goal_callback(self, data: Twist):
        self.goal = data

    def state_callback(self, data: String):
        self.state = data.data

    def monitor_win(self, stdscr):
        rospy.init_node('monitor', anonymous=True)
        rate = rospy.Rate(10) # 10hz

        if not self.simulated:
            rospy.Subscriber('/mavros/battery', BatteryState, self.battery_callback)

        rospy.Subscriber("cmd_vel", Twist, self.vel_callback)
        rospy.Subscriber("wolf_control/goal", Twist, self.goal_callback)
        rospy.Subscriber("wolf_control/mission_state", String, self.state_callback)
        goal_pub = rospy.Publisher('wolf_control/goal', Twist, queue_size=10)
        kill_pub = rospy.Publisher('killswitch', Bool, queue_size=1)

        stdscr.clear()
        stdscr.nodelay(1)
        while not rospy.is_shutdown():
            stdscr.erase()
            stdscr.addstr(0, 0, 'Battery Voltage: {:.3}V'.format(self.voltage))
            stdscr.addstr(2, 0, 'Mission State:   {}'.format(self.state))
            stdscr.addstr(4, 0, 'Armed:           {}'.format(self.state))
            stdscr.addstr(0, 35, 'Current RC Rates:      Linear {:.3}, {:.3}, {:.3}; Angular: {:.3}, {:.3}, {:.3}'
                        .format(self.rcrates.linear.x, self.rcrates.linear.y, self.rcrates.linear.z, self.rcrates.angular.x, self.rcrates.angular.y, self.rcrates.angular.z))
            stdscr.addstr(2, 35, 'Current Position Goal: Linear {:.3}, {:.3}, {:.3}; Angular: {:.3}, {:.3}, {:.3}'
                        .format(self.goal.linear.x, self.goal.linear.y, self.goal.linear.z, self.goal.angular.x, self.goal.angular.y, self.goal.angular.z))


            #check if we should quit
            c = stdscr.getch()
            if c == ord('q'):
                break
            if c == ord('s'):
                self.set_pose = not self.set_pose
            if c == ord('k'):
                kill_pub.publish(Bool(False))
                self.is_kill = False
            if c == ord('a'):
                kill_pub.publish(Bool(True))
                self.is_kill = True

            #draw instructions on how to use the program
            if not self.set_pose:
                stdscr.addstr(5, 30, 'Press "s" to publish a message to wolf_control/goal')
                stdscr.addstr(6, 30, 'Press "k" to kill the robot')
                stdscr.addstr(7, 30, 'Press "a" to arm the robot')
                stdscr.addstr(8, 30, 'Press "q" to quit this program')
            stdscr.refresh()
            
            #draw boxes if applicable
            if self.set_pose:
                curses.echo()
                win = curses.newwin(16, 50, 5, 30)
                win.box()
                goal = Twist()
                
                win.addstr(0, 10, 'Publish to wolf_control/goal')
                win.addstr(2, 1, 'Linear X: ')
                goal.linear.x = float(win.getstr(2, 11).decode(encoding="utf-8"))
                win.addstr(4, 1, 'Linear Y: ')
                goal.linear.y = float(win.getstr(4, 11).decode(encoding="utf-8"))
                win.addstr(6, 1, 'Linear Z: ')
                goal.linear.z = float(win.getstr(6, 11).decode(encoding="utf-8"))
                win.addstr(8, 1, 'Angular X: ')
                goal.angular.x = float(win.getstr(8, 12).decode(encoding="utf-8"))
                win.addstr(10, 1, 'Angular Y: ')
                goal.angular.y = float(win.getstr(10, 12).decode(encoding="utf-8"))
                win.addstr(12, 1, 'Angular Z: ')
                goal.angular.z = float(win.getstr(12, 12).decode(encoding="utf-8"))

                win.refresh()
                curses.noecho()
                self.set_pose = False

                goal_pub.publish(goal)
                

            rate.sleep()
    
if __name__ == '__main__':
    try:
        monitor = Monitor()
        wrapper(monitor.monitor_win)
    except rospy.ROSInterruptException:
        pass
