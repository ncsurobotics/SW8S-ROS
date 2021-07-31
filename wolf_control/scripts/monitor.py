#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from curses import wrapper
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

class Monitor:
    voltage = 16.4
    simulated = True
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
        stdscr.clear()
        stdscr.nodelay(1)
        while not rospy.is_shutdown():
            stdscr.erase()
            stdscr.addstr(0, 0, 'Battery Voltage: {}V'.format(self.voltage))
            stdscr.addstr(2, 0, 'Mission State:   {}'.format(self.state))
            stdscr.addstr(0, 35, 'Current RC Rates:      Linear {:.3}, {:.3}, {:.3}; Angular: {:.3}, {:.3}, {:.3}'
                        .format(self.rcrates.linear.x, self.rcrates.linear.y, self.rcrates.linear.z, self.rcrates.angular.x, self.rcrates.angular.y, self.rcrates.angular.z))
            stdscr.addstr(2, 35, 'Current Position Goal: Linear {:.3}, {:.3}, {:.3}; Angular: {:.3}, {:.3}, {:.3}'
                        .format(self.goal.linear.x, self.goal.linear.y, self.goal.linear.z, self.goal.angular.x, self.goal.angular.y, self.goal.angular.z))

            #check if we should quit
            c = stdscr.getch()
            if c == ord('q'):
                break
            
            stdscr.refresh()
            rate.sleep()
    
if __name__ == '__main__':
    try:
        monitor = Monitor()
        wrapper(monitor.monitor_win)
    except rospy.ROSInterruptException:
        pass