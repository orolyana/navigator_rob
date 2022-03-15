#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

msg = """
Navigate your CityBot!

---------------------------
Where to visit:
+----+----+----+----+----+----+----+
|  0 |  1 |  2 |  3 |  4 |  5 |  6 |
+----+----+----+----+----+----+----+
|  7 |  8 |  9 | 10 | 11 | 12 | 13 |
+----+----+----+----+----+----+----+
| 14 | 15 | 16 | 17 | 18 | 19 | 20 |
+----+----+----+----+----+----+----+
| 21 | 22 | 23 | 24 | 25 | 26 | 27 |
+----+----+----+----+----+----+----+
| 28 | 29 | 30 | 31 | 32 | 33 | 34 |
+----+----+----+----+----+----+----+
| 35 | 36 | 37 | 38 | 39 | 40 | 41 |
+----+----+----+----+----+----+----+
| 42 | 43 | 44 | 45 | 46 | 47 | 48 |
+----+----+----+----+----+----+----+


enter a sequence of locations, separated by commas

24 is the center of the map and the starting location

press enter when ready

CTRL-C to quit
"""


class RobotUI:

        def __init__(self):

                self.navigate_vel = rospy.Publisher('/navigate_vel',String, queue_size=10)

        def publish_navigation(self,val):
            self.navigate_vel.publish(String(val))
            print("Published")
if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('city_navigator')
    navigation_ui = RobotUI()

    try:
        print(msg)
        while True:
            command = input("Navigation Destination(s):")
            command_list = command.split(',')
            error = 0
            for i in command_list:
                try:
                    if(int(i) > 48 or int(i) < 0):
                        print("Destination " + i + " is out of bounds. Please try a different destination.")
                        error = 1
                        break
                except ValueError:
                    print("Input " + i + " is not an integer. Try again.")
                    error = 1
                    break
            if(error != 1):
                print(command_list)
                navigation_ui.publish_navigation(command)
                


    except Exception as e:
        print(e)

    finally:
        #pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)