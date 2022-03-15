#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Vector3
import moveit_commander
import math
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from operator import itemgetter
import graph

right_path = []
nodes = []
x = -9
while x <= 9:
    y = -9
    while y <= 9:
        nodes.append((x,y))
        y += 3
    x += 3

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw

class RobotMovement:

        def __init__(self):
            #Once everything is initialized will set to true
            self.initialized = False

            #set up Subscriber and Publishers
            #TODO link correct function for Sub
            # # set up ROS / OpenCV bridge
            self.bridge = cv_bridge.CvBridge()
            
            # initalize the debugging window
            cv2.namedWindow("window", 1)

            self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
            self.navigator = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            rospy.Subscriber("/scan", LaserScan, self.process_scan)
            rospy.Subscriber("/navigate_vel", String, self.run_sequence)

            self.tb_2_vel_pub = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size =10)
            self.odometry_2 = rospy.Subscriber("tb3_0/odom", Odometry, self.odometry_callback_2)
            self.image_sub_2 = rospy.Subscriber('tb3_0/camera/rgb/image_raw', Image, self.image_callback_2)

            self.odometry = rospy.Subscriber("/odom", Odometry, self.odometry_callback)
            self.position = None
            rospy.sleep(1)
            self.initialized = True
            
            self.stop = False
            self.detect = True
            
            self.cur_location = (0,0)

        def run_sequence(self,msg):
            msg = (msg.data)
            command_list = msg.split(',')
            destinations = []
            for coords in command_list:                
                x_val = -9 + ((int(coords) // 7) * 3)
                y_val = -9 + ((int(coords) % 7) * 3)
                destinations.append([x_val, y_val])
            tuple_destinations = [tuple(l) for l in destinations]
            g = graph.Graph(nodes)
            right_path = g.plan_path(self.cur_location,tuple_destinations)
            self.cur_location = right_path[len(right_path) - 1]
            self.navigate_graph(right_path)

        def image_callback(self, msg):

            self.view = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        def image_callback_2(self, msg):

            self.view_2 = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        def odometry_callback(self, data):
            self.position = data.pose.pose
        
        def odometry_callback_2(self, data):
            self.position_2 = data.pose.pose

        #helper function for rotating bot. used to survey the numbers on the blocks.
        def turn_around(self, angle):
            relative_angle = (angle * math.pi) / 180
            angular_speed = 0.2
            twister = Twist()
            twister.angular.z = -abs(angular_speed)
            self.navigator.publish(twister)
            current_angle = 0
            firstTime = rospy.Time.now().to_sec()
            while current_angle < relative_angle:
                curTime = rospy.Time.now().to_sec() - firstTime
                current_angle = angular_speed*(curTime)
            twister.angular.z = 0
            self.navigator.publish(twister)

        def travel(self, x, y):
            # for debugging
            print("traveling")  
            while True:
                goal = Point()
                speed = Twist()
                goal.x = x
                goal.y = y
                rot_q = self.position.orientation
                (roll, pitch,  theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
                inc_x = goal.x - self.position.position.x
                inc_y = goal.y - self.position.position.y

                angle_to_goal = math.atan2(inc_y, inc_x)
                distance = pow((pow(inc_x, 2) + pow(inc_y,2)), 0.5)
                if distance < 0.1:
                    speed.linear.x = 0.0
                    speed.angular.z = 0.0
                    self.navigator.publish(speed)
                    break
                self.detect_stopsign()
                self.navigate_around_cone()
                if (self.stop is True) and (self.detect is True):
                    speed.linear.x = 0.0
                    speed.angular.z = 0.0
                    self.navigator.publish(speed)
                    print("sleeping")
                    rospy.sleep(3)
                    print("awake")
                    self.detect = False
                    self.stop = False
                elif abs(angle_to_goal- theta) > 0.35:
                    speed.linear.x = 0.0
                    prop_control = 0.3
                    if((angle_to_goal - theta) > 3.5):
                        speed.angular.z = prop_control * -1
                    elif((angle_to_goal - theta) > -1.7 and (angle_to_goal - theta) < 0):
                        speed.angular.z = prop_control * -1
                    else:
                        speed.angular.z = prop_control
                else:
                    speed.linear.x = 0.5
                    image = self.view
                    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                    lower_yellow = numpy.array([30, 64, 127]) #TODO
                    upper_yellow = numpy.array([30, 255, 255]) #TODO
                    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

                    # this erases all pixels that aren't yellow
                    h, w, d = image.shape
                    search_top = int(3*h/4)
                    search_bot = int(3*h/4 + 20)
                    mask[0:search_top, 0:w] = 0
                    mask[search_bot:h, 0:w] = 0

                    # using moments() function, the center of the yellow pixels is determined
                    M = cv2.moments(mask)
                    # if there are any yellow pixels found
                    if M['m00'] > 0:
                            # center of the yellow pixels in the image
                            cx = int(M['m10']/M['m00'])
                            cy = int(M['m01']/M['m00'])

                            # a red circle is visualized in the debugging window to indicate
                            # the center point of the yellow pixels
                            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                            prop_control = 0.3
                            center = w/2
                            error = (center - cx)
                            speed.angular.z = prop_control * error*3.1415/180
                self.navigator.publish(speed)

        #takes in a list like: [[0,0], [0,3], [0,6], [-3,3]] and navigates thru the items one at a time
        def navigate_graph(self, graphlist):
            for pair in graphlist:
                self.travel(pair[0],pair[1])
        #publish laser scan data to self.directly_ahead node. used in cv dumbbell algorithms to approximate best position for pickup
        def process_scan(self, data):
            self.directly_ahead = data.ranges[0]

        def detect_stopsign(self):
            image = self.view
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Color Detection Range 
            lower_red = numpy.array([0, 20, 20])
            upper_red = numpy.array([0, 200, 200])
            mask = cv2.inRange(hsv,lower_red, upper_red)
            
            h, w, d = image.shape
            M = cv2.moments(mask)

            if M['m00'] > 0:
                    if M['m00'] > 270000:
                        self.stop = True
                        print("stop")
            
            if M['m00'] < 8000:
                self.detect = True
            
            # shows the debugging window
            cv2.imshow("window", image)
            cv2.waitKey(3)
        
        def avoid_robot_collision(self):
            image = self.view
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Color Detection Range 
            lower_black = numpy.array([0, 0, 0])
            upper_black = numpy.array([255, 0, 100])
            mask = cv2.inRange(hsv,lower_black, upper_black)
            
            h, w, d = image.shape
            M = cv2.moments(mask)

            if M['m00'] > 18000000:
                print("avoiding robot ", M['m00'])
                speed = Twist()
                # turn left
                speed.angular.z = 0
                speed.linear.x = 0
                self.navigator.publish(speed)
                rospy.sleep(2)
                
        def navigate_around_cone(self):
            image = self.view
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Color Detection Range 
            lower_orange = numpy.array([0, 100, 100])
            upper_orange = numpy.array([10, 255, 255])
            mask = cv2.inRange(hsv,lower_orange, upper_orange)
            
            h, w, d = image.shape
            M = cv2.moments(mask)

            if M['m00'] > 4150000:
                print("avoiding cone", M['m00'])
                speed = Twist()
                # turn left
                speed.linear.x = 0
                speed.angular.z = 0.5
                self.navigator.publish(speed)
                rospy.sleep(3)
                
                # go forward (left)
                speed.angular.z = 0
                speed.linear.x = 0.2
                self.navigator.publish(speed)
                rospy.sleep(4)
                
                # turn right
                speed.linear.x = 0
                speed.angular.z = -0.5
                self.navigator.publish(speed)
                rospy.sleep(3)
                
                # go forward
                speed.angular.z = 0
                speed.linear.x = 0.2
                self.navigator.publish(speed)
                rospy.sleep(5)
                
                # turn right
                speed.linear.x = 0
                speed.angular.z = -0.5
                self.navigator.publish(speed)
                rospy.sleep(3)
                
                # go forward (right)
                speed.angular.z = 0
                speed.linear.x = 0.2
                self.navigator.publish(speed)
                rospy.sleep(4)
                
                # turn left
                speed.linear.x = 0
                speed.angular.z = 0.5
                self.navigator.publish(speed)
                rospy.sleep(3)
                
                # continue
                speed.linear.x = 0.06
                speed.angular.z = 0
                self.navigator.publish(speed)
            else:
                self.avoid_robot_collision()

                
        def travel_2(self, x, y):
            # for debugging
            print("traveling")  
            while True:
                goal = Point()
                speed = Twist()
                goal.x = x
                goal.y = y
                rot_q = self.position_2.orientation
                (roll, pitch,  theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
                inc_x = goal.x - self.position_2.position.x
                inc_y = goal.y - self.position_2.position.y

                angle_to_goal = math.atan2(inc_y, inc_x)
                distance = pow((pow(inc_x, 2) + pow(inc_y,2)), 0.5)
                if distance < 0.1:
                    speed.linear.x = 0.0
                    speed.angular.z = 0.0
                    self.tb_2_vel_pub.publish(speed)
                    break
                if abs(angle_to_goal- theta) > 0.35:
                    speed.linear.x = 0.0
                    prop_control = 0.3
                    if((angle_to_goal - theta) > 3.5):
                        speed.angular.z = prop_control * -1
                    elif((angle_to_goal - theta) > -1.7 and (angle_to_goal - theta) < 0):
                        speed.angular.z = prop_control * -1
                    else:
                        speed.angular.z = prop_control
                else:
                    speed.linear.x = 0.5
                    image = self.view_2
                    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                    lower_yellow = numpy.array([30, 64, 127]) #TODO
                    upper_yellow = numpy.array([30, 255, 255]) #TODO
                    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

                    # this erases all pixels that aren't yellow
                    h, w, d = image.shape
                    search_top = int(3*h/4)
                    search_bot = int(3*h/4 + 20)
                    mask[0:search_top, 0:w] = 0
                    mask[search_bot:h, 0:w] = 0

                    # using moments() function, the center of the yellow pixels is determined
                    M = cv2.moments(mask)
                    # if there are any yellow pixels found
                    if M['m00'] > 0:
                            # center of the yellow pixels in the image
                            cx = int(M['m10']/M['m00'])
                            cy = int(M['m01']/M['m00'])

                            # a red circle is visualized in the debugging window to indicate
                            # the center point of the yellow pixels
                            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                            prop_control = 0.3
                            center = w/2
                            error = (center - cx)
                            speed.angular.z = prop_control * error*3.1415/180
                self.tb_2_vel_pub.publish(speed)

        def navigate_graph_2(self, graphlist):
            for pair in graphlist:
                self.travel_2(pair[0],pair[1])

        def drive_square(self):
            nodes = [(6,6), (6,9), (9,9), (9,6), (6,6)]
            while True:
                self.navigate_graph_2(nodes)

            
        #call this to make robot stoop and open grasp. ready to grip dumbbell -> needs fixing. it doesnt pick it up well...
        def run(self):
            self.drive_square()
            rospy.spin()

if __name__ == '__main__':
        rospy.init_node('robotmovement')
        robot_movement = RobotMovement()
        robot_movement.run()
