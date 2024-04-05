#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
"""
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('red_light_green_light')
 
red_light_twist = Twist()
green_light_twist = Twist()
green_light_twist.linear.x = 0.5
 
driving_forward = False
light_change_time = rospy.Time.now()
rate = rospy.Rate(10)
 
while not rospy.is_shutdown():
    if driving_forward:
        cmd_vel_pub.publish(green_light_twist)
    else:
        cmd_vel_pub.publish(red_light_twist)
    
    if light_change_time < rospy.Time.now():
    	driving_forward = not driving_forward
    	light_change_time = rospy.Time.now() + rospy.Duration(3)
    rate.sleep()

"""
class RobotController:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('robot_controller', anonymous=True)
        # 로봇 속도를 제어하기 위한 퍼블리셔 생성
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # 속도 메시지 초기화
        self.vel_msg = Twist()

    def go(self, speed=0.5):
        self.vel_msg.linear.x = speed
        self.vel_msg.angular.z = 0
        self.vel_pub.publish(self.vel_msg)
    
    def back(self, speed=0.5):
        self.vel_msg.linear.x = -speed
        self.vel_msg.angular.z = 0
        self.vel_pub.publish(self.vel_msg)

    def left(self, speed=0.5):
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = speed
        self.vel_pub.publish(self.vel_msg)

    def right(self, speed=0.5):
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = -speed
        self.vel_pub.publish(self.vel_msg)

    def stop(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.vel_pub.publish(self.vel_msg)
        
robot = RobotController()
while True:
	for _ in range(10):
		robot.go()
	robot.stop()
	robot.right()
	for _ in range(5):
		robot.go()
	robot.stop()
	robot.left()
	robot.left()
	for _ in range(10):
		robot.back()
	robot.stop()

