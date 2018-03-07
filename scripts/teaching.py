#!/usr/bin/env python
#coding: UTF-8

from geometry_msgs.msg import Twist
from raspimouse_ros_2.msg import LightSensorValues, ButtonValues
from pfoe_rpm.msg import Event
import rospy, rosbag, rosparam
import datetime

class Teaching:
    def __init__(self):
        rospy.Subscriber('/cmd_vel', Twist, self.cmdvel_callback, queue_size = 1)
        rospy.Subscriber('/lightsensors', LightSensorValues, self.sensor_callback, queue_size = 1)
        rospy.Subscriber('/buttons', ButtonValues, self.button_callback, queue_size = 1)

        self.event_pub = rospy.Publisher('/event', Event, queue_size=100)

        self.cmd_vel = Twist()
        self.sensor_values = LightSensorValues()
        self.e = Event()
        self.mode = False
        self.bagfile_open = False

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.logger()
            rate.sleep()
    
    #モータ指令のコールバック関数
    def cmdvel_callback(self, msg):
        self.cmd_vel = msg

    #センサのコールバック関数
    def sensor_callback(self, msg):
        self.sensor_values = msg

    #ボタンのコールバック関数
    def button_callback(self, msg):
        self.mode = msg.rear_toggle

    #記録する関数
    def logger(self):
        if not self.mode:
            if self.bagfile_open:
                self.bag.close()
                self.bagfile_open = False
            return
        else:
            print ("教示中・・・")
            if not self.bagfile_open:
                name = "PFoE_" + datetime.datetime.today().strftime("%Y_%m_%d_%H_%M_%S") + ".bag"
                rosparam.set_param("/bagfile", name)
                self.bag = rosbag.Bag(name, 'w')
                self.bagfile_open = True
        
        self.e.left_forward = self.sensor_values.left_forward
        self.e.left_side = self.sensor_values.left_side
        self.e.right_side = self.sensor_values.right_side
        self.e.right_forward = self.sensor_values.right_forward
        self.e.linear_x = self.cmd_vel.linear.x
        self.e.angular_z = self.cmd_vel.angular.z

        self.event_pub.publish(self.e)
        self.bag.write('/event', self.e)

if __name__ == '__main__':
    rospy.init_node('Teaching')
    Teaching()
