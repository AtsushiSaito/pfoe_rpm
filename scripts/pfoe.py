#!/usr/bin/env python
#coding: UTF-8

import rospy, rosbag, rosparam
import math, random, copy
import numpy as np
from std_srvs.srv import Trigger, TriggerResponse
from raspimouse_ros_2.msg import LightSensorValues, ButtonValues
from geometry_msgs.msg import Twist
from pfoe_rpm.msg import Event
from pfoe_rpm.msg import PFoE_Debug

class Particle:
    def __init__(self, w, t):
        self.weight = w
        self.time = t

class Action:
    def __init__(self, lx, az):
        self.lx = lx
        self.az = az

class Observation:
    def __init__(self, ls, lf, rf, rs):
        self.ls = math.log10(ls) if ls > 0 else math.log10(1)
        self.lf = math.log10(lf) if lf > 0 else math.log10(1)
        self.rf = math.log10(rf) if rf > 0 else math.log10(1)
        self.rs = math.log10(rs) if rs > 0 else math.log10(1)

class Event:
    def __init__(self, action, observation):
        self.act = action
        self.obs = observation
        self.id = 0

class ParticleFilter:
    def __init__(self, N):
        self.N = N
        self.p = []
        for i in range(self.N):
            self.p.append(Particle(0.0, 0))

    def init(self, e_max):
        self.event_max = e_max
        for p in self.p:
            p.time = random.randint(0, self.event_max - 2)
            p.weight = (1.0 / self.N)

    def likelihood(self, s, o):
        ans = 1.0
        d = [(s.ls - o.ls), (s.lf - o.lf), (s.rf - o.rf), (s.rs - o.rs)]
        for i in d:
            ans /= (1+ math.fabs(i))
        
        return ans

    def motionUpdate(self):
        for p in self.p:
            if random.randint(0, 10) == 0:
                p.time = random.randint(0, self.event_max - 2)
            rand = random.randint(0, 3)
            if rand == 0:
                p.time += 1
            elif rand == 1:
                p.time += 2

            if p.time  >= (self.event_max - 1):
                p.time = random.randint(0, self.event_max - 2)

    def resampling(self):
        p_sum = 0.0
        new_p = []
        AccumPD = []
        for p in self.p:
            AccumPD.append(p_sum + p.weight)
            p_sum += p.weight
        point = random.uniform(0.0, 1.0 / self.N)
        while point < 1.0:
            if AccumPD[0] >= point:
                new_p.append(Particle(1.0 / self.N, copy.deepcopy(self.p[0].time)))
                point += 1.0 / self.N
            else:
                AccumPD.pop(0)
                self.p.pop(0)
        self.p = new_p

    def normalize(self):
        self.eta = 0.0
        for p in self.p:
            self.eta += p.weight
        for p in self.p:
            p.weight /= self.eta

    def sensorUpdates(self, sensor_values, event): 
        for p in self.p:
            p.weight *= self.likelihood(sensor_values, event[p.time].obs)
        self.normalize()
        self.resampling()
        return self.modeEvent()

    def modeEvent(self):
        p_list = [0 for i in range(self.event_max)]
        for p in self.p:
            p_list[p.time] += 1
        return p_list.index(max(p_list))

    def getInfo(self):
        return [self.eta, self.p]

class Episode:
    def __init__(self):
        self.event = []
        self.id_counter = 0

    def addEvent(self, e):
        e.id = self.id_counter
        self.event.append(e)
        self.id_counter += 1

class Replay:
    def __init__(self):
        self.pf = ParticleFilter(1000)
        self.episode = Episode()
        self.twist = Twist()
        self.pfoe_debug = PFoE_Debug()
        self.bag_open = False
        self.on = False

        rospy.ServiceProxy('/motor_on', Trigger).call()
        rospy.Subscriber('/buttons', ButtonValues, self.button_callback, queue_size = 1)
        rospy.Subscriber('/lightsensors', LightSensorValues, self.sensor_callback, queue_size = 1)

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.pfoe = rospy.Publisher('/pfoe', PFoE_Debug, queue_size = 1)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.on:
                print ("Ready")
                self.bag_open = False
                rate.sleep()
                continue
            elif not self.bag_open:
                self.readEpisode()
                self.bag_open = True
                self.pf.init(len(self.episode.event))
                rate.sleep()
                continue
            target_e = self.pf.sensorUpdates(self.sensor_values, self.episode.event)
            info = self.pf.getInfo()
            self.pfoe_debug.recall_event = target_e
            self.pfoe_debug.eta = info[0]
            #self.pfoe_debug.particle_time = [p for p in info[1]]
            self.twist.linear.x = self.episode.event[target_e].act.lx
            self.twist.angular.z = self.episode.event[target_e].act.az
            
            self.pfoe.publish(self.pfoe_debug)
            self.cmd_vel.publish(self.twist)
            self.pf.motionUpdate()
            print ("--------")
            rate.sleep()

    def button_callback(self, msg):
        self.on = msg.front_toggle

    def sensor_callback(self, msg):
        self.sensor_values = Observation(msg.left_side, msg.left_forward, msg.right_forward, msg.right_side)
    
    def readEpisode(self):
        self.bagfile = rospy.get_param("/bagfile")
        self.bag = rosbag.Bag(self.bagfile)
        for topic, msg, t in self.bag.read_messages(topics=['/event']):
            a = Action(msg.linear_x, msg.angular_z)
            o = Observation(msg.left_side, msg.left_forward, msg.right_forward, msg.right_side)  
            self.episode.addEvent(Event(a, o))

if __name__ == '__main__':
    rospy.init_node('Replay')
    Replay()
