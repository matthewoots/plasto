#!/usr/bin/env python3

import math 
import random
import time
import rospy
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped

# launch this with
# python send_command_auto.py

def normalize_angle(angle):
    """
    Wrap the angle between -pi and pi.
    Args:
        angle (float): angle to wrap.
    Returns:
         The wrapped angle.
    """
    a = normalize_angle_positive(angle)
    if a > np.pi:
        a -= 2. * np.pi

    return a 



def normalize_angle_positive(angle):
    """
    Wrap the angle between 0 and 2 * pi.
    Args:
        angle (float): angle to wrap.
    Returns:
         The wrapped angle.
    """
    pi_2 = 2. * np.pi

    return np.fmod(np.fmod(angle, pi_2) + pi_2, pi_2) 



class auto_publisher:
    def __init__(self):
        self.pose = Point()
        self.target = PoseStamped()
        self.init = False
        self.counter = 0
        self.pub = rospy.Publisher('/plasto_node/goal', PoseStamped, queue_size=20, latch=True)
        self.sub = rospy.Subscriber("/plasto_node/pose", PoseStamped, self.callback, queue_size=20)
        epoch = rospy.Time.now()
        while self.pub.get_num_connections() < 1:
            # Do nothing
            if (rospy.Time.now() - epoch).to_sec() > 5.0:
                print("No connections in 5s")
                return

    def callback(self, data):
        # print(self.pose)
        self.pose = data.pose.position
        bearing = math.atan2(self.pose.y, self.pose.x)
        if not self.init:
            self.counter += 1
            next_bearing = \
                normalize_angle(bearing - math.pi + \
                (random.random()*2 - 1) * (math.pi/10))
            h = math.sqrt( \
                math.pow(self.pose.x, 2) + \
                math.pow(self.pose.y, 2))
            
            self.target.pose.position.x = h * math.cos(next_bearing)
            self.target.pose.position.y = h * math.sin(next_bearing)
            self.target.pose.position.z = self.pose.z
            self.pub.publish(self.target)
            print("[" + str(self.counter) + "] publish new point")
            self.init = True
            return
        
        distance = math.sqrt( \
        math.pow(self.target.pose.position.x - self.pose.x, 2) + \
        math.pow(self.target.pose.position.y - self.pose.y, 2) + \
        math.pow(self.target.pose.position.z - self.pose.z, 2))

        # if (distance < 0.2):
        if (distance < 2.0):
            self.init = False
            # time.sleep(2)
            time.sleep(0.1)
            return
        

if __name__ == '__main__':
    rospy.init_node('auto_command_publisher')
    auto_publisher()
    rospy.spin()