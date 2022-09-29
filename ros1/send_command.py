#!/usr/bin/env python3

import sys
import rospy
from geometry_msgs.msg import Point

# launch this with
# python send_command.py x y z

def publisher():
    pub = rospy.Publisher('/goal', Point, queue_size=20)

    rospy.init_node('command_publisher', anonymous=True)

    epoch = rospy.Time.now()
    while pub.get_num_connections() < 1:
        # Do nothing
        if (rospy.Time.now() - epoch).to_sec() > 5.0:
            print("No connections in 5s")
            return

    p = Point()

    # in NWU frame
    p.x = float(sys.argv[1])
    p.y = float(sys.argv[2])
    p.z = float(sys.argv[3])
    
    pub.publish(p)


if __name__ == '__main__':
    publisher()