#!/usr/bin/env python
# license removed for brevity
#from bola import Bola
from robo import Robo
from bola import Bola
import math
import numpy
import rospy
from geometry_msgs.msg import Pose

if __name__ == '__main__':
        rospy.init_node('goleiro',anonymous=True)
        try:
            Bola()
            Robo()
        except rospy.ROSInterruptException: 
            pass
