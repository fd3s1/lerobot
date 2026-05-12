#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import csv
import time
import rospkg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import AttitudeTarget



if __name__ == '__main__':

    try:
        rospy.init_node("network_io")

        
        rospy.loginfo("Waiting for trigger.")

        rospy.spin()

    except rospy.ROSInterruptException:
        pass