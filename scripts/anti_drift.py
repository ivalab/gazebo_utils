#!/usr/bin/env python
import rospy
from gazebo_utils.gazebo_world_to_map import MapCreator

if __name__ == '__main__':
    rospy.init_node("gazebo_anti_drift")
    update_freq = rospy.get_param('~update_freq', 1.0)


    map = MapCreator()
    map.runRobotAntiDrift()

    rospy.spin()

