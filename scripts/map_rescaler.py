#!/usr/bin/env python
import rospy
from gazebo_utils.gazebo_world_to_map import MapCreator

if __name__ == '__main__':
    rospy.init_node("gazebo_map_rescaler")
    world_file = rospy.get_param('~world_file', None)
    new_file = rospy.get_param('~new_file', None)
    scale = rospy.get_param('~scale', 1.0)

    if world_file is None or new_file is None:
        rospy.logerr("gazebo_map_rescaler needs both current and new world file names!")
    else:
        map = MapCreator()
        map.rescaleXMLWalls(world_file=world_file, new_world_file=new_file, scale=scale)

