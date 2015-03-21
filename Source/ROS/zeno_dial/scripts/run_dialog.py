#! /usr/bin/env python
__author__ = 'mandeep'
import os
import rospy

if __name__ == '__main__':
    rospy.init_node("exec_zenodial")
    path_z=rospy.get_param("/zeno_dial_dir","/home/catkin_ws/zeno_dial/")
    os.system("cd "+path_z+"; ant ZenoDial")