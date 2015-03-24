#! /usr/bin/env python
__author__ = 'mandeep'
import os
import rospy

if __name__ == '__main__':
    rospy.init_node("exec_zenodial")
    #rospy.set_param("zeno_dial_dir","/home/catkin_ws/src/zeno_dial/")
    path_z=rospy.get_param("zeno_dial_dir")
    os.system("cd "+path_z+"; ant ZenoDial")