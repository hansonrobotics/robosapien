#!/usr/bin/env python
__author__ = 'mandeep'

import os
import rospy

if __name__ == '__main__':
    rospy.init_node("chrome")
    #chrome='google-chrome --app="https://localhost:5000"'
    chrome='google-chrome --app="https://localhost:5000"'
    os.system(chrome)

