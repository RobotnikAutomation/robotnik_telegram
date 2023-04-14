#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from robotnik_telegram.msg_manager import *

def main():

    rospy.init_node("msg_manager_node")

    rc_node = MSGManager()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
