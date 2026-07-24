#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from spinal.msg import ServoControlCmd


def main():
    rospy.init_node("extra_servo_cmd_publisher")

    pub = rospy.Publisher(
        "/extra_servo_cmd",
        ServoControlCmd,
        queue_size=10
    )

    rate = rospy.Rate(10)  # 10 Hz

    # Publisher の接続待ち
    rospy.sleep(1.0)

    for x in range(51):  # 0〜40
        if rospy.is_shutdown():
            break

        msg = ServoControlCmd()
        msg.index = [6]
        msg.angles = [90 - x]
        
        pub.publish(msg)

        rospy.loginfo(
            "Published to /extra_servo_cmd: index=%s, angles=%s",
            msg.index,
            msg.angles
        )

        rate.sleep()
    msg = ServoControlCmd()
    msg.index = [6]
    msg.angles = [90]
    pub.publish(msg)


if __name__ == "__main__":
    main()
