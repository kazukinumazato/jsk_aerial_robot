#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Fly a square while linearly changing the commanded roll and pitch."""

import rospy

from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Odometry


def lerp(start, end, fraction):
    return start + (end - start) * fraction


def make_nav_message(x, y, h, vx, vy, roll, pitch):
    msg = FlightNav()
    msg.header.stamp = rospy.Time.now()
    msg.control_frame = FlightNav.WORLD_FRAME
    msg.target = FlightNav.COG

    msg.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
    msg.target_pos_x = x
    msg.target_pos_y = y
    msg.target_vel_x = vx
    msg.target_vel_y = vy

    msg.pos_z_nav_mode = FlightNav.POS_MODE
    msg.target_pos_z = h
    msg.target_vel_z = 0.0

    msg.roll_nav_mode = FlightNav.POS_MODE
    msg.pitch_nav_mode = FlightNav.POS_MODE
    msg.target_roll = roll
    msg.target_pitch = pitch

    return msg


def main():
    rospy.init_node("square_pose")

    # a: side length [m], b: attitude angle [rad], h: altitude [m]
    a = float(rospy.get_param("~a", 1.0))
    b = float(rospy.get_param("~b", 0.1))
    h = float(rospy.get_param("~h", 1.0))
    flight_time = float(rospy.get_param("~flight_time", 20.0))
    rate_hz = float(rospy.get_param("~rate_hz", 40.0))
    reset_duration = float(rospy.get_param("~reset_duration", 2.0))

    nav_pub = rospy.Publisher("/crobat/uav/nav", FlightNav, queue_size=1)
    rate = rospy.Rate(rate_hz)

    odom = rospy.wait_for_message("/crobat/uav/cog/odom", Odometry)
    start_x = odom.pose.pose.position.x
    start_y = odom.pose.pose.position.y

    # Position offsets: +y, +x, -y, -x.
    corners = (
        (0.0, 0.0),
        (0.0, a),
        (a, a),
        (a, 0.0),
        (0.0, 0.0),
    )

    # Roll/pitch commands at the start and end of each edge.
    attitudes = (
        (0.0, 0.0),
        (-b, 0.0),
        (0.0, b),
        (b, 0.0),
        (0.0, -b),
    )

    edge_duration = flight_time / 4.0
    speed = a / edge_duration
    velocities = (
        (0.0, speed),
        (speed, 0.0),
        (0.0, -speed),
        (-speed, 0.0),
    )

    start_time = rospy.Time.now()

    rospy.loginfo(
        "Square pose: start=(%.3f, %.3f, %.3f), a=%.3f m, "
        "b=%.3f rad, flight_time=%.3f s, speed=%.3f m/s",
        start_x,
        start_y,
        h,
        a,
        b,
        flight_time,
        speed,
    )

    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start_time).to_sec()
        if elapsed >= flight_time:
            break

        edge = min(int(elapsed / edge_duration), 3)
        fraction = (elapsed - edge * edge_duration) / edge_duration

        x = start_x + lerp(corners[edge][0], corners[edge + 1][0], fraction)
        y = start_y + lerp(corners[edge][1], corners[edge + 1][1], fraction)
        roll = lerp(attitudes[edge][0], attitudes[edge + 1][0], fraction)
        pitch = lerp(attitudes[edge][1], attitudes[edge + 1][1], fraction)
        vx, vy = velocities[edge]

        nav_pub.publish(make_nav_message(x, y, h, vx, vy, roll, pitch))
        rate.sleep()

    # Hold the initial position and return from (0, -b) to (0, 0).
    reset_start = rospy.Time.now()
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - reset_start).to_sec()
        fraction = min(elapsed / reset_duration, 1.0) if reset_duration > 0.0 else 1.0
        pitch = lerp(-b, 0.0, fraction)

        nav_pub.publish(
            make_nav_message(start_x, start_y, h, 0.0, 0.0, 0.0, pitch)
        )

        if fraction >= 1.0:
            break
        rate.sleep()

    nav_pub.publish(make_nav_message(start_x, start_y, h, 0.0, 0.0, 0.0, 0.0))
    rospy.loginfo("Square pose completed; returned to the initial position and level attitude.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
