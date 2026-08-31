#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Fly a square trajectory while keeping a commanded roll angle."""

import math

import rospy
from aerial_robot_msgs.msg import FlightNav


class SquareRollTrajectory:
    def __init__(self):
        self.side_length = float(rospy.get_param("~side_length", 1.0))
        self.lap_time = float(rospy.get_param("~lap_time", 20.0))
        self.loops = int(rospy.get_param("~loops", 1))
        self.center_x = float(rospy.get_param("~center_x", 0.0))
        self.center_y = float(rospy.get_param("~center_y", 0.0))
        self.roll = math.radians(float(rospy.get_param("~roll_deg", 45.0)))
        self.ramp_time = float(rospy.get_param("~ramp_time", 3.0))
        self.publish_hz = float(rospy.get_param("~publish_hz", 40.0))
        self.nav_topic = rospy.get_param("~nav_topic", "/crobat/uav/nav")

        if self.side_length <= 0.0:
            raise ValueError("~side_length must be > 0")
        if self.lap_time <= 0.0:
            raise ValueError("~lap_time must be > 0")
        if self.loops < 1:
            raise ValueError("~loops must be >= 1")
        if self.ramp_time < 0.0:
            raise ValueError("~ramp_time must be >= 0")
        if self.publish_hz <= 0.0:
            raise ValueError("~publish_hz must be > 0")

        self.edge_time = self.lap_time / 4.0
        self.speed = self.side_length / self.edge_time
        half = self.side_length / 2.0
        # Counter-clockwise, starting at the lower-left corner.
        self.corners = (
            (self.center_x - half, self.center_y - half),
            (self.center_x + half, self.center_y - half),
            (self.center_x + half, self.center_y + half),
            (self.center_x - half, self.center_y + half),
        )

        self.publisher = rospy.Publisher(self.nav_topic, FlightNav, queue_size=1)
        self.rate = rospy.Rate(self.publish_hz)

    def make_message(self, x, y, vx, vy, roll):
        msg = FlightNav()
        msg.header.stamp = rospy.Time.now()
        msg.control_frame = FlightNav.WORLD_FRAME
        msg.target = FlightNav.COG
        msg.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
        msg.target_pos_x = x
        msg.target_pos_y = y
        msg.target_vel_x = vx
        msg.target_vel_y = vy
        msg.roll_nav_mode = FlightNav.POS_MODE
        msg.target_roll = roll
        return msg

    def square_target(self, elapsed):
        phase = elapsed % self.lap_time
        edge = min(int(phase / self.edge_time), 3)
        fraction = (phase - edge * self.edge_time) / self.edge_time
        start = self.corners[edge]
        end = self.corners[(edge + 1) % 4]
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        x = start[0] + fraction * dx
        y = start[1] + fraction * dy
        return x, y, dx / self.edge_time, dy / self.edge_time

    def hold_and_ramp_roll(self, start_roll, end_roll):
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            fraction = min(elapsed / self.ramp_time, 1.0) if self.ramp_time > 0.0 else 1.0
            roll = start_roll + fraction * (end_roll - start_roll)
            x, y = self.corners[0]
            self.publisher.publish(self.make_message(x, y, 0.0, 0.0, roll))
            if fraction >= 1.0:
                return
            self.rate.sleep()

    def run(self):
        rospy.loginfo(
            "Square roll trajectory: side=%.3f m, lap=%.3f s, loops=%d, roll=%.1f deg",
            self.side_length,
            self.lap_time,
            self.loops,
            math.degrees(self.roll),
        )

        self.hold_and_ramp_roll(0.0, self.roll)
        if rospy.is_shutdown():
            return

        start_time = rospy.Time.now()
        duration = self.loops * self.lap_time
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= duration:
                break
            x, y, vx, vy = self.square_target(elapsed)
            self.publisher.publish(self.make_message(x, y, vx, vy, self.roll))
            self.rate.sleep()

        if rospy.is_shutdown():
            return

        # Stop at the initial corner, then return to level attitude.
        x, y = self.corners[0]
        self.publisher.publish(self.make_message(x, y, 0.0, 0.0, self.roll))
        self.hold_and_ramp_roll(self.roll, 0.0)
        rospy.loginfo("Square roll trajectory completed; roll returned to zero.")


if __name__ == "__main__":
    rospy.init_node("square_roll_trajectory")
    try:
        SquareRollTrajectory().run()
    except (rospy.ROSInterruptException, ValueError) as error:
        rospy.logerr("square_roll_trajectory: %s", error)
