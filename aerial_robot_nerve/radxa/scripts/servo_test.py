#!/usr/bin/env python3
"""Sweep Crobat's four conventional-PWM servos for a bench test."""

import time

import rospy
from spinal.msg import ServoControlCmd


SERVO_INDICES = [4, 5, 6, 7]


def publish_command(publisher, indices, angles):
    command = ServoControlCmd()
    command.index = indices
    command.angles = angles
    publisher.publish(command)


def center_all_servos(publisher, angle):
    """Reliably leave all servos at the requested center angle."""
    for _ in range(3):
        publish_command(publisher, SERVO_INDICES, [angle] * len(SERVO_INDICES))
        time.sleep(0.05)


def main():
    # Keep Python's normal SIGINT behavior so the finally block runs on Ctrl-C.
    rospy.init_node("radxa_servo_test", disable_signals=True)

    topic = rospy.get_param("~topic", "/crobat/extra_servo_cmd")
    start_angle = int(rospy.get_param("~start_angle", 60))
    end_angle = int(rospy.get_param("~end_angle", 120))
    duration = float(rospy.get_param("~duration", 3.0))
    rate_hz = float(rospy.get_param("~rate", 20.0))
    center_angle = int(rospy.get_param("~center_angle", 90))
    connection_timeout = float(rospy.get_param("~connection_timeout", 5.0))

    if not 0 <= start_angle <= 180 or not 0 <= end_angle <= 180:
        raise ValueError("start_angle and end_angle must be in [0, 180]")
    if not 0 <= center_angle <= 180:
        raise ValueError("center_angle must be in [0, 180]")
    if duration <= 0.0 or rate_hz <= 0.0:
        raise ValueError("duration and rate must be positive")

    publisher = rospy.Publisher(topic, ServoControlCmd, queue_size=10)
    deadline = time.monotonic() + connection_timeout
    while publisher.get_num_connections() == 0 and time.monotonic() < deadline:
        time.sleep(0.05)
    if publisher.get_num_connections() == 0:
        raise RuntimeError("no subscriber connected to " + topic)

    steps = max(1, int(round(duration * rate_hz)))
    try:
        for servo_number, index in enumerate(SERVO_INDICES, start=1):
            rospy.loginfo(
                "servo %d (index %d): set %d deg, then sweep to %d deg in %.3f s",
                servo_number, index, start_angle, end_angle, duration)
            # The initial move is intentionally a step, as required by this test.
            publish_command(publisher, [index], [start_angle])
            start_time = time.monotonic()
            for step in range(1, steps + 1):
                target_time = start_time + duration * step / steps
                remaining = target_time - time.monotonic()
                if remaining > 0.0:
                    time.sleep(remaining)
                fraction = float(step) / steps
                angle = int(round(start_angle + (end_angle - start_angle) * fraction))
                publish_command(publisher, [index], [angle])
    finally:
        rospy.loginfo("returning all servos to %d deg", center_angle)
        center_all_servos(publisher, center_angle)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
