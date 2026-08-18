#!/usr/bin/env python3

"""Periodically announce a multicast subscription using IGMPv2.

Some Wi-Fi access points do not forward the OptiTrack multicast stream after
Linux sends its default IGMPv3 membership report.  Sending an IGMPv2 report
keeps the stream flowing to the Cubie A7Z.  This script requires CAP_NET_RAW;
the Radxa Docker container is privileged and therefore has that capability.
"""

import argparse
import datetime
import socket
import struct
import time


DEFAULT_GROUP_IP = "239.255.42.99"


def get_outgoing_ip(group_ip):
    """Return the local IPv4 address selected by the route to group_ip."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # UDP connect does not transmit anything, but asks the kernel to select
        # the same route/interface that will be used for the multicast group.
        sock.connect((group_ip, 1))
        return sock.getsockname()[0]
    finally:
        sock.close()


def send_igmp_v2_membership_report(group_ip, interface_ip):
    # IGMPv2 Membership Report packet format
    # Type (1 byte), Max Resp Time (1 byte), Checksum (2 bytes), Group Address (4 bytes)

    IGMP_MEMBERSHIP_REPORT_TYPE = 0x16  # IGMPv2 Membership Report
    MAX_RESP_TIME = 0  # Not used in Membership Report
    CHECKSUM_PLACEHOLDER = 0

    # Convert group IP to bytes
    group_bytes = socket.inet_aton(group_ip)

    # Create IGMP packet with a placeholder checksum
    igmp_packet = struct.pack('!BBH4s',
                               IGMP_MEMBERSHIP_REPORT_TYPE,
                               MAX_RESP_TIME,
                               CHECKSUM_PLACEHOLDER,
                               group_bytes)

    # Calculate checksum
    def calculate_checksum(data):
        if len(data) % 2:
            data += b'\x00'
        checksum = sum(struct.unpack('!%dH' % (len(data) // 2), data))
        checksum = (checksum >> 16) + (checksum & 0xFFFF)
        checksum += checksum >> 16
        return ~checksum & 0xFFFF

    checksum = calculate_checksum(igmp_packet)

    # Rebuild the packet with the correct checksum
    igmp_packet = struct.pack('!BBH4s',
                               IGMP_MEMBERSHIP_REPORT_TYPE,
                               MAX_RESP_TIME,
                               checksum,
                               group_bytes)

    # Create a raw socket and force the report onto the selected interface.
    sock = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_IGMP)
    try:
        sock.setsockopt(
            socket.IPPROTO_IP,
            socket.IP_MULTICAST_IF,
            socket.inet_aton(interface_ip),
        )
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
        sock.sendto(igmp_packet, (group_ip, 0))
    finally:
        sock.close()

    dt_now = datetime.datetime.now()
    print(
        "[{}] IGMPv2 Membership Report sent to {} from {}".format(
            dt_now, group_ip, interface_ip
        ),
        flush=True,
    )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--group", default=DEFAULT_GROUP_IP)
    parser.add_argument(
        "--interface-ip",
        help="Local IPv4 address to use; defaults to the route selected by the kernel",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.0,
        help="Seconds between reports; zero sends one report and exits",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.interval < 0:
        raise ValueError("--interval must be non-negative")

    while True:
        try:
            interface_ip = args.interface_ip or get_outgoing_ip(args.group)
            send_igmp_v2_membership_report(args.group, interface_ip)
        except OSError as error:
            print(
                "[{}] Failed to send IGMPv2 Membership Report: {}".format(
                    datetime.datetime.now(), error
                ),
                flush=True,
            )
            if args.interval == 0:
                raise
        if args.interval == 0:
            break
        time.sleep(args.interval)


if __name__ == "__main__":
    main()
