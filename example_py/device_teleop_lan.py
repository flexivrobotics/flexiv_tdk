#!/usr/bin/env python3

"""
device_teleop_lan.py

Example only: hold one or more robots at the current TCP pose.

DeviceTeleopLAN is the interface for device-to-robot teleoperation, for example VR
controllers or haptic devices driving one or more robots. This script does not
talk to any such device. It only shows the call order: Init(), set a contact-wrench
limit, then stream commands with Start(). A real application must read its own device
driver and, together with the robot state, write a smooth MotionControlCmds sequence.
Do not copy a target pose in from an unrelated frame or with a large jump.

The main thread only writes MotionControlCmds. Command objects stay alive until after Stop().
"""

__copyright__ = "Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import argparse
import sys
import time

# pip install flexivtdk
import flexivtdk

# Soft contact limit used by this example, not a device-specific value. [N]:[Nm].
EXAMPLE_MAX_CONTACT_WRENCH = [25.0, 25.0, 25.0, 10.0, 10.0, 10.0]


def log_info(msg):
    print("[info] {}".format(msg))


def log_warn(msg):
    print("[warn] {}".format(msg), file=sys.stderr)


def log_error(msg):
    print("[error] {}".format(msg), file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(
        description="Hold one or more robots at the current TCP pose. The robots do not move.")
    parser.add_argument("-r", "--robot-sn", action="append", required=True,
                        help="Serial number of a robot to hold. Repeat for each additional robot.")
    parser.add_argument("-A", "--lan-ip", action="append", default=[],
                        help="LAN IPv4 address of the NIC connected to the robots. Repeatable.")
    parser.add_argument("-t", "--seconds", type=float, default=30.0,
                        help="How long to hold the pose. Default: 30.")
    parser.add_argument("-n", "--no-zero-ft", action="store_true",
                        help="Skip force/torque sensor zeroing during Init().")
    args = parser.parse_args()
    if args.seconds <= 0:
        log_error("--seconds must be positive")
        return 1
    if not args.lan_ip:
        log_warn("network_interface_whitelist is not provided, will search all network interfaces.")

    teleop = None
    try:
        teleop = flexivtdk.DeviceTeleopLAN(args.robot_sn, args.lan_ip)
        zero_ft = (flexivtdk.ZeroFTSensor.Disable if args.no_zero_ft
                   else flexivtdk.ZeroFTSensor.Enable)
        teleop.Init(zero_ft)

        # Init() has already switched the control mode, so the contact limit can be set before
        # Start(). A real device should choose this from its own safety requirements.
        for i in range(len(args.robot_sn)):
            teleop.SetRobotMaxContactWrench(i, EXAMPLE_MAX_CONTACT_WRENCH)

        # One command object per robot, in the same order as --robot-sn. Keep this list and the
        # objects it holds until after Stop(); the realtime threads read these same objects.
        zero_twist = [0.0] * 6
        cmds = []
        hold_poses = []
        for i, serial in enumerate(args.robot_sn):
            pose = list(teleop.instance(i).states().tcp_pose)
            cmd = flexivtdk.MotionControlCmds()
            cmd.write(pose, zero_twist, zero_twist)
            cmds.append(cmd)
            hold_poses.append(pose)
            log_info("Holding robot [{}] at its current TCP pose".format(serial))

        teleop.Start(cmds)
        log_info("Control loop started. Updating the same command objects from this thread.")

        deadline = time.monotonic() + args.seconds
        next_status = time.monotonic()
        while time.monotonic() < deadline:
            for cmd, pose in zip(cmds, hold_poses):
                cmd.write(pose, zero_twist, zero_twist)
            if time.monotonic() >= next_status:
                for i, serial in enumerate(args.robot_sn):
                    if teleop.fault(i):
                        log_warn("Robot [{}] is in fault. Other robots keep holding.".format(serial))
                next_status = time.monotonic() + 1.0
            time.sleep(0.01)

        log_info("Stopping teleoperation.")
        teleop.Stop()
        teleop = None
    except KeyboardInterrupt:
        log_info("Program interrupted by user, stopping teleoperation...")
        if teleop is not None:
            try:
                teleop.Stop()
            except Exception as e:
                log_error("Failed to stop teleoperation: {}".format(e))
                return 1
        return 0
    except Exception as e:
        log_error("Error: {}".format(e))
        if teleop is not None:
            try:
                teleop.Stop()
            except Exception as stop_error:
                log_error("Failed to stop teleoperation: {}".format(stop_error))
        return 1

    log_info("Teleoperation exiting.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
