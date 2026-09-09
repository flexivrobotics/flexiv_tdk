#!/usr/bin/env python

"""
transparent_cartesian_teleop_wan.py

Example usage of Transparent Cartesian teleoperation over WAN (TDK Standard Edition,
peer-to-peer TCP). Controls a follower robot from a leader robot with transparent
force feedback. Supports keyboard and digital input engage/disengage, message latency
query, and teleop status query.

This program is provided only as an example. Users must adapt it to their own application
requirements, safety procedures, and software architecture before deployment.

"""

__copyright__ = "Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import argparse
import threading
import time
import sys
import math
from typing import List, Optional, Dict, Callable

# pip install flexivtdk (installs the matching flexivrdk dependency)
import flexivrdk
import flexivtdk


def _deg2rad_list(deg_list):
    return [math.radians(d) for d in deg_list]


# Immutable example parameters
kPreferredJntPos = _deg2rad_list([60, -60, -85, 115, 70, 0, 0])
kHomeJntPos = _deg2rad_list([0, -40, 0, 90, 0, 40, 0])
kDefaultMaxContactWrench = [25.0, 25.0, 25.0, 10.0, 10.0, 10.0]
kIdx = 0
kJointGroup = flexivrdk.JointGroup.ARM_1

# Mutable process state
g_stop_event = threading.Event()


def log_info(msg):
    print(f"[info] {msg}")


def log_warn(msg):
    print(f"[warn] {msg}", file=sys.stderr)


def log_error(msg):
    print(f"[error] {msg}", file=sys.stderr)


def print_teleop_status(status):
    """Print a TeleopStatus snapshot from GetTeleopStatus()."""
    log_info(
        "Teleop status: initialized={} started={} engaged={} stopped={} fault={} "
        "motion_restricted={} latency={:.1f}/{:.1f} ms".format(
            status.initialized, status.started, status.engaged, status.stopped,
            status.fault, status.motion_restricted, status.latency_ms,
            status.latency_threshold_ms))
    if status.primary.code == flexivtdk.TeleopIssueCode.NONE:
        log_info("No teleop restriction. Safe to continue.")
        return
    issue = status.primary
    level = flexivtdk.TeleopIssueLevelStr[int(issue.level)]
    log_warn(f"[{level}] {issue.title} | {issue.description} | {issue.suggestion}")
    for extra in status.issues:
        if (extra.code == issue.code and extra.side == issue.side
                and extra.joint_index == issue.joint_index):
            continue
        code = flexivtdk.TeleopIssueCodeStr[int(extra.code)]
        log_warn(f"  also: [{code}] {extra.title}")


class WanTeleoperationController:
    """Encapsulates WAN teleoperation functionality for better organization and maintainability."""

    def __init__(self, teleop: flexivtdk.TransparentCartesianTeleopWAN):
        self.teleop = teleop
        self.index = kIdx
        self._command_map = self._create_command_map()
        self._menu = self._create_menu()

    def _create_command_map(self) -> Dict[str, Callable]:
        return {
            'r': lambda: self._safe_engage(True),
            'R': lambda: self._safe_engage(False),
            'i': lambda: self._safe_set_nullspace(kPreferredJntPos),
            'I': lambda: self._safe_set_nullspace(kHomeJntPos),
            'p': lambda: self._safe_set_max_contact_wrench(kDefaultMaxContactWrench),
            'u': self._start_teleop,
            'U': self._stop_teleop,
            'l': self._print_latency,
            'h': self._print_teleop_status,
        }

    def _create_menu(self) -> str:
        return """
  --- Teleop Engagement ---
    r        : Engage teleop
    R        : Disengage teleop

  --- Null Space Posture ---
    i/I      : Set local nullspace to Preferred/Home posture

  --- Max Contact Wrench ---
    p        : Set default max contact wrench

  --- Reinit and start ---
    u        : Recall Init and Start
    U        : Stop teleop

  --- TCP message latency ---
    l        : Print current message latency in milliseconds

  --- Teleop status ---
    h        : Print why teleop is restricted / paused and what to do next

  --- Help ---
    Any other key to show this help menu
    """

    def _start_teleop(self):
        try:
            self.teleop.Init()
            self.teleop.Start()
            log_info("Teleop started")
        except Exception as e:
            log_error(f"Failed to start teleop: {e}")

    def _stop_teleop(self):
        try:
            self.teleop.Stop()
            log_info("Teleop stopped")
        except Exception as e:
            log_error(f"Failed to stop teleop: {e}")
            g_stop_event.set()

    def _print_latency(self):
        try:
            ok, latency_ms = self.teleop.CheckTeleopConnectionLatency(self.index)
            if ok:
                log_info(f"Current message latency is: {latency_ms}ms")
            else:
                log_warn("WAN teleop is disconnected.")
        except Exception as e:
            log_error(f"Error checking TCP latency: {e}")

    def _print_teleop_status(self):
        try:
            print_teleop_status(self.teleop.GetTeleopStatus(self.index, kJointGroup))
        except Exception as e:
            log_error(f"Failed to get teleop status: {e}")

    def _safe_engage(self, engage: bool):
        try:
            self.teleop.Engage(self.index, kJointGroup, engage)
            log_info(f"Teleop {'engaged' if engage else 'disengaged'}")
        except Exception as e:
            log_error(f"Failed to {'engage' if engage else 'disengage'} teleop: {e}")

    def _safe_set_nullspace(self, posture: List[float]):
        try:
            self.teleop.SetNullSpacePosture(self.index, kJointGroup, posture)
            log_info("Nullspace posture set")
        except Exception as e:
            log_error(f"Failed to set nullspace posture: {e}")

    def _safe_set_max_contact_wrench(self, wrench: List[float]):
        try:
            self.teleop.SetMaxContactWrench(self.index, kJointGroup, wrench)
            log_info("Max contact wrench set")
        except Exception as e:
            log_error(f"Failed to set max contact wrench: {e}")

    def handle_command(self, user_input: str) -> bool:
        if not user_input:
            print(self._menu)
            return True

        ch = user_input[0]
        if ch in self._command_map:
            try:
                self._command_map[ch]()
                return True
            except Exception as e:
                log_error(f"Exception executing command '{ch}': {e}")
                return False
        else:
            print(self._menu)
            return True


def read_digital_input_task(teleop: flexivtdk.TransparentCartesianTeleopWAN):
    while not g_stop_event.is_set():
        try:
            di_state = teleop.digital_inputs(kIdx)
            if di_state and len(di_state) > 0:
                teleop.Engage(kIdx, kJointGroup, bool(di_state[0]))
        except Exception as e:
            log_error(f"Exception in ReadDigitalInputTask: {e}")
        time.sleep(0.01)
    log_info("ReadDigitalInputTask exiting.")


def console_task(teleop: flexivtdk.TransparentCartesianTeleopWAN):
    controller = WanTeleoperationController(teleop)
    print(controller._menu)

    while not g_stop_event.is_set():
        try:
            user_input = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            log_info("Console exiting by user interrupt.")
            g_stop_event.set()
            break

        if not controller.handle_command(user_input):
            g_stop_event.set()
            break

    log_info("Console thread exiting.")


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Transparent Cartesian Teleop WAN example (Standard Edition)")
    parser.add_argument("-l", "--leader_sn", required=True, help="serial number of leader robot")
    parser.add_argument("-f", "--follower_sn", required=True, help="serial number of follower robot")
    parser.add_argument("-r", "--role", required=True, choices=["leader", "follower"], help="role in teleop")
    parser.add_argument("-t", "--tcp-role", required=True, choices=["server", "client"], help="tcp role")
    parser.add_argument("-i", "--public-ip", required=True, help="public IPv4 address of TCP server")
    parser.add_argument("-p", "--port", required=True, type=int, help="listening port of TCP server")
    parser.add_argument("-W", "--wan-interface", action="append",
        help="OS network-interface name allowed for WAN traffic (for example, wlo1 or enp3s0)",
        default=[])
    parser.add_argument("-D", "--enable-digital-input", action="store_true",
        help="enable digital input reading task")
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None):
    args = parse_args(argv)

    if args.role == 'follower':
        role = flexivtdk.Role.WAN_TELEOP_FOLLOWER
    else:
        role = flexivtdk.Role.WAN_TELEOP_LEADER

    network_cfg = flexivtdk.NetworkCfgStd()
    network_cfg.is_tcp_server = (args.tcp_role == 'server')
    network_cfg.public_ipv4_address = args.public_ip
    network_cfg.listening_port = args.port
    if args.wan_interface:
        network_cfg.wan_interface_whitelist = args.wan_interface

    robot_pairs = [(args.leader_sn, args.follower_sn)]

    teleop = None
    console_thr = None
    pedal_thread = None

    try:
        teleop = flexivtdk.TransparentCartesianTeleopWAN(robot_pairs, role, network_cfg)

        pair_sn = teleop.robot_pair_sn(kIdx)
        log_info(f"role={teleop.role()} robot_pair_sn=({pair_sn[0]}, {pair_sn[1]})")

        teleop.Init()
        teleop.Start()
        teleop.SetMaxContactWrench(kIdx, kJointGroup, kDefaultMaxContactWrench)
        log_info("WAN Teleop started.")

        console_thr = threading.Thread(target=console_task, args=(teleop,), daemon=True)
        console_thr.start()
        log_info("Console task started.")

        if args.role == 'leader' and args.enable_digital_input:
            log_info("Starting ReadDigitalInputTask thread as role is 'leader' and requested by "
                     "--enable-digital-input flag.")
            pedal_thread = threading.Thread(
                target=read_digital_input_task, args=(teleop,), daemon=True)
            pedal_thread.start()
        else:
            log_info("ReadDigitalInputTask thread NOT started (role is not 'leader' or "
                     "--enable-digital-input flag not provided).")

        console_thr.join()
        g_stop_event.set()

        if pedal_thread:
            pedal_thread.join(timeout=1.0)

        if teleop:
            teleop.Stop()
        log_info("WAN Teleop stopped.")

    except KeyboardInterrupt:
        log_info("Program interrupted by user")
        g_stop_event.set()
    except Exception as e:
        log_error(f"Exception in main: {e}")
        g_stop_event.set()
        sys.exit(1)
    finally:
        if console_thr and console_thr.is_alive():
            console_thr.join(timeout=1.0)
        if pedal_thread and pedal_thread.is_alive():
            pedal_thread.join(timeout=1.0)
        if teleop:
            try:
                teleop.Stop()
            except Exception:
                pass


if __name__ == "__main__":
    main()
