#!/usr/bin/env python

"""
transparent_cartesian_teleop_wan_pro.py

Example usage of Transparent Cartesian teleoperation over WAN (TDK Professional Edition,
TDK Server credential). Controls a follower robot from a leader robot with transparent
force feedback. Supports keyboard and digital input engage/disengage, message latency
query, teleop status query, and optional GripperRemoteControl when a follower gripper
device name is provided.

This program is provided only as an example. Users must adapt it to their own application
requirements, safety procedures, and software architecture before deployment.

"""

from __future__ import annotations

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


def _clamp(value, lo, hi):
    return max(lo, min(value, hi))


# Immutable example parameters
kPreferredJntPos = _deg2rad_list([60, -60, -85, 115, 70, 0, 0])
kHomeJntPos = _deg2rad_list([0, -40, 0, 90, 0, 40, 0])
kDefaultMaxContactWrench = [25.0, 25.0, 25.0, 10.0, 10.0, 10.0]
kIdx = 0
kJointGroup = flexivrdk.JointGroup.ARM_1

# Mutable process state
g_stop_event = threading.Event()
g_gripper_name = ""


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


def print_gripper_states(gripper: flexivtdk.GripperRemoteControl):
    try:
        states = gripper.states(kIdx, kJointGroup)
        log_info(
            "Gripper states: width = {:.4f} m, force = {:.2f} N, is_moving = {}".format(
                states.width, states.force, states.is_moving))
    except Exception as e:
        log_warn(f"Gripper states not available yet: {e}")


class WanTeleoperationController:
    """Encapsulates WAN teleoperation functionality for better organization and maintainability."""

    def __init__(self, teleop: flexivtdk.TransparentCartesianTeleopWAN,
                 gripper: Optional[flexivtdk.GripperRemoteControl] = None):
        self.teleop = teleop
        self.gripper = gripper
        self.index = kIdx
        self._command_map = self._create_command_map()
        self._menu = self._create_menu()

    def _create_command_map(self) -> Dict[str, Callable]:
        command_map = {
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
        if self.gripper is not None:
            command_map.update({
                'G': self._gripper_enable,
                'D': self._gripper_disable,
                'N': self._gripper_init,
                'o': self._gripper_open,
                'c': self._gripper_close,
                's': self._gripper_stop,
                't': lambda: print_gripper_states(self.gripper),
                'P': self._print_gripper_params,
            })
        return command_map

    def _create_menu(self) -> str:
        menu = """
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
"""
        if self.gripper is not None:
            menu += """
  --- Gripper Lifecycle (Leader only, via RPC) ---
    G        : Enable gripper on follower robot
    D        : Disable gripper on follower robot
    N        : Trigger initialization of the enabled gripper

  --- Gripper Motion (Leader only, via reliable topic) ---
    o        : Open gripper (Move to max width)
    c        : Close gripper (Grasp with moderate force)
    s        : Stop and hold gripper

  --- Gripper States ---
    t        : Print latest gripper states
    P        : Print gripper params (valid command ranges)
"""
        menu += """
  --- Help ---
    Any other key to show this help menu
    """
        return menu

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

    def _require_leader(self, command: str) -> bool:
        if self.teleop.role() != flexivtdk.Role.WAN_TELEOP_LEADER:
            log_warn(f"Command '{command}' is only available on the leader")
            return False
        return True

    def _gripper_enable(self):
        if not self._require_leader('G'):
            return
        self.gripper.Enable(self.index, kJointGroup, g_gripper_name)
        log_info(f"Follower gripper [{g_gripper_name}] enabled")

    def _gripper_disable(self):
        if not self._require_leader('D'):
            return
        self.gripper.Disable(self.index, kJointGroup)
        log_info("Follower gripper disabled")

    def _gripper_init(self):
        if not self._require_leader('N'):
            return
        self.gripper.Init(self.index, kJointGroup)
        log_info("Follower gripper initialization triggered")

    def _gripper_open(self):
        if not self._require_leader('o'):
            return
        params = self.gripper.params(self.index, kJointGroup)
        velocity = _clamp(0.5 * params.max_vel, params.min_vel, params.max_vel)
        force_limit = _clamp(0.5 * params.max_force, params.min_force, params.max_force)
        self.gripper.Move(self.index, kJointGroup, params.max_width, velocity, force_limit)
        log_info(
            "Open command sent: width = {:.4f} m, velocity = {:.4f} m/s, force_limit = {:.2f} N"
            .format(params.max_width, velocity, force_limit))

    def _gripper_close(self):
        if not self._require_leader('c'):
            return
        params = self.gripper.params(self.index, kJointGroup)
        force = _clamp(0.5 * params.max_force, params.min_force, params.max_force)
        self.gripper.Grasp(self.index, kJointGroup, force)
        log_info("Grasp command sent: force = {:.2f} N".format(force))

    def _gripper_stop(self):
        if not self._require_leader('s'):
            return
        self.gripper.Stop(self.index, kJointGroup)
        log_info("Stop command sent")

    def _print_gripper_params(self):
        try:
            params = self.gripper.params(self.index, kJointGroup)
            log_info(
                "Gripper params: width = [{:.4f}, {:.4f}] m, velocity = [{:.4f}, "
                "{:.4f}] m/s, force = [{:.2f}, {:.2f}] N".format(
                    params.min_width, params.max_width, params.min_vel, params.max_vel,
                    params.min_force, params.max_force))
        except Exception as e:
            log_warn(f"Gripper params not available yet: {e}")

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


def read_digital_input_task(teleop: flexivtdk.TransparentCartesianTeleopWAN,
                            gripper: Optional[flexivtdk.GripperRemoteControl] = None):
    di1_stable = False
    di1_counter = 0
    gripper_opened = False
    while not g_stop_event.is_set():
        try:
            di_state = teleop.digital_inputs(kIdx)
            if di_state and len(di_state) > 0:
                teleop.Engage(kIdx, kJointGroup, bool(di_state[0]))

            if gripper is not None and di_state and len(di_state) > 1:
                di1_raw = bool(di_state[1])
                if di1_raw != di1_stable:
                    di1_counter += 1
                    if di1_counter >= 3:
                        di1_stable = di1_raw
                        di1_counter = 0
                        if di1_stable:
                            try:
                                params = gripper.params(kIdx, kJointGroup)
                                if not gripper_opened:
                                    width = _clamp(0.5 * params.max_width, params.min_width,
                                                   params.max_width)
                                    velocity = _clamp(0.5 * params.max_vel, params.min_vel,
                                                      params.max_vel)
                                    force_limit = _clamp(0.5 * params.max_force,
                                                         params.min_force, params.max_force)
                                    gripper.Move(kIdx, kJointGroup, width, velocity, force_limit)
                                    gripper_opened = True
                                    log_info(
                                        f"DI1 pressed: opening gripper to width = {width:.4f} m")
                                else:
                                    force = _clamp(40.0, params.min_force, params.max_force)
                                    gripper.Grasp(kIdx, kJointGroup, force)
                                    gripper_opened = False
                                    log_info(
                                        f"DI1 pressed: closing gripper with force = {force:.2f} N")
                            except Exception as e:
                                log_warn(f"DI1 gripper action failed: {e}")
                else:
                    di1_counter = 0
        except Exception as e:
            log_error(f"Exception in ReadDigitalInputTask: {e}")
        time.sleep(0.01)
    log_info("ReadDigitalInputTask exiting.")


def console_task(teleop: flexivtdk.TransparentCartesianTeleopWAN,
                 gripper: Optional[flexivtdk.GripperRemoteControl] = None):
    controller = WanTeleoperationController(teleop, gripper)
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
        description="Transparent Cartesian Teleop WAN example (Professional Edition)")
    parser.add_argument("-l", "--leader_sn", required=True, help="serial number of leader robot")
    parser.add_argument("-f", "--follower_sn", required=True, help="serial number of follower robot")
    parser.add_argument("-r", "--role", required=True, choices=["leader", "follower"], help="role in teleop")
    parser.add_argument("-c", "--client-config", required=True,
        help="Path to client.conf from the TDK Server credential package")
    parser.add_argument("-n", "--gripper-name", default="",
        help="Follower gripper device name. Enables GripperRemoteControl")
    parser.add_argument("-D", "--enable-digital-input", action="store_true",
        help="enable digital input reading task")
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None):
    global g_gripper_name
    args = parse_args(argv)
    g_gripper_name = args.gripper_name

    if args.role == 'follower':
        role = flexivtdk.Role.WAN_TELEOP_FOLLOWER
    else:
        role = flexivtdk.Role.WAN_TELEOP_LEADER

    network_cfg = flexivtdk.NetworkCfgPro()
    network_cfg.client_config_file = args.client_config
    robot_pairs = [(args.leader_sn, args.follower_sn)]

    teleop = None
    gripper = None
    console_thr = None
    pedal_thread = None

    try:
        teleop = flexivtdk.TransparentCartesianTeleopWAN(robot_pairs, role, network_cfg)

        pair_sn = teleop.robot_pair_sn(kIdx)
        log_info(f"role={teleop.role()} robot_pair_sn=({pair_sn[0]}, {pair_sn[1]})")

        if g_gripper_name:
            gripper = flexivtdk.GripperRemoteControl(teleop, network_cfg)
            log_info(f"Gripper remote control is ENABLED, target gripper device: [{g_gripper_name}]")
        else:
            log_info("Gripper remote control is DISABLED (no --gripper-name provided).")

        teleop.Init()
        teleop.Start()
        teleop.SetMaxContactWrench(kIdx, kJointGroup, kDefaultMaxContactWrench)

        if role == flexivtdk.Role.WAN_TELEOP_FOLLOWER and gripper is not None:
            try:
                gripper.EnableLocal(kIdx, kJointGroup, g_gripper_name)
            except Exception as e:
                log_error(
                    f"Failed to auto-enable gripper [{g_gripper_name}] on follower: {e}. "
                    "The leader can still retry via the 'G' console command.")

        log_info("WAN Teleop started.")

        console_thr = threading.Thread(
            target=console_task, args=(teleop, gripper), daemon=True)
        console_thr.start()
        log_info("Console task started.")

        if args.role == 'leader' and args.enable_digital_input:
            di1_note = ("toggles follower gripper open/close"
                        if gripper is not None else "unused (gripper feature disabled)")
            log_info(
                "Starting ReadDigitalInputTask thread as role is 'leader' and requested by "
                f"--enable-digital-input flag. DI0: arm teleop engagement; DI1: {di1_note}.")
            pedal_thread = threading.Thread(
                target=read_digital_input_task, args=(teleop, gripper), daemon=True)
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
