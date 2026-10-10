#!/usr/bin/env python

"""
transparent_cartesian_teleop_wan_pro.py

Example usage of Transparent Cartesian teleoperation over WAN (TDK Professional Edition,
TDK Server credential). Controls a follower robot from a leader robot with transparent force
feedback. Supports keyboard and digital input engage/disengage, message latency query, nullspace
posture tuning, max contact wrench setting, and teleop status query. Optionally enables
GripperRemoteControl when a follower gripper device name is provided.

Note: This program is provided only as an example. Users must adapt it to their own application
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

# pip install flexivtdk
import flexivtdk


# Helper to convert degree lists to radians
def _deg2rad_list(deg_list):
    return [math.radians(d) for d in deg_list]

# Example of null-space postures
kPreferredJntPos = _deg2rad_list([60, -60, -85, 115, 70, 0, 0])
kHomeJntPos = _deg2rad_list([0, -40, 0, 90, 0, 40, 0])

# Example of max contact wrench setting
kDefaultMaxContactWrench = [25.0, 25.0, 25.0, 10.0, 10.0, 10.0]

# Robot pair index used by this example
kIdx = 0

# Gripper commands that require [-n gripper_name]
kGripperCommands = "GDNocstP"
# Gripper commands that are only available on the leader
kGripperLeaderCommands = "GDNocs"
# Gripper motion commands that are disabled when DI1 controls the gripper
kGripperMotionCommands = "ocs"

# Global thread-safe stop event
_stop_event = threading.Event()


def log_info(msg):
    print("[info] {}".format(msg))


def log_warn(msg):
    print("[warn] {}".format(msg), file=sys.stderr)


def log_error(msg):
    print("[error] {}".format(msg), file=sys.stderr)


def clamp(value, low, high):
    return max(low, min(value, high))


def role_str(role):
    return flexivtdk.RoleTypeStr[int(role)]


def print_teleop_status(idx, status):
    """Print GetTeleopStatus() snapshot and the primary operator-facing issue."""
    log_info(
        "Teleop pair {} status: initialized={} started={} engaged={} stopped={} fault={} "
        "motion_restricted={} latency={:.1f}/{:.1f} ms".format(
            idx, status.initialized, status.started, status.engaged, status.stopped,
            status.fault, status.motion_restricted, status.latency_ms,
            status.latency_threshold_ms))
    if status.primary.code == flexivtdk.TeleopIssueCode.NONE:
        log_info("No teleop restriction. Safe to continue.")
        return
    issue = status.primary
    log_warn("[{}/{}] {} | {} | {}".format(
        flexivtdk.TeleopIssueLevelStr[int(issue.level)],
        flexivtdk.TeleopIssueSideStr[int(issue.side)], issue.title, issue.description,
        issue.suggestion))
    for extra in status.issues:
        if (extra.code == issue.code and extra.side == issue.side
                and extra.joint_index == issue.joint_index):
            continue
        log_warn("  also: [{}/{}] {}".format(
            flexivtdk.TeleopIssueCodeStr[int(extra.code)],
            flexivtdk.TeleopIssueSideStr[int(extra.side)], extra.title))


def print_gripper_states(gripper: flexivtdk.GripperRemoteControl):
    try:
        states = gripper.states(kIdx)
        log_info("Gripper states: width = {:.4f} m, force = {:.2f} N, is_moving = {}".format(
            states.width, states.force, states.is_moving))
    except Exception as e:
        log_warn(f"Gripper states not available yet: {e}")


class WanProTeleoperationController:
    """Console commands for TransparentCartesianTeleopWAN and optional GripperRemoteControl."""

    def __init__(self, teleop: flexivtdk.TransparentCartesianTeleopWAN,
                 gripper: Optional[flexivtdk.GripperRemoteControl], gripper_name: str,
                 di_controls_gripper: bool):
        self.teleop = teleop
        self.gripper = gripper
        self.gripper_name = gripper_name
        # True if the leader DI1 controls the gripper motion, then console motion commands are disabled
        self.di_controls_gripper = di_controls_gripper
        self.cmd = self._get_initial_axis_lock_cmd()
        self._command_map = self._create_command_map()
        self._menu = self._create_menu()

    def _is_leader(self) -> bool:
        return self.teleop.role() == flexivtdk.Role.WAN_TELEOP_LEADER

    def _default_axis_lock_cmd(self):
        cmd = flexivtdk.AxisLock()
        cmd.lock_trans_axis = [False, False, False]
        cmd.lock_ori_axis = [False, False, False]
        cmd.coord = flexivtdk.CoordType.TCP
        return cmd

    def _get_initial_axis_lock_cmd(self):
        """Initialize axis lock command. GetAxisLockState is leader-only on WAN."""
        if not self._is_leader():
            return self._default_axis_lock_cmd()
        try:
            return self.teleop.GetAxisLockState(kIdx)
        except Exception as e:
            log_warn(f"Failed to get initial axis lock state, using default: {e}")
            return self._default_axis_lock_cmd()

    def _create_command_map(self) -> Dict[str, Callable]:
        """Create a mapping of keyboard commands to their corresponding methods."""
        return {
            # Translation locks WORLD coord
            'x': lambda: self._toggle_axis_lock(0, 'trans', flexivtdk.CoordType.WORLD),
            'y': lambda: self._toggle_axis_lock(1, 'trans', flexivtdk.CoordType.WORLD),
            'z': lambda: self._toggle_axis_lock(2, 'trans', flexivtdk.CoordType.WORLD),
            # Orientation locks WORLD coord
            'q': lambda: self._toggle_axis_lock(0, 'ori', flexivtdk.CoordType.WORLD),
            'w': lambda: self._toggle_axis_lock(1, 'ori', flexivtdk.CoordType.WORLD),
            'e': lambda: self._toggle_axis_lock(2, 'ori', flexivtdk.CoordType.WORLD),
            # Translation locks TCP coord
            'X': lambda: self._toggle_axis_lock(0, 'trans', flexivtdk.CoordType.TCP),
            'Y': lambda: self._toggle_axis_lock(1, 'trans', flexivtdk.CoordType.TCP),
            'Z': lambda: self._toggle_axis_lock(2, 'trans', flexivtdk.CoordType.TCP),
            # Orientation locks TCP coord
            'Q': lambda: self._toggle_axis_lock(0, 'ori', flexivtdk.CoordType.TCP),
            'W': lambda: self._toggle_axis_lock(1, 'ori', flexivtdk.CoordType.TCP),
            'E': lambda: self._toggle_axis_lock(2, 'ori', flexivtdk.CoordType.TCP),
            # Axis lock presets
            'a': self._unlock_all_axes,
            'A': self._lock_all_axes,
            # Teleop engage/disengage
            'r': lambda: self._engage(True),
            'R': lambda: self._engage(False),
            # Null-space postures
            'i': lambda: self._set_nullspace(kPreferredJntPos),
            'I': lambda: self._set_nullspace(kHomeJntPos),
            # Max contact wrench
            'p': lambda: self._set_max_contact_wrench(kDefaultMaxContactWrench),
            # Reinit and start
            'u': self._start_teleop,
            'U': self._stop_teleop,
            # TCP message latency
            'l': self._print_latency,
            # Teleop status / fault recovery / identity
            'h': self._print_status,
            'F': self._clear_fault,
            'n': self._print_identity,
            # Gripper lifecycle
            'G': self._gripper_enable,
            'D': self._gripper_disable,
            'N': self._gripper_init,
            # Gripper motion
            'o': self._gripper_open,
            'c': self._gripper_close,
            's': self._gripper_stop,
            # Gripper states
            't': lambda: print_gripper_states(self.gripper),
            'P': self._print_gripper_params,
        }

    def _create_menu(self) -> str:
        """Create the command menu string."""
        menu = """
  --- Axis Lock (leader only) ---
    x/y/z    : Toggle translation lock in WORLD coord (X/Y/Z)
    q/w/e    : Toggle orientation lock in WORLD coord (Rx/Ry/Rz)
    X/Y/Z    : Toggle translation lock in TCP coord (X/Y/Z)
    Q/W/E    : Toggle orientation lock in TCP coord (Rx/Ry/Rz)
    a        : Unlock all axes (TCP coord)
    A        : Lock all axes (TCP coord)

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
    l        : Print current message latency (disconnected / clock mismatch / over limit)

  --- Teleop status ---
    h        : Print why teleop is restricted / paused and what to do next

  --- Fault recovery ---
    F        : Clear fault (blocking, up to 30 s), then press 'u' to restart teleop

  --- Identity ---
    n        : Print role() and robot_pair_sn()
"""
        if self.gripper is not None:
            menu += """
  --- Gripper Lifecycle (Leader only, via RPC) ---
    G        : Enable gripper on follower robot
    D        : Disable gripper on follower robot
    N        : Trigger initialization of the enabled gripper

  --- Gripper Motion (Leader only, via reliable topic, disabled when -D is enabled) ---
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

    def _apply_axis_lock(self) -> bool:
        if not self._is_leader():
            log_warn("Axis lock is only available on the leader")
            return False
        self.teleop.SetAxisLockCmd(kIdx, self.cmd)
        return True

    def _toggle_axis_lock(self, axis_index: int, lock_type: str, coord_type: flexivtdk.CoordType):
        """Toggle axis lock for the specified axis and type."""
        attr = "lock_trans_axis" if lock_type == "trans" else "lock_ori_axis"
        axes = list(getattr(self.cmd, attr))
        axes[axis_index] = not axes[axis_index]
        setattr(self.cmd, attr, axes)
        self.cmd.coord = coord_type
        if self._apply_axis_lock():
            log_info(
                f"Axis lock toggled: {lock_type}[{axis_index}] = {axes[axis_index]}, coord = {coord_type}")

    def _unlock_all_axes(self):
        """Unlock all axes in TCP coordinate system."""
        self.cmd.lock_ori_axis = [False, False, False]
        self.cmd.lock_trans_axis = [False, False, False]
        self.cmd.coord = flexivtdk.CoordType.TCP
        if self._apply_axis_lock():
            log_info("All axes unlocked")

    def _lock_all_axes(self):
        """Lock all axes in TCP coordinate system."""
        self.cmd.lock_ori_axis = [True, True, True]
        self.cmd.lock_trans_axis = [True, True, True]
        self.cmd.coord = flexivtdk.CoordType.TCP
        if self._apply_axis_lock():
            log_info("All axes locked")

    def _engage(self, engage: bool):
        """Engage or disengage teleop."""
        self.teleop.Engage(kIdx, engage)
        log_info(f"Teleop {'engaged' if engage else 'disengaged'}")

    def _set_nullspace(self, posture: List[float]):
        """Set nullspace posture."""
        self.teleop.SetNullSpacePosture(kIdx, posture)
        log_info("Nullspace posture set")

    def _set_max_contact_wrench(self, wrench: List[float]):
        """Set max contact wrench."""
        self.teleop.SetMaxContactWrench(kIdx, wrench)
        log_info("Max contact wrench set")

    def _start_teleop(self):
        """Initialize and start teleoperation."""
        self.teleop.Init()
        self.teleop.Start()
        log_info("Teleop started")

    def _stop_teleop(self):
        """Stop teleoperation."""
        self.teleop.Stop()
        log_info("Teleop stopped")

    def _print_latency(self):
        """Print current TCP message latency."""
        ok, latency_ms = self.teleop.CheckTeleopConnectionLatency(kIdx)
        if ok:
            log_info("pair {} message latency: {:.1f} ms (within limit)".format(kIdx, latency_ms))
        elif latency_ms < 0.0:
            log_warn("pair {} clock mismatch: latency {:.1f} ms".format(kIdx, latency_ms))
        elif latency_ms > 1.0e12:
            log_warn("pair {} disconnected: latency {:.1f} ms".format(kIdx, latency_ms))
        else:
            log_warn("pair {} latency over limit: {:.1f} ms".format(kIdx, latency_ms))

    def _print_status(self):
        """Print GetTeleopStatus() for the current robot pair."""
        print_teleop_status(kIdx, self.teleop.GetTeleopStatus(kIdx))

    def _clear_fault(self):
        """Clear fault of all robots, then teleop can be restarted with 'u'."""
        log_info("Clearing fault, this may take up to 30 seconds...")
        results = self.teleop.ClearFault()
        for i, ok in enumerate(results):
            log_info("Teleop pair {} clear fault: {}".format(i, ok))

    def _print_identity(self):
        """Print role() and robot_pair_sn()."""
        leader_sn, follower_sn = self.teleop.robot_pair_sn(kIdx)
        log_info(f"pair {kIdx} role={role_str(self.teleop.role())} "
                 f"leader_sn={leader_sn} follower_sn={follower_sn}")

    def _gripper_enable(self):
        self.gripper.Enable(kIdx, self.gripper_name)
        log_info(f"Follower gripper [{self.gripper_name}] enabled")

    def _gripper_disable(self):
        self.gripper.Disable(kIdx)
        log_info("Follower gripper disabled")

    def _gripper_init(self):
        self.gripper.Init(kIdx)
        log_info("Follower gripper initialization triggered")

    def _gripper_open(self):
        params = self.gripper.params(kIdx)
        velocity = clamp(0.5 * params.max_vel, params.min_vel, params.max_vel)
        force_limit = clamp(0.5 * params.max_force, params.min_force, params.max_force)
        self.gripper.Move(kIdx, params.max_width, velocity, force_limit)
        log_info("Open command sent: width = {:.4f} m, velocity = {:.4f} m/s, "
                 "force_limit = {:.2f} N".format(params.max_width, velocity, force_limit))

    def _gripper_close(self):
        params = self.gripper.params(kIdx)
        force = clamp(0.5 * params.max_force, params.min_force, params.max_force)
        self.gripper.Grasp(kIdx, force)
        log_info("Grasp command sent: force = {:.2f} N".format(force))

    def _gripper_stop(self):
        self.gripper.Stop(kIdx)
        log_info("Stop command sent")

    def _print_gripper_params(self):
        try:
            params = self.gripper.params(kIdx)
            log_info("Gripper params: width = [{:.4f}, {:.4f}] m, velocity = [{:.4f}, {:.4f}] m/s, "
                     "force = [{:.2f}, {:.2f}] N".format(
                         params.min_width, params.max_width, params.min_vel, params.max_vel,
                         params.min_force, params.max_force))
        except Exception as e:
            log_warn(f"Gripper params not available yet: {e}")

    def handle_command(self, user_input: str):
        """Handle a single user command. A failed command is reported but never exits."""
        if not user_input:
            log_warn("Empty command!")
            print(self._menu)
            return

        ch = user_input[0]
        if ch in kGripperCommands and self.gripper is None:
            log_warn("Gripper commands require [-n gripper_name].")
            return
        if self.gripper is not None and not self._is_leader() and ch in kGripperLeaderCommands:
            log_warn(f"Command '{ch}' is only available on the leader")
            return
        if self.di_controls_gripper and ch in kGripperMotionCommands:
            log_warn(f"Gripper motion is controlled by DI1 (-D enabled); console command '{ch}' "
                     "is disabled.")
            return
        if ch not in self._command_map:
            log_warn("Invalid command!")
            print(self._menu)
            return

        try:
            self._command_map[ch]()
        except Exception as e:
            log_error(f"Command '{ch}' failed: {e}")
            try:
                print_teleop_status(kIdx, self.teleop.GetTeleopStatus(kIdx))
            except Exception as status_e:
                log_error(f"Failed to query teleop status: {status_e}")


# Read digital input and engage/disengage teleop accordingly. Leader DI0 follows the deadman
# switch. With [-n], a rising edge on DI1 toggles the follower gripper open/closed.
def read_digital_input_task(teleop: flexivtdk.TransparentCartesianTeleopWAN,
                            gripper: Optional[flexivtdk.GripperRemoteControl]):
    logged_idle = False
    di1_stable = False
    di1_counter = 0
    gripper_opened = False
    last_error = None

    while not _stop_event.is_set():
        try:
            status = teleop.GetTeleopStatus(kIdx)
            # Robot fault / Stop() drops teleop back to not-started. Engage() is
            # invalid in that state and must not be polled every cycle.
            if (not status.started) or status.stopped:
                if not logged_idle:
                    log_warn(
                        "ReadDigitalInputTask: teleop is not started, pause Engage "
                        "(call Init + Start to resume)")
                    logged_idle = True
            else:
                logged_idle = False
                digital_inputs = teleop.digital_inputs(kIdx)
                teleop.Engage(kIdx, bool(digital_inputs[0]))

                if gripper is not None and len(digital_inputs) > 1:
                    di1_raw = bool(digital_inputs[1])
                    if di1_raw != di1_stable:
                        di1_counter += 1
                        if di1_counter >= 3:
                            di1_stable = di1_raw
                            di1_counter = 0
                            if di1_stable:
                                try:
                                    params = gripper.params(kIdx)
                                    if not gripper_opened:
                                        width = clamp(0.5 * params.max_width, params.min_width,
                                                      params.max_width)
                                        velocity = clamp(0.5 * params.max_vel, params.min_vel,
                                                         params.max_vel)
                                        force_limit = clamp(0.5 * params.max_force,
                                                            params.min_force, params.max_force)
                                        gripper.Move(kIdx, width, velocity, force_limit)
                                        gripper_opened = True
                                        log_info("DI1 pressed: opening gripper to width = "
                                                 "{:.4f} m".format(width))
                                    else:
                                        force = clamp(40.0, params.min_force, params.max_force)
                                        gripper.Grasp(kIdx, force)
                                        gripper_opened = False
                                        log_info("DI1 pressed: closing gripper with force = "
                                                 "{:.2f} N".format(force))
                                except Exception as e:
                                    log_warn(f"DI1 gripper action failed: {e}")
                    else:
                        di1_counter = 0
            last_error = None
        except Exception as e:
            msg = str(e)
            if msg != last_error:
                log_error(f"Exception in ReadDigitalInputTask: {e}")
                last_error = msg
        time.sleep(0.01)
    log_info("ReadDigitalInputTask exiting.")


# monitor robot fault and prompt the operator to recover
def fault_monitor_task(teleop: flexivtdk.TransparentCartesianTeleopWAN):
    last_fault = False
    last_error = None
    while not _stop_event.is_set():
        try:
            fault = teleop.fault(kIdx)
            fault_changed = (fault != last_fault)
            last_fault = fault
            if fault_changed and fault:
                log_error("Fault detected. Press 'h' for details, 'F' to clear fault, then 'u' to "
                          "restart teleop.")
                print_teleop_status(kIdx, teleop.GetTeleopStatus(kIdx))
            elif fault_changed:
                log_info("Fault cleared. Press 'u' to restart teleop.")
            last_error = None
        except Exception as e:
            msg = str(e)
            if msg != last_error:
                log_error(f"Exception in FaultMonitorTask: {e}")
                last_error = msg
        time.sleep(0.1)
    log_info("FaultMonitorTask exiting.")


# console task to read user inputs and send commands accordingly
def console_task(controller: WanProTeleoperationController):
    print(controller._menu)

    while not _stop_event.is_set():
        try:
            user_input = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            log_info("Console input closed.")
            _stop_event.set()
            break

        controller.handle_command(user_input)

    log_info("Console thread exiting.")


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Transparent Cartesian Teleop WAN example, TDK Professional Edition (Python)")
    parser.add_argument("-l", "--leader_sn", required=True, help="serial number of leader robot")
    parser.add_argument("-f", "--follower_sn", required=True, help="serial number of follower robot")
    parser.add_argument("-r", "--role", required=True, choices=["leader", "follower"], help="role in teleop")
    parser.add_argument("-c", "--client-config", required=True,
                        help="path to client.conf from the TDK Server credential package")
    parser.add_argument("-A", "--lan-ip", action="append", default=[],
                        help="LAN IPv4 address of the NIC connected to the robot")
    parser.add_argument("-n", "--gripper-name", default="",
                        help="follower gripper device name configured in Elements -> Device. To enable "
                             "GripperRemoteControl, both leader and follower sides must provide the same "
                             "gripper name")
    parser.add_argument("-D", "--enable-digital-input", action="store_true",
                        help="enable digital input reading task. Leader DI0 engages arm teleop; with -n, "
                             "DI1 toggles the follower gripper and the console gripper motion commands "
                             "(o/c/s) are disabled")
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None):
    # Program Setup
    # ==============================================================================================
    # Parse arguments
    args = parse_args(argv)
    if not args.lan_ip:
        log_warn("LAN whitelist is not provided, RDK will search all network interfaces.")

    if args.role == 'follower':
        role = flexivtdk.Role.WAN_TELEOP_FOLLOWER
    else:  # leader
        role = flexivtdk.Role.WAN_TELEOP_LEADER

    # Network configuration (Professional Edition, TDK Server credential)
    network_cfg = flexivtdk.NetworkCfgPro()
    network_cfg.client_config_file = args.client_config
    network_cfg.lan_interface_whitelist = args.lan_ip

    robot_pairs = [(args.leader_sn, args.follower_sn)]

    teleop = None
    console_thr = None
    fault_monitor_thr = None
    pedal_thread = None

    try:
        # TDK Initialization
        # ==========================================================================================
        teleop = flexivtdk.TransparentCartesianTeleopWAN(robot_pairs, role, network_cfg)
        leader_sn, follower_sn = teleop.robot_pair_sn(kIdx)
        log_info(f"This instance role={role_str(teleop.role())} leader_sn={leader_sn} "
                 f"follower_sn={follower_sn}")

        # Allocate the gripper remote control object on top of the arm teleop object only if
        # [-n] is provided. It reuses the role, robot serial number pairs and rdk::Robot
        # connection of the arm teleop. Without [-n] the program runs as basic arm teleop only.
        gripper = None
        if args.gripper_name:
            gripper = flexivtdk.GripperRemoteControl(teleop, network_cfg)
            log_info(f"Gripper remote control is ENABLED, target gripper device: [{args.gripper_name}]")
        else:
            log_info("Gripper remote control is DISABLED (no [-n gripper_name] provided).")

        teleop.Init()
        teleop.Start()
        teleop.SetMaxContactWrench(kIdx, kDefaultMaxContactWrench)

        # Follower side: enable the configured gripper locally so the leader can see states and
        # send commands without waiting for the Enable RPC.
        if role == flexivtdk.Role.WAN_TELEOP_FOLLOWER and gripper is not None:
            try:
                gripper.EnableLocal(kIdx, args.gripper_name)
            except Exception as e:
                log_error(f"Failed to auto-enable gripper [{args.gripper_name}] on follower: {e}. "
                          "The leader can still retry via the 'G' console command.")

        start_pedal_thread = (role == flexivtdk.Role.WAN_TELEOP_LEADER and args.enable_digital_input)
        di_controls_gripper = start_pedal_thread and gripper is not None

        controller = WanProTeleoperationController(
            teleop, gripper, args.gripper_name, di_controls_gripper)
        console_thr = threading.Thread(target=console_task, args=(controller,), daemon=True)
        console_thr.start()

        fault_monitor_thr = threading.Thread(target=fault_monitor_task, args=(teleop,), daemon=True)
        fault_monitor_thr.start()

        if start_pedal_thread:
            log_info("Starting ReadDigitalInputTask thread as role is 'leader' and requested by -D "
                     "flag. DI0: arm teleop engagement; DI1: "
                     + ("toggles follower gripper open/close" if gripper is not None
                        else "unused (gripper feature disabled, no -n)") + ".")
            pedal_thread = threading.Thread(
                target=read_digital_input_task, args=(teleop, gripper), daemon=True)
            pedal_thread.start()
        else:
            log_info("ReadDigitalInputTask thread NOT started (role is not 'leader' or -D flag not "
                     "provided).")

        console_thr.join()

        # Set the stop event when console thread exits
        _stop_event.set()

        if fault_monitor_thr:
            fault_monitor_thr.join(timeout=1.0)
        if pedal_thread:
            pedal_thread.join(timeout=1.0)

        teleop.Stop()
        log_info("WAN Teleop stopped.")

    except KeyboardInterrupt:
        log_info("Program interrupted by user")
        _stop_event.set()
    except Exception as e:
        log_error(f"Exception in main: {e}")
        _stop_event.set()
        sys.exit(1)
    finally:
        # Ensure threads are joined properly
        if console_thr and console_thr.is_alive():
            console_thr.join(timeout=1.0)
        if fault_monitor_thr and fault_monitor_thr.is_alive():
            fault_monitor_thr.join(timeout=1.0)
        if pedal_thread and pedal_thread.is_alive():
            pedal_thread.join(timeout=1.0)

        # Ensure teleop is stopped
        if teleop:
            try:
                teleop.Stop()
            except Exception:
                pass  # Ignore errors during cleanup


if __name__ == "__main__":
    main()
