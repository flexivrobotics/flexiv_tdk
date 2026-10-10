#!/usr/bin/env python

"""
transparent_cartesian_teleop_lan.py

Example usage of Transparent Cartesian teleoperation under Local Area Network,
controlling a follower robot using a leader robot with transparent force feedback.
Supports both keyboard and digital input engage/disengage signal reading, 
with various axes lock modes, force scaling, and max contact wrench setting, etc.

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

# Global thread-safe stop event
_stop_event = threading.Event()

def log_info(msg):
    print("[info] {}".format(msg))


def log_warn(msg):
    print("[warn] {}".format(msg), file=sys.stderr)


def log_error(msg):
    print("[error] {}".format(msg), file=sys.stderr)


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

class TeleoperationController:
    """Encapsulates teleoperation functionality for better organization and maintainability."""
    
    def __init__(self, teleop: flexivtdk.TransparentCartesianTeleopLAN):
        self.teleop = teleop
        self.index = 0
        self.cmd = self._get_initial_axis_lock_cmd()
        self._command_map = self._create_command_map()
        self._menu = self._create_menu()
    
    def _get_initial_axis_lock_cmd(self):
        """Initialize axis lock command object."""
        try:
            return self.teleop.GetAxisLockState(self.index)
        except Exception as e:
            # Fallback if GetAxisLockState overload returns differently
            cmd = flexivtdk.AxisLock()
            cmd.lock_trans_axis = [False, False, False]
            cmd.lock_ori_axis = [False, False, False]
            cmd.coord = flexivtdk.CoordType.TCP
            log_warn(f"Failed to get initial axis lock state, using default: {e}")
            return cmd
    
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
            # Teleop engage/disengage
            'r': lambda: self._safe_engage(True),
            'R': lambda: self._safe_engage(False),
            # Wrench feedback scaling
            't': lambda: self._safe_set_wrench_feedback_scaling(0.5),
            'T': lambda: self._safe_set_wrench_feedback_scaling(2.0),
            'c': lambda: self._safe_set_wrench_feedback_scaling(1.0),
            # Presets: unlock/lock all axes in TCP coord
            'u': self._unlock_all_axes,
            'U': self._lock_all_axes,
            # Null-space postures
            'i': lambda: self._safe_set_leader_nullspace(kPreferredJntPos),
            'I': lambda: self._safe_set_leader_nullspace(kHomeJntPos),
            'o': lambda: self._safe_set_follower_nullspace(kPreferredJntPos),
            'O': lambda: self._safe_set_follower_nullspace(kHomeJntPos),
            # Max contact wrench (follower)
            'p': lambda: self._safe_set_follower_max_contact_wrench(kDefaultMaxContactWrench),
            # Start/stop teleop
            'b': self._stop_teleop,
            'B': self._start_teleop,
            # Check teleop stopped state
            's': self._check_teleop_state,
            'h': self._print_status,
            'F': self._clear_fault,
            'H': self._home_all,
            # Print action/state
            'P': self._print_action_state,
        }
    
    def _create_menu(self) -> str:
        """Create the command menu string."""
        return """
  --- Axis Lock ---
    x/y/z    : Toggle translation lock in WORLD coord (X/Y/Z)
    q/w/e    : Toggle orientation lock in WORLD coord (Rx/Ry/Rz)
    X/Y/Z    : Toggle translation lock in TCP coord (X/Y/Z)
    Q/W/E    : Toggle orientation lock in TCP coord (Rx/Ry/Rz)

  --- Teleop Engagement ---
    r        : Engage teleop
    R        : Disengage teleop

  --- Wrench Feedback Scaling ---
    t        : Set wrench feedback scaling to 0.5
    T        : Set wrench feedback scaling to 2.0
    c        : Reset wrench feedback scaling to 1.0 default value

  --- Axis Lock Presets ---
    u        : Unlock all axes (TCP coord)
    U        : Lock all axes (TCP coord)

  --- Null Space Posture ---
    i/I      : Set local nullspace to Preferred/Home posture
    o/O      : Set remote nullspace to Preferred/Home posture

  --- Max Contact Wrench ---
    p        : Set default remote max contact wrench

  --- Start/Stop ---
    b        : Stop teleop
    B        : Start teleop

  --- Is teleop stopped or not ---
    s        : Query if teleop stopped or not

  --- Teleop status ---
    h        : Print why teleop is restricted / paused and what to do next

  --- Fault recovery ---
    F        : Clear fault (blocking, up to 30 s), then press 'B' to restart teleop

  --- Home all robots ---
    H        : Stop teleop and home all robots. Confirm the workspace is clear first.

  --- Print action/states via instance ---
    P        : Print follower action and state

  --- Help ---
    Any other key to show this help menu
    """
    
    def _toggle_axis_lock(self, axis_index: int, lock_type: str, coord_type: flexivtdk.CoordType):
        """Toggle axis lock for the specified axis and type."""
        attr = "lock_trans_axis" if lock_type == "trans" else "lock_ori_axis"
        axes = list(getattr(self.cmd, attr))
        axes[axis_index] = not axes[axis_index]
        setattr(self.cmd, attr, axes)
        self.cmd.coord = coord_type
        self.teleop.SetAxisLockCmd(self.index, self.cmd)
        log_info(
            f"Axis lock toggled: {lock_type}[{axis_index}] = {axes[axis_index]}, coord = {coord_type}")
    
    def _unlock_all_axes(self):
        """Unlock all axes in TCP coordinate system."""
        self.cmd.lock_ori_axis = [False, False, False]
        self.cmd.lock_trans_axis = [False, False, False]
        self.cmd.coord = flexivtdk.CoordType.TCP
        self.teleop.SetAxisLockCmd(self.index, self.cmd)
        log_info("All axes unlocked")
    
    def _lock_all_axes(self):
        """Lock all axes in TCP coordinate system."""
        self.cmd.lock_ori_axis = [True, True, True]
        self.cmd.lock_trans_axis = [True, True, True]
        self.cmd.coord = flexivtdk.CoordType.TCP
        self.teleop.SetAxisLockCmd(self.index, self.cmd)
        log_info("All axes locked")
    
    def _start_teleop(self):
        """Initialize and start teleoperation."""
        self.teleop.Init()
        self.teleop.Start()
        log_info("Teleop started")
    
    def _stop_teleop(self):
        """Stop teleoperation."""
        self.teleop.Stop()
        log_info("Teleop stopped")

    def _home_all(self):
        """Stop teleoperation and home all robots."""
        self.teleop.Stop()
        self.teleop.HomeAll()
        log_info("Teleop stopped for homing")

    def _clear_fault(self):
        """Clear fault of all robots, then teleop can be restarted with 'B'."""
        log_info("Clearing fault, this may take up to 30 seconds...")
        results = self.teleop.ClearFault()
        for i, (leader_ok, follower_ok) in enumerate(results):
            log_info("Teleop pair {} clear fault: leader={} follower={}".format(
                i, leader_ok, follower_ok))

    def _print_action_state(self):
        """Print follower action and state."""
        _leader, follower = self.teleop.instances(self.index)
        log_info("Follower q: [{}]".format(
            ", ".join("{:.2f}".format(v) for v in follower.states().q)))
        log_info("Follower q_d: [{}]".format(
            ", ".join("{:.2f}".format(v) for v in follower.actions().q_d)))
    
    def _check_teleop_state(self):
        """Check and log the teleoperation state."""
        stopped = self.teleop.stopped(self.index)
        log_info("Teleop pair {} {}".format(self.index, "stopped" if stopped else "started"))

    def _print_status(self):
        """Print GetTeleopStatus() for the current robot pair."""
        print_teleop_status(self.index, self.teleop.GetTeleopStatus(self.index))
    
    def _safe_engage(self, engage: bool):
        """Engage or disengage teleop."""
        self.teleop.Engage(self.index, engage)
        log_info(f"Teleop {'engaged' if engage else 'disengaged'}")
    
    def _safe_set_wrench_feedback_scaling(self, factor: float):
        """Set wrench feedback scaling."""
        self.teleop.SetWrenchFeedbackScalingFactor(self.index, factor)
        log_info(f"Wrench feedback scaling factor set to {factor}")
    
    def _safe_set_leader_nullspace(self, posture: List[float]):
        """Set leader nullspace posture."""
        self.teleop.SetLeaderNullSpacePosture(self.index, posture)
        log_info("Leader nullspace posture set")
    
    def _safe_set_follower_nullspace(self, posture: List[float]):
        """Set follower nullspace posture."""
        self.teleop.SetFollowerNullSpacePosture(self.index, posture)
        log_info("Follower nullspace posture set")
    
    def _safe_set_follower_max_contact_wrench(self, wrench: List[float]):
        """Set follower max contact wrench."""
        self.teleop.SetFollowerMaxContactWrench(self.index, wrench)
        log_info("Follower max contact wrench set")
    
    def handle_command(self, user_input: str):
        """Handle a single user command. A failed command is reported but never exits."""
        if not user_input:
            log_warn("Empty command!")
            print(self._menu)
            return
        
        ch = user_input[0]
        if ch not in self._command_map:
            log_warn("Invalid command!")
            print(self._menu)
            return

        try:
            self._command_map[ch]()
        except Exception as e:
            log_error(f"Command '{ch}' failed: {e}")
            try:
                print_teleop_status(self.index, self.teleop.GetTeleopStatus(self.index))
            except Exception as status_e:
                log_error(f"Failed to query teleop status: {status_e}")


# monitor robot fault and prompt the operator to recover
def fault_monitor_task(teleop: flexivtdk.TransparentCartesianTeleopLAN):
    last_fault = False
    last_error = None
    while not _stop_event.is_set():
        try:
            fault = teleop.any_fault()
            fault_changed = (fault != last_fault)
            last_fault = fault
            if fault_changed and fault:
                log_error("Fault detected. Press 'h' for details, 'F' to clear fault, then 'B' to "
                          "restart teleop.")
                print_teleop_status(0, teleop.GetTeleopStatus(0))
            elif fault_changed:
                log_info("Fault cleared. Press 'B' to restart teleop.")
            last_error = None
        except Exception as e:
            msg = str(e)
            if msg != last_error:
                log_error(f"Exception in FaultMonitorTask: {e}")
                last_error = msg
        time.sleep(0.1)
    log_info("FaultMonitorTask exiting.")


# read digital input and engage/disengage teleop accordingly
def read_digital_input_task(teleop: flexivtdk.TransparentCartesianTeleopLAN):
    idx = 0
    logged_idle = False
    last_error = None
    while not _stop_event.is_set():
        try:
            status = teleop.GetTeleopStatus(idx)
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
                # digital_inputs for LAN returns tuple (leader_inputs, follower_inputs)
                di_pair = teleop.digital_inputs(idx)
                leader_di, _follower_di = di_pair
                # use leader's first DI port as engage/disengage signal
                if leader_di and len(leader_di) > 0:
                    teleop.Engage(idx, bool(leader_di[0]))
            last_error = None
        except Exception as e:
            msg = str(e)
            if msg != last_error:
                log_error(f"Exception in ReadDigitalInputTask: {e}")
                last_error = msg
        time.sleep(0.01)
    log_info("ReadDigitalInputTask exiting.")


# console task to read user inputs and send commands accordingly
def console_task(teleop: flexivtdk.TransparentCartesianTeleopLAN):
    controller = TeleoperationController(teleop)
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
    return


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Transparent Cartesian Teleop LAN example")
    parser.add_argument("-l", "--leader_sn", required=True, help="serial number of leader robot")
    parser.add_argument("-f", "--follower_sn", required=True, help="serial number of follower robot")
    parser.add_argument("-A", "--lan-ip", action="append",
                        help="LAN IPv4 address of the NIC connected to the robot", default=[])
    parser.add_argument("-D", "--enable-digital-input", action="store_true", help="enable digital input reading task")
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None):
    # Program Setup
    # ==============================================================================================
    # Parse arguments
    args = parse_args(argv)
    robot_pairs = [(args.leader_sn, args.follower_sn)]
    if not args.lan_ip:
        log_warn("network_interface_whitelist is not provided, will search all network interfaces.")
    
    teleop = None
    console_thr = None
    fault_monitor_thr = None
    pedal_thread = None
    
    try:
        # TDK Initialization
        # ==========================================================================================
        # Instantiate teleop interface
        teleop = flexivtdk.TransparentCartesianTeleopLAN(robot_pairs, args.lan_ip)
        teleop.Init()
        teleop.Start()
        log_info("Teleop started.")

        # Start console task thread
        console_thr = threading.Thread(target=console_task, args=(teleop,), daemon=True)
        console_thr.start()
        log_info("Console task started.")

        # Start fault monitor task thread
        fault_monitor_thr = threading.Thread(target=fault_monitor_task, args=(teleop,), daemon=True)
        fault_monitor_thr.start()

        # Start digital input reading task thread accordingly
        if args.enable_digital_input:
            pedal_thread = threading.Thread(target=read_digital_input_task, args=(teleop,), daemon=True)
            pedal_thread.start()
            log_info("DI reading task started.")

        console_thr.join()

        # Set the stop event when console thread exits
        _stop_event.set()  
        
        if fault_monitor_thr:
            fault_monitor_thr.join(timeout=1.0)
        if pedal_thread:
            pedal_thread.join(timeout=1.0)

        # Stop teleop process 
        if teleop:
            teleop.Stop()
        log_info("Teleop stopped.")
        
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
            except:
                pass  # Ignore errors during cleanup


if __name__ == "__main__":
    main()



