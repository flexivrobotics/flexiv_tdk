#!/usr/bin/env python

"""
transparent_cartesian_teleop_wan.py

Example usage of Transparent Cartesian teleoperation under Wide Area Network,
controlling a follower robot using a leader robot with transparent force feedback.
Supports both keyboard and digital input engage/disengage signal reading,
with various axes lock modes, nullspace posture, and max contact wrench setting.

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

class WanTeleoperationController:
    """Encapsulates WAN teleoperation functionality for better organization and maintainability."""
    
    def __init__(self, teleop: flexivtdk.TransparentCartesianTeleopWAN):
        self.teleop = teleop
        self.index = 0
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
            return self.teleop.GetAxisLockState(self.index)
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
            # Axis lock presets (u/U are already used for Init/Stop on WAN)
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
        }
    
    def _create_menu(self) -> str:
        """Create the command menu string."""
        return """
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

  --- Help ---
    Any other key to show this help menu
    """

    def _apply_axis_lock(self) -> bool:
        if not self._is_leader():
            log_warn("Axis lock is only available on the leader")
            return False
        self.teleop.SetAxisLockCmd(self.index, self.cmd)
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

    def _start_teleop(self):
        """Initialize and start teleoperation."""
        self.teleop.Init()
        self.teleop.Start()
        log_info("Teleop started")
    
    def _stop_teleop(self):
        """Stop teleoperation."""
        self.teleop.Stop()
        log_info("Teleop stopped")

    def _clear_fault(self):
        """Clear fault of all robots, then teleop can be restarted with 'u'."""
        log_info("Clearing fault, this may take up to 30 seconds...")
        results = self.teleop.ClearFault()
        for i, ok in enumerate(results):
            log_info("Teleop pair {} clear fault: {}".format(i, ok))
    
    def _print_latency(self):
        """Print current TCP message latency."""
        ok, latency_ms = self.teleop.CheckTeleopConnectionLatency(self.index)
        if ok:
            log_info("pair {} message latency: {:.1f} ms (within limit)".format(
                self.index, latency_ms))
        elif latency_ms < 0.0:
            log_warn("pair {} clock mismatch: latency {:.1f} ms".format(self.index, latency_ms))
        elif latency_ms > 1.0e12:
            log_warn("pair {} disconnected: latency {:.1f} ms".format(self.index, latency_ms))
        else:
            log_warn("pair {} latency over limit: {:.1f} ms".format(self.index, latency_ms))

    def _print_status(self):
        """Print GetTeleopStatus() for the current robot pair."""
        print_teleop_status(self.index, self.teleop.GetTeleopStatus(self.index))

    def _print_identity(self):
        """Print role() and robot_pair_sn()."""
        leader_sn, follower_sn = self.teleop.robot_pair_sn(self.index)
        log_info(
            f"pair {self.index} role={role_str(self.teleop.role())} "
            f"leader_sn={leader_sn} follower_sn={follower_sn}")
    
    def _engage(self, engage: bool):
        """Engage or disengage teleop."""
        self.teleop.Engage(self.index, engage)
        log_info(f"Teleop {'engaged' if engage else 'disengaged'}")
    
    def _set_nullspace(self, posture: List[float]):
        """Set nullspace posture."""
        self.teleop.SetNullSpacePosture(self.index, posture)
        log_info("Nullspace posture set")
    
    def _set_max_contact_wrench(self, wrench: List[float]):
        """Set max contact wrench."""
        self.teleop.SetMaxContactWrench(self.index, wrench)
        log_info("Max contact wrench set")
    
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


# read digital input and engage/disengage teleop accordingly
def read_digital_input_task(teleop: flexivtdk.TransparentCartesianTeleopWAN):
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
                # Digital input for WAN returns a list of inputs
                di_state = teleop.digital_inputs(idx)
                # Use first DI port as engage/disengage signal
                if di_state and len(di_state) > 0:
                    teleop.Engage(idx, bool(di_state[0]))
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
            fault = teleop.fault(0)
            fault_changed = (fault != last_fault)
            last_fault = fault
            if fault_changed and fault:
                log_error("Fault detected. Press 'h' for details, 'F' to clear fault, then 'u' to "
                          "restart teleop.")
                print_teleop_status(0, teleop.GetTeleopStatus(0))
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
def console_task(teleop: flexivtdk.TransparentCartesianTeleopWAN):
    controller = WanTeleoperationController(teleop)
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


def port_number(value: str) -> int:
    """argparse type for a TCP port in range 1-65535."""
    try:
        port = int(value)
    except ValueError:
        raise argparse.ArgumentTypeError(f"invalid port number: {value}")
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError(f"invalid port number: {value}, valid range is 1-65535")
    return port


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Transparent Cartesian Teleop WAN example (Python)")
    parser.add_argument("-l", "--leader_sn", required=True, help="serial number of leader robot")
    parser.add_argument("-f", "--follower_sn", required=True, help="serial number of follower robot")
    parser.add_argument("-r", "--role", required=True, choices=["leader", "follower"], help="role in teleop")
    parser.add_argument("-t", "--tcp-role", required=True, choices=["server", "client"], help="tcp role")
    parser.add_argument("-i", "--public-ip", required=True, help="public IPv4 address of TCP server")
    parser.add_argument("-p", "--port", required=True, type=port_number, help="listening port of TCP server")
    parser.add_argument("-A", "--lan-ip", action="append",
                        help="LAN IPv4 address of the NIC connected to the robot", default=[])
    parser.add_argument("-W", "--wan-iface", action="append",
                        help="WAN interface name, e.g. wlo1 or enp3s0", default=[])
    parser.add_argument("-D", "--enable-digital-input", action="store_true", help="enable digital input reading task")
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None):
    # Program Setup
    # ==============================================================================================
    # Parse arguments
    args = parse_args(argv)
    if not args.lan_ip:
        log_warn("LAN whitelist is not provided, will search all network interfaces.")
    if not args.wan_iface:
        log_warn("WAN interface whitelist is not provided, will search all network interfaces.")
    
    # Determine role
    if args.role == 'follower':
        role = flexivtdk.Role.WAN_TELEOP_FOLLOWER
    else:  # leader
        role = flexivtdk.Role.WAN_TELEOP_LEADER
    
    # Network configuration (Standard Edition TCP peer-to-peer)
    network_cfg = flexivtdk.NetworkCfgStd()
    network_cfg.is_tcp_server = (args.tcp_role == 'server')
    network_cfg.public_ipv4_address = args.public_ip
    network_cfg.listening_port = args.port
    if args.lan_ip:
        network_cfg.lan_interface_whitelist = args.lan_ip
    if args.wan_iface:
        network_cfg.wan_interface_whitelist = args.wan_iface
    
    # Robot pairs
    robot_pairs = [(args.leader_sn, args.follower_sn)]
    
    teleop = None
    console_thr = None
    fault_monitor_thr = None
    pedal_thread = None
    
    try:
        # TDK Initialization
        # ==========================================================================================
        # Instantiate teleop interface
        teleop = flexivtdk.TransparentCartesianTeleopWAN(robot_pairs, role, network_cfg)
        leader_sn, follower_sn = teleop.robot_pair_sn(0)
        log_info(
            f"This instance role={role_str(teleop.role())} leader_sn={leader_sn} follower_sn={follower_sn}")

        # Initialize teleop, this will Clear fault, Calibrate the force sensors, initialize teleop control parameters, etc. 
        teleop.Init()

        # Start teleop process
        teleop.Start()
        
        # Set max contact wrench
        teleop.SetMaxContactWrench(0, kDefaultMaxContactWrench)
        
        log_info("WAN Teleop started.")

        # Start console task thread
        console_thr = threading.Thread(target=console_task, args=(teleop,), daemon=True)
        console_thr.start()
        log_info("Console task started.")

        # Start fault monitor task thread
        fault_monitor_thr = threading.Thread(target=fault_monitor_task, args=(teleop,), daemon=True)
        fault_monitor_thr.start()

        # Start digital input reading task thread only if role is leader AND -D is given
        if args.role == 'leader' and args.enable_digital_input:
            log_info("Starting ReadDigitalInputTask thread as role is 'leader' and requested by --enable-digital-input flag.")
            pedal_thread = threading.Thread(target=read_digital_input_task, args=(teleop,), daemon=True)
            pedal_thread.start()
        else:
            log_info("ReadDigitalInputTask thread NOT started (role is not 'leader' or --enable-digital-input flag not provided).")

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
            except:
                pass  # Ignore errors during cleanup


if __name__ == "__main__":
    main()
