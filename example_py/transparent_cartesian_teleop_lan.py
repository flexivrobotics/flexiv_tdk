#!/usr/bin/env python

"""
transparent_cartesian_teleop_lan.py

Example usage of Transparent Cartesian teleoperation under Local Area Network,
controlling a follower robot using a leader robot with transparent force feedback.
Supports both keyboard and digital input engage/disengage signal reading, 
with various axes lock modes, force scaling, and max contact wrench setting, etc.

"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import argparse
import threading
import time
import sys
import spdlog
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
kDefaultMaxContactWrench = [25.0, 25.0, 25.0, 40.0, 40.0, 40.0]

# Global thread-safe stop event
_stop_event = threading.Event()

# Logger setup
logger = spdlog.ConsoleLogger("Example")

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
            logger.warning(f"Failed to get initial axis lock state, using default: {e}")
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

  --- Help ---
    Any other key to show this help menu
    """
    
    def _toggle_axis_lock(self, axis_index: int, lock_type: str, coord_type: flexivtdk.CoordType):
        """Toggle axis lock for the specified axis and type."""
        try:
            if lock_type == 'trans':
                self.cmd.lock_trans_axis[axis_index] = not self.cmd.lock_trans_axis[axis_index]
            elif lock_type == 'ori':
                self.cmd.lock_ori_axis[axis_index] = not self.cmd.lock_ori_axis[axis_index]
            
            self.cmd.coord = coord_type
            self.teleop.SetAxisLockCmd(self.index, self.cmd)
            logger.info(f"Axis lock toggled: {lock_type}[{axis_index}] = {not (self.cmd.lock_trans_axis[axis_index] if lock_type == 'trans' else self.cmd.lock_ori_axis[axis_index])}, coord = {coord_type}")
        except Exception as e:
            logger.error(f"Failed to toggle axis lock: {e}")
    
    def _unlock_all_axes(self):
        """Unlock all axes in TCP coordinate system."""
        try:
            self.cmd.lock_ori_axis = [False, False, False]
            self.cmd.lock_trans_axis = [False, False, False]
            self.cmd.coord = flexivtdk.CoordType.TCP
            self.teleop.SetAxisLockCmd(self.index, self.cmd)
            logger.info("All axes unlocked")
        except Exception as e:
            logger.error(f"Failed to unlock all axes: {e}")
    
    def _lock_all_axes(self):
        """Lock all axes in TCP coordinate system."""
        try:
            self.cmd.lock_ori_axis = [True, True, True]
            self.cmd.lock_trans_axis = [True, True, True]
            self.cmd.coord = flexivtdk.CoordType.TCP
            self.teleop.SetAxisLockCmd(self.index, self.cmd)
            logger.info("All axes locked")
        except Exception as e:
            logger.error(f"Failed to lock all axes: {e}")
    
    def _start_teleop(self):
        """Initialize and start teleoperation."""
        try:
            # Check if already initialized to avoid re-initialization
            if not hasattr(self, '_initialized') or not self._initialized:
                self.teleop.Init()
                self._initialized = True
            self.teleop.Start()
            logger.info("Teleop started")
        except Exception as e:
            logger.error(f"Failed to start teleop: {e}")
    
    def _stop_teleop(self):
        """Stop teleoperation and set the stop event."""
        try:
            self.teleop.Stop()
            _stop_event.set()
            logger.info("Teleop stopped")
        except Exception as e:
            logger.error(f"Failed to stop teleop: {e}")
            _stop_event.set()
    
    def _check_teleop_state(self):
        """Check and log the teleoperation state."""
        try:
            stopped = self.teleop.stopped(self.index)
            logger.info("Teleop pair {} {}", self.index, "stopped" if stopped else "started")
        except Exception as e:
            logger.error(f"Failed to check teleop state: {e}")
    
    def _safe_engage(self, engage: bool):
        """Safely engage or disengage teleop with error handling."""
        try:
            self.teleop.Engage(self.index, engage)
            logger.info(f"Teleop {'engaged' if engage else 'disengaged'}")
        except Exception as e:
            logger.error(f"Failed to {'engage' if engage else 'disengage'} teleop: {e}")
    
    def _safe_set_wrench_feedback_scaling(self, factor: float):
        """Safely set wrench feedback scaling with error handling."""
        try:
            self.teleop.SetWrenchFeedbackScalingFactor(self.index, factor)
            logger.info(f"Wrench feedback scaling factor set to {factor}")
        except Exception as e:
            logger.error(f"Failed to set wrench feedback scaling: {e}")
    
    def _safe_set_leader_nullspace(self, posture: List[float]):
        """Safely set leader nullspace posture with error handling."""
        try:
            self.teleop.SetLeaderNullSpacePosture(self.index, posture)
            logger.info("Leader nullspace posture set")
        except Exception as e:
            logger.error(f"Failed to set leader nullspace posture: {e}")
    
    def _safe_set_follower_nullspace(self, posture: List[float]):
        """Safely set follower nullspace posture with error handling."""
        try:
            self.teleop.SetFollowerNullSpacePosture(self.index, posture)
            logger.info("Follower nullspace posture set")
        except Exception as e:
            logger.error(f"Failed to set follower nullspace posture: {e}")
    
    def _safe_set_follower_max_contact_wrench(self, wrench: List[float]):
        """Safely set follower max contact wrench with error handling."""
        try:
            self.teleop.SetFollowerMaxContactWrench(self.index, wrench)
            logger.info("Follower max contact wrench set")
        except Exception as e:
            logger.error(f"Failed to set follower max contact wrench: {e}")
    
    def handle_command(self, user_input: str) -> bool:
        """Handle a single user command."""
        if not user_input:
            print(self._menu)
            return True
        
        ch = user_input[0]
        if ch in self._command_map:
            try:
                self._command_map[ch]()
                return True
            except Exception as e:
                logger.error(f"Exception executing command '{ch}': {e}")
                return False
        else:
            logger.info(self._menu)
            return True


# read digital input and engage/disengage teleop accordingly
def read_digital_input_task(teleop: flexivtdk.TransparentCartesianTeleopLAN):
    idx = 0
    while not _stop_event.is_set():
        try:
            # digital_inputs for LAN returns tuple (leader_inputs, follower_inputs)
            di_pair = teleop.digital_inputs(idx)
            leader_di, follower_di = di_pair
            # use leader's first DI port as engage/disengage signal
            if leader_di and len(leader_di) > 0:
                engage_state = bool(leader_di[0])
                teleop.Engage(idx, engage_state)
                logger.info(f"Digital input engage state: {engage_state}")
        except Exception as e:
            logger.error(f"Exception in ReadDigitalInputTask: {e}")
            _stop_event.set()
            return
        time.sleep(0.01)
    logger.info("ReadDigitalInputTask exiting.")


# console task to read user inputs and send commands accordingly
def console_task(teleop: flexivtdk.TransparentCartesianTeleopLAN):
    controller = TeleoperationController(teleop)
    print(controller._menu)
    
    while not _stop_event.is_set():
        try:
            user_input = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            logger.info("Console exiting by user interrupt.")
            _stop_event.set()
            break
        
        if not controller.handle_command(user_input):
            _stop_event.set()
            break

    logger.info("Console thread exiting.")
    return


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Transparent Cartesian Teleop LAN example")
    parser.add_argument("-l", "--leader_sn", required=True, help="serial number of leader robot")
    parser.add_argument("-f", "--follower_sn", required=True, help="serial number of follower robot")
    parser.add_argument("-A", "--lan-ip", action="append", help="lan interface ip whitelist", default=[])
    parser.add_argument("-D", "--enable-digital-input", action="store_true", help="enable digital input reading task")
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None):
    # Program Setup
    # ==============================================================================================
    # Parse arguments
    args = parse_args(argv)
    robot_pairs = [(args.leader_sn, args.follower_sn)]
    try:
        # TDK Initialization
        # ==========================================================================================
        # Instantiate teleop interface
        teleop = flexivtdk.TransparentCartesianTeleopLAN(robot_pairs, args.lan_ip)
        teleop.Init()
        teleop.Start()
        logger.info("Teleop started.")

        # Start console task thread
        console_thr = threading.Thread(target=console_task, args=(teleop,), daemon=True)
        console_thr.start()
        logger.info("Console task started.")

        # Start digital input reading task thread accordingly
        if args.enable_digital_input:
            pedal_thread = threading.Thread(target=read_digital_input_task, args=(teleop,), daemon=True)
            pedal_thread.start()
            logger.info("DI reading task started.")

        console_thr.join()

        # Set the stop event when console thread exits
        _stop_event.set()  
        
        if pedal_thread:
            pedal_thread.join(timeout=1.0)

        # Stop teleop process 
        teleop.Stop()
        logger.info("Teleop stopped.")
        
    except KeyboardInterrupt:
        logger.info("Program interrupted by user")
        _stop_event.set()
    except Exception as e:
        logger.error(f"Exception in main: {e}")
        _stop_event.set()
        sys.exit(1)
    finally:
        # Ensure threads are joined properly
        if console_thr and console_thr.is_alive():
            console_thr.join(timeout=1.0)
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



