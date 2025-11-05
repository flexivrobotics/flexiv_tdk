#!/usr/bin/env python

"""
transparent_cartesian_teleop_wan.py

Example usage of Transparent Cartesian teleoperation under Wide Area Network,
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
kDefaultMaxContactWrench = [25.0, 25.0, 25.0, 10.0, 10.0, 10.0]

# Global thread-safe stop event
_stop_event = threading.Event()

# Logger setup
logger = spdlog.ConsoleLogger("Example")

class WanTeleoperationController:
    """Encapsulates WAN teleoperation functionality for better organization and maintainability."""
    
    def __init__(self, teleop: flexivtdk.TransparentCartesianTeleopWAN):
        self.teleop = teleop
        self.index = 0
        self._command_map = self._create_command_map()
        self._menu = self._create_menu()
    
    def _create_command_map(self) -> Dict[str, Callable]:
        """Create a mapping of keyboard commands to their corresponding methods."""
        return {
            # Teleop engage/disengage
            'r': lambda: self._safe_engage(True),
            'R': lambda: self._safe_engage(False),
            # Null-space postures
            'i': lambda: self._safe_set_nullspace(kPreferredJntPos),
            'I': lambda: self._safe_set_nullspace(kHomeJntPos),
            # Max contact wrench
            'p': lambda: self._safe_set_max_contact_wrench(kDefaultMaxContactWrench),
            # Reinit and start
            'u': self._start_teleop,
            'U': self._stop_teleop,
            # TCP message latency
            'l': self._print_latency,
        }
    
    def _create_menu(self) -> str:
        """Create the command menu string."""
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

  --- Help ---
    Any other key to show this help menu
    """
    
    def _start_teleop(self):
        """Initialize and start teleoperation."""
        try:
            self.teleop.Init()
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
    
    def _print_latency(self):
        """Print current TCP message latency."""
        try:
            ok_latency = self.teleop.CheckTcpConnectionLatency(self.index)
            # binding returns tuple (bool, float) in current version
            if isinstance(ok_latency, tuple):
                ok, latency_ms = ok_latency
                logger.info("TCP connected={} latency_ms={}", ok, latency_ms)
            else:
                # fallback
                logger.info("CheckTcpConnectionLatency returned: {}", ok_latency)
        except Exception as e:
            logger.error("Error checking TCP latency: {}", e)
    
    def _safe_engage(self, engage: bool):
        """Safely engage or disengage teleop with error handling."""
        try:
            self.teleop.Engage(self.index, engage)
            logger.info(f"Teleop {'engaged' if engage else 'disengaged'}")
        except Exception as e:
            logger.error(f"Failed to {'engage' if engage else 'disengage'} teleop: {e}")
    
    def _safe_set_nullspace(self, posture: List[float]):
        """Safely set nullspace posture with error handling."""
        try:
            self.teleop.SetNullSpacePosture(self.index, posture)
            logger.info("Nullspace posture set")
        except Exception as e:
            logger.error(f"Failed to set nullspace posture: {e}")
    
    def _safe_set_max_contact_wrench(self, wrench: List[float]):
        """Safely set max contact wrench with error handling."""
        try:
            self.teleop.SetMaxContactWrench(self.index, wrench)
            logger.info("Max contact wrench set")
        except Exception as e:
            logger.error(f"Failed to set max contact wrench: {e}")
    
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
                logger.error("Exception executing command '{}': {}", ch, e)
                return False
        else:
            print(self._menu)
            return True


# read digital input and engage/disengage teleop accordingly
def read_digital_input_task(teleop: flexivtdk.TransparentCartesianTeleopWAN):
    idx = 0
    while not _stop_event.is_set():
        try:
            # Digital input for WAN returns a list of inputs
            di_state = teleop.digital_inputs(idx)
            # Use first DI port as engage/disengage signal
            if di_state and len(di_state) > 0:
                engage_state = bool(di_state[0])
                teleop.Engage(idx, engage_state)
                logger.info(f"Digital input engage state: {engage_state}")
        except Exception as e:
            logger.error(f"Exception in ReadDigitalInputTask: {e}")
            _stop_event.set()
            return
        time.sleep(0.01)
    logger.info("ReadDigitalInputTask exiting.")


# console task to read user inputs and send commands accordingly
def console_task(teleop: flexivtdk.TransparentCartesianTeleopWAN):
    controller = WanTeleoperationController(teleop)
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
    parser = argparse.ArgumentParser(description="Transparent Cartesian Teleop WAN example (Python)")
    parser.add_argument("-l", "--leader_sn", required=True, help="serial number of leader robot")
    parser.add_argument("-f", "--follower_sn", required=True, help="serial number of follower robot")
    parser.add_argument("-r", "--role", required=True, choices=["leader", "follower"], help="role in teleop")
    parser.add_argument("-t", "--tcp-role", required=True, choices=["server", "client"], help="tcp role")
    parser.add_argument("-i", "--public-ip", required=True, help="public IPv4 address of TCP server")
    parser.add_argument("-p", "--port", required=True, type=int, help="listening port of TCP server")
    parser.add_argument("-A", "--lan-ip", action="append", help="lan interface ip whitelist", default=[])
    parser.add_argument("-W", "--wan-ip", action="append", help="wan interface ip whitelist", default=[])
    parser.add_argument("-D", "--enable-digital-input", action="store_true", help="enable digital input reading task")
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None):
    # Program Setup
    # ==============================================================================================
    # Parse arguments
    args = parse_args(argv)
    
    # Validate arguments
    if args.role not in ['leader', 'follower']:
        logger.error("Valid inputs for --role are: leader, follower")
        sys.exit(1)
    
    if args.tcp_role not in ['server', 'client']:
        logger.error("Valid inputs for --tcp-role are: server, client")
        sys.exit(1)
    
    # Determine role
    if args.role == 'follower':
        role = flexivtdk.Role.WAN_TELEOP_FOLLOWER
    else:  # leader
        role = flexivtdk.Role.WAN_TELEOP_LEADER
    
    # Network configuration
    network_cfg = flexivtdk.NetworkCfg()
    network_cfg.is_tcp_server = (args.tcp_role == 'server')
    network_cfg.public_ipv4_address = args.public_ip
    network_cfg.listening_port = args.port
    
    # Set interface whitelists if provided
    if args.lan_ip:
        network_cfg.lan_interface_whitelist = args.lan_ip
    if args.wan_ip:
        network_cfg.wan_interface_whitelist = args.wan_ip
    
    # Robot pairs
    robot_pairs = [(args.leader_sn, args.follower_sn)]
    
    teleop = None
    console_thr = None
    pedal_thread = None
    
    try:
        # TDK Initialization
        # ==========================================================================================
        # Instantiate teleop interface
        teleop = flexivtdk.TransparentCartesianTeleopWAN(robot_pairs, role, network_cfg)

        # Initialize teleop, this will Clear fault, Calibrate the force sensors, initialize teleop control parameters, etc. 
        teleop.Init()

        # Start teleop process
        teleop.Start()
        
        # Set max contact wrench
        teleop.SetMaxContactWrench(0, kDefaultMaxContactWrench)
        
        logger.info("WAN Teleop started.")

        # Start console task thread
        console_thr = threading.Thread(target=console_task, args=(teleop,), daemon=True)
        console_thr.start()
        logger.info("Console task started.")

        # Start digital input reading task thread accordingly
        # Only start if role is leader and enable_di flag is provided
        if args.role == 'leader' and args.enable_digital_input:
            logger.info("Starting ReadDigitalInputTask thread as role is 'leader' and requested by --enable-digital-input flag.")
            pedal_thread = threading.Thread(target=read_digital_input_task, args=(teleop,), daemon=True)
            pedal_thread.start()
        else:
            logger.info("ReadDigitalInputTask thread NOT started (role is not 'leader' or --enable-digital-input flag not provided).")

        console_thr.join()

        # Set the stop event when console thread exits
        _stop_event.set()  
        
        if pedal_thread:
            pedal_thread.join(timeout=1.0)

        # Stop teleop process 
        if teleop:
            teleop.Stop()
        logger.info("WAN Teleop stopped.")
        
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



