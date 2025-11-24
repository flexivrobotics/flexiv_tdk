#!/usr/bin/env python3
"""
joint_teleop_over_wan.py

Example usage of joint-space robot-robot teleoperation over WAN (Wide Area Network) connection.
Monitors pedal input from robot connected to the server machine to activate/deactivate teleoperation.
"""

import sys
import argparse
import time
import spdlog
# pip install flexivtdk
import flexivtdk

# Constants
JOINT_STIFFNESS_RATIO = [0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02]
LAST_JOINT_SHAPED_INERTIA = 0.05

# Logger setup
logger = spdlog.ConsoleLogger("Example")

def print_help():
    """Print usage information"""
    help_text = """Usage: python joint_teleop_over_wan.py [-s serial_num] [-r server/client] [-i ip] [-p port]
  -s, --serial-number    <required> Serial number of the local robot.
  -r, --tcp-role         <required> Role in the TCP connection, [server] or [client].
  -i, --ip               <required> Public IPv4 address of the TCP server machine.
  -p, --port             <required> Listening port of the TCP server machine.
  -l, --lan-whitelist-ip <optional> The ip address of the network card connected to the robot control box.
  -w, --wan-whitelist-ip <optional> The ip address of the network card connected to the Internet."""
    print(help_text)

def main():
    parser = argparse.ArgumentParser(
        description="Joint Teleoperation over WAN",
        add_help=False
    )
    
    parser.add_argument('-s', '--serial-number', required=True, 
                       help='Serial number of the local robot')
    parser.add_argument('-r', '--tcp-role', required=True,
                       choices=['server', 'client'],
                       help='Role in the TCP connection: server or client')
    parser.add_argument('-i', '--ip', required=True,
                       help='Public IPv4 address of the TCP server machine')
    parser.add_argument('-p', '--port', type=int, required=True,
                       help='Listening port of the TCP server machine')
    parser.add_argument('-l', '--lan-whitelist-ip',
                       help='IP address of the network card connected to the robot')
    parser.add_argument('-w', '--wan-whitelist-ip',
                       help='IP address of the network card connected to the Internet')
    
    # If no arguments provided, show help
    if len(sys.argv) == 1:
        print_help()
        sys.exit(1)
    
    try:
        args = parser.parse_args()
    except SystemExit:
        print_help()
        sys.exit(1)
    
    # Validate required arguments
    if not args.serial_number or not args.tcp_role or not args.ip or not args.port:
        print_help()
        sys.exit(1)
    
    # Setup network configuration
    lan_interface_whitelist = []
    wan_interface_whitelist = []
    
    if args.lan_whitelist_ip:
        lan_interface_whitelist.append(args.lan_whitelist_ip)
    if args.wan_whitelist_ip:
        wan_interface_whitelist.append(args.wan_whitelist_ip)
    
    if not lan_interface_whitelist or not wan_interface_whitelist:
        logger.warn("LAN or WAN whitelist is not provided, will search all network interfaces.")
    
    # Determine TCP role
    is_tcp_server = args.tcp_role == 'server'
    if args.tcp_role not in ['server', 'client']:
        logger.error("Valid inputs for --tcp-role are: server, client")
        sys.exit(1)
    
    # Network configuration
    network_cfg = flexivtdk.NetworkCfg()
    network_cfg.is_tcp_server = is_tcp_server
    network_cfg.public_ipv4_address = args.ip
    network_cfg.listening_port = args.port
    network_cfg.lan_interface_whitelist = lan_interface_whitelist
    network_cfg.wan_interface_whitelist = wan_interface_whitelist
    
    logger.info(f"Robot serial number: {args.serial_number}")
    logger.info(f"Start as {args.tcp_role} on {args.ip}:{args.port}")
    
    try:
        # Create teleop control interface instance
        joint_teleop = flexivtdk.JointTeleopWAN(args.serial_number, network_cfg)
        
        # Run initialization sequence
        joint_teleop.Init()
        
        if is_tcp_server:
            # Set 20 degrees soft limit for only one side of teleoperation
            joint_teleop.SetSoftLimit(20.0)
            
            # Server stays at current pose
            joint_teleop.SyncPose(False, [])
        else:
            # Client syncs pose with server
            joint_teleop.SyncPose(True)
        
        # Enable inertia shaping for the last joint
        shaped_joint_inertia = []
        dof = joint_teleop.DoF()
        for i in range(dof):
            if i == dof - 1:  # Last joint
                shaped_joint_inertia.append((True, LAST_JOINT_SHAPED_INERTIA))
            else:
                shaped_joint_inertia.append((False, 1.0))
        
        joint_teleop.SetInertiaShaping(shaped_joint_inertia)
        
        # Start control loop
        joint_teleop.Start()
        
        # Set joint impedance
        joint_teleop.SetJointImpedance(JOINT_STIFFNESS_RATIO)
        
        # Block until faulted
        last_pedal_input = False
        while not joint_teleop.fault():
            # Server is activated by pedal, client will auto sync the activation
            if is_tcp_server:
                pedal_input = joint_teleop.digital_inputs()[0]
                if pedal_input != last_pedal_input:
                    joint_teleop.Activate(pedal_input)
                    last_pedal_input = pedal_input
            # 100ms sleep
            time.sleep(0.1)  
        
        # Fault occurred, stop the teleoperation
        joint_teleop.Stop()
        
    except Exception as e:
        logger.error(str(e))
        sys.exit(1)

if __name__ == "__main__":
    main()



