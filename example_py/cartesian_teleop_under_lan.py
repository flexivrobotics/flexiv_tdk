#!/usr/bin/env python3
"""
Example usage of Cartesian-space robot-robot teleoperation under LAN(Local Area Network) connection.
Monitors digital input from first robot to activate/deactivate teleoperation.
"""

import sys
import argparse
import time
import spdlog
# pip install flexivtdk
import flexiv_tdk


# Constants
SHAPED_CART_INERTIA = [60.0, 60.0, 60.0, 20.0, 20.0, 20.0]
CART_STIFFNESS_RATIO = [0.1, 0.1, 0.1, 0.005, 0.005, 0.005]
CART_DAMPING_RATIO = [0.6, 0.6, 0.6, 0.6, 0.6, 0.6]

def print_help():
    """Print usage information"""
    help_text = """Usage: python cartesian_teleop_under_lan.py [-1 serial_num] [-2 serial_num]
  -1, --first-sn    Serial number of the first robot.
  -2, --second-sn   Serial number of the second robot."""
    print(help_text)

def main():
    parser = argparse.ArgumentParser(
        description="Cartesian-space robot-robot teleoperation under LAN",
        add_help=False
    )
    
    parser.add_argument('-1', '--first-sn', required=True,
                       help='Serial number of the first robot')
    parser.add_argument('-2', '--second-sn', required=True,
                       help='Serial number of the second robot')
    
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
    if not args.first_sn or not args.second_sn:
        print_help()
        sys.exit(1)
    
    try:
        # Create teleop control interface
        cart_teleop = flexiv.tdk.CartesianTeleopLAN([args.first_sn, args.second_sn])

        # Only control 1 pair of robots
        robot_pair_idx = 0

        # Run initialization sequence
        cart_teleop.Init()

        # Sync pose, first robot stays still, second robot moves to its tcp pose
        first_robot_state = cart_teleop.robot_states(robot_pair_idx).first
        cart_teleop.SyncPose(robot_pair_idx, first_robot_state.tcp_pose)

        # Enable inertia shaping for all Cartesian axes
        shaped_cart_inertia = []
        for i in range(flexiv.tdk.kCartDoF): 
            shaped_cart_inertia.append((True, SHAPED_CART_INERTIA[i]))
        
        cart_teleop.SetInertiaShaping(robot_pair_idx, shaped_cart_inertia)

        # Start control loop
        cart_teleop.Start()

        # Set impedance properties
        cart_teleop.SetCartesianImpedance(
            robot_pair_idx, 
            CART_STIFFNESS_RATIO, 
            CART_DAMPING_RATIO
        )

        # Block until faulted
        last_pedal_input = False
        while not cart_teleop.any_fault():
            # Activate by pedal
            digital_inputs = cart_teleop.digital_inputs(robot_pair_idx).first
            # First digital input of the first robot is used as pedal input
            pedal_input = digital_inputs[0]
            if pedal_input != last_pedal_input:
                cart_teleop.Activate(robot_pair_idx, pedal_input)
                last_pedal_input = pedal_input
            
            # Sync null-space posture of the second robot to that of the first
            first_robot_q = cart_teleop.robot_states(robot_pair_idx).first.q
            # Create a pair with the same joint positions for both robots
            null_space_postures = (first_robot_q, first_robot_q)
            cart_teleop.SetNullSpacePostures(robot_pair_idx, null_space_postures)
            
            time.sleep(0.1)

        # Fault occurred, stop the teleoperation
        cart_teleop.Stop()

    except Exception as e:
        spdlog.error(str(e))
        sys.exit(1)

if __name__ == "__main__":
    main()