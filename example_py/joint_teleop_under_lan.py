
#!/usr/bin/env python

"""
joint_teleop_under_lan.py

Example usage of joint-space robot-robot teleoperation under LAN (Local Area Network) connection.
Monitors pedal input from the first robot to activate/deactivate teleoperation.

"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import spdlog
import argparse
import time

# pip install flexivtdk
import flexivtdk

logger=spdlog.ConsoleLogger("Example")

def print_help():
    """Print usage information"""
    print("Usage: python joint_teleop_under_lan.py --first-sn <robot_sn> --second-sn <robot_sn>")
    print("  --first-sn    Serial number of the first robot.")
    print("  --second-sn   Serial number of the second robot.")


def main():
    parser = argparse.ArgumentParser(description="Joint Teleoperation over LAN")
    parser.add_argument("--first-sn", required=True, help="Serial number of the first robot")
    parser.add_argument("--second-sn", required=True, help="Serial number of the second robot")
    
    args = parser.parse_args()
    
    if not args.first_sn or not args.second_sn:
        print_help()
        return 1

    robot_pairs= [(args.first_sn, args.second_sn)]
    try:
        # Create teleop control interface
        joint_teleop = flexivtdk.JointTeleopLAN(robot_pairs)

        # Only control 1 pair of robots
        robot_pair_idx = 0

        # Joint stiffness ratios for impedance control
        joint_stiffness_ratio = [0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02]
        
        # Last joint shaped inertia value
        last_joint_shaped_inertia = 0.05

        # Run initialization sequence
        logger.info("Initializing teleoperation...")
        joint_teleop.Init()

        # Set 20 degrees soft limit
        logger.info("Setting soft limit...")
        joint_teleop.SetSoftLimit(robot_pair_idx, 20.0)

        # Sync pose, first robot stays still, second robot moves to its pose
        logger.info("Synchronizing poses...")
        joint_teleop.SyncPose(robot_pair_idx, [])

        # Enable inertia shaping for the last joint
        logger.info("Setting up inertia shaping...")
        dof = joint_teleop.DoF(robot_pair_idx)
        shaped_joint_inertia = []
        for i in range(dof):
            if i == dof - 1:  # Last joint
                shaped_joint_inertia.append((True, last_joint_shaped_inertia))
            else:
                shaped_joint_inertia.append((False, 1.0))
        
        joint_teleop.SetInertiaShaping(robot_pair_idx, shaped_joint_inertia)

        # Start control loop
        joint_teleop.Start()

        # Set impedance properties
        joint_teleop.SetJointImpedance(robot_pair_idx, joint_stiffness_ratio)

        # Store last pedal input state
        last_pedal_input = False
        
        logger.info("Teleoperation started. Press pedal to activate/deactivate.")

        # Monitor loop
        while not joint_teleop.any_fault():
            # Activate by pedal
            digital_inputs = joint_teleop.digital_inputs(robot_pair_idx)
            # First digital input of the first robot
            pedal_input = digital_inputs[0][0]  
            
            if pedal_input != last_pedal_input:
                logger.info(f"Pedal state changed: {pedal_input}")
                joint_teleop.Activate(robot_pair_idx, pedal_input)
                last_pedal_input = pedal_input
            
            # Sleep to prevent excessive CPU usage
            time.sleep(0.1)

        logger.info("Fault detected, stopping teleoperation...")
        # Fault occurred, stop the teleoperation
        joint_teleop.Stop()

    except Exception as e:
        logger.info(f"Error: {e}")
        return 1

    logger.info("Teleoperation exiting.")
    return 0


if __name__ == "__main__":
    main()