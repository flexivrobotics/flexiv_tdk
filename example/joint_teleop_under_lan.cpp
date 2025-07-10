/**
 * @example joint_teleop_under_lan.cpp
 * @brief joint-space robot-robot teleoperation under LAN (Local Area Network) connection.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/tdk/joint_teleop_lan.hpp>

#include <spdlog/spdlog.h>

#include <getopt.h>
#include <iostream>
#include <thread>

namespace {
const std::vector<double> kJointStiffnessRatio = {0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02};
constexpr double kLastJointShapedInertia = 0.05;
}

void PrintHelp()
{
    // clang-format off
    std::cout << "Usage: sudo ./joint_teleop_under_lan [-1 serial_num] [-2 serial_num]" << std::endl;
    std::cout << "  -1  --first-sn    Serial number of the first robot." << std::endl;
    std::cout << "  -2  --second-sn   Serial number of the second robot." << std::endl;
    // clang-format on
}

const struct option kLongOptions[] = {
    // clang-format off
    {"first-sn",    required_argument, 0, '1'},
    {"second-sn",   required_argument, 0, '2'},
    {0,                             0, 0,  0}
    // clang-format on
};

int main(int argc, char* argv[])
{
    // Parse program arguments
    std::string first_sn, second_sn;
    int opt = 0;
    while ((opt = getopt_long(argc, argv, "1:2:", kLongOptions, nullptr)) != -1) {
        switch (opt) {
            case '1':
                first_sn = std::string(optarg);
                break;
            case '2':
                second_sn = std::string(optarg);
                break;
            default:
                PrintHelp();
                return 1;
        }
    }
    if (first_sn.empty() || second_sn.empty()) {
        PrintHelp();
        return 1;
    }

    try {
        // Create teleop control interface
        flexiv::tdk::JointTeleopLAN joint_teleop({{first_sn, second_sn}});

        // Only control 1 pair of robots
        unsigned int robot_pair_idx = 0;

        // Run initialization sequence
        joint_teleop.Init();

        // Set 20 degrees soft limit
        joint_teleop.SetSoftLimit(robot_pair_idx, 20.0);

        // Sync pose, first robot stays still, second robot moves to its pose
        joint_teleop.SyncPose(robot_pair_idx, {});

        // Enable inertia shaping for the last joint
        std::vector<std::pair<bool, double>> shaped_joint_inertia;
        shaped_joint_inertia.resize(joint_teleop.DoF(robot_pair_idx), {false, 1.0});
        shaped_joint_inertia.back() = {true, kLastJointShapedInertia};
        joint_teleop.SetInertiaShaping(robot_pair_idx, shaped_joint_inertia);

        // Start control loop
        joint_teleop.Start();

        // Set impedance properties
        joint_teleop.SetJointImpedance(robot_pair_idx, kJointStiffnessRatio);

        // Block until faulted
        bool last_pedal_input = false;
        while (!joint_teleop.any_fault()) {
            // Activate by pedal
            bool pedal_input = joint_teleop.digital_inputs(robot_pair_idx).first[0];
            if (pedal_input != last_pedal_input) {
                joint_teleop.Activate(robot_pair_idx, pedal_input);
                last_pedal_input = pedal_input;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Fault occurred, stop the teleoperation
        joint_teleop.Stop();

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}
