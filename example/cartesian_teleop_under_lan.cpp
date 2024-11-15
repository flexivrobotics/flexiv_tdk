/**
 * @example cartesian_teleop_under_lan.cpp
 * Run Cartesian-space robot-robot teleoperation under LAN (Local Area Network) connection.
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/tdk/cartesian_teleop_lan.hpp>

#include <spdlog/spdlog.h>
#include <getopt.h>
#include <iostream>
#include <thread>

namespace {
const struct option kLongOptions[] = {{"first-sn", required_argument, 0, '1'},
    {"second-sn", required_argument, 0, '2'}, {0, 0, 0, 0}};
}

void PrintHelp()
{
    // clang-format off
    std::cout << "Usage: ./cartesian_teleop_under_lan [-1 serial_num] [-2 serial_num]" << std::endl;
    std::cout << "  -1  --first-sn    Serial number of the first robot." << std::endl;
    std::cout << "  -2  --second-sn   Serial number of the second robot." << std::endl;
    // clang-format on
}

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
        flexiv::tdk::CartesianTeleopLAN cart_teleop({{first_sn, second_sn}});

        // Run initialization sequence
        cart_teleop.Init();

        // Sync pose, first robot stays still, second robot moves to its pose
        cart_teleop.SyncPose(0, {});

        // Start control loop
        cart_teleop.Start();

        // Block until faulted
        bool last_pedal_input = false;
        while (!cart_teleop.any_fault()) {
            // Activate by pedal
            bool pedal_input = cart_teleop.digital_inputs(0).first[0];
            if (pedal_input != last_pedal_input) {
                cart_teleop.Activate(0, pedal_input);
                last_pedal_input = pedal_input;
            }
            // Sync null-space posture of the second robot to that of the first
            auto first_robot_q = cart_teleop.joint_positions(0).first;
            cart_teleop.SetNullSpacePostures(0, std::pair {first_robot_q, first_robot_q});
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Fault occurred, stop the teleoperation
        cart_teleop.Stop();

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}
