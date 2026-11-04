/**
 * @example device_teleop_lan.cpp
 * @brief Example only: hold one or more robots at the current TCP pose.
 *
 * DeviceTeleopLan is the interface for device-to-robot teleoperation, for example VR
 * controllers or haptic devices driving one or more robots. This program does not
 * talk to any such device. It only shows the call order: Init(), set a contact-wrench limit,
 * then stream commands with Start(). A real application must read its own device driver and,
 * together with the robot state, write a smooth MotionControlCmds sequence. Do not copy a
 * target pose in from an unrelated frame or with a large jump.
 *
 * The main thread only writes MotionControlCmds. Command objects stay alive until after Stop().
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/tdk/data.hpp>
#include <flexiv/tdk/device_teleop_lan.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <getopt.h>

namespace {
std::atomic<bool> g_stop {false};

/** Soft contact limit used by this example, not a device-specific value. [N]:[Nm]. */
const std::array<double, flexiv::tdk::kCartDoF> kExampleMaxContactWrench
    = {25.0, 25.0, 25.0, 10.0, 10.0, 10.0};

void OnSignal(int)
{
    g_stop.store(true);
}

void LogWarn(const std::string& msg)
{
    std::cerr << "[warn] " << msg << std::endl;
}

void LogError(const std::string& msg)
{
    std::cerr << "[error] " << msg << std::endl;
}

void LogInfo(const std::string& msg)
{
    std::cout << "[info] " << msg << std::endl;
}

void PrintHelp()
{
    // clang-format off
    std::cout << "Usage: sudo ./device_teleop_lan -r <serial_num> [-r <serial_num> ...] [-A <lan_ipv4>] [-t <seconds>] [-n]" << std::endl;
    std::cout << "  -r  --robot-sn    Serial number of a robot to hold. Repeat for each additional robot." << std::endl;
    std::cout << "  -A  --lan-ip      [optional] LAN IPv4 address of the NIC connected to the robots. Repeatable." << std::endl;
    std::cout << "  -t  --seconds     [optional] How long to hold the pose. Default: 30." << std::endl;
    std::cout << "  -n  --no-zero-ft  [optional] Skip force/torque sensor zeroing during Init()." << std::endl;
    std::cout << "The robots do not move. Ctrl+C stops the control loop and holds the pose." << std::endl;
    // clang-format on
}

const struct option kLongOptions[] = {
    // clang-format off
    {"robot-sn",   required_argument, 0, 'r'},
    {"lan-ip",     required_argument, 0, 'A'},
    {"seconds",    required_argument, 0, 't'},
    {"no-zero-ft", no_argument,       0, 'n'},
    {0,                            0, 0,  0}
    // clang-format on
};
}

int main(int argc, char* argv[])
{
    std::vector<std::string> robot_sn_vec;
    std::vector<std::string> network_interface_whitelist;
    int hold_seconds = 30;
    bool zero_ft = true;

    int opt = 0;
    while ((opt = getopt_long(argc, argv, "r:A:t:n", kLongOptions, nullptr)) != -1) {
        switch (opt) {
            case 'r':
                robot_sn_vec.emplace_back(optarg);
                break;
            case 'A':
                network_interface_whitelist.emplace_back(optarg);
                break;
            case 't':
                try {
                    hold_seconds = std::stoi(optarg);
                } catch (const std::exception&) {
                    PrintHelp();
                    return 1;
                }
                break;
            case 'n':
                zero_ft = false;
                break;
            default:
                PrintHelp();
                return 1;
        }
    }
    if (robot_sn_vec.empty() || hold_seconds <= 0) {
        PrintHelp();
        return 1;
    }
    if (network_interface_whitelist.empty()) {
        LogWarn("network_interface_whitelist is not provided, will search all network interfaces.");
    }

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    try {
        flexiv::tdk::DeviceTeleopLan teleop(robot_sn_vec, network_interface_whitelist);
        try {
            teleop.Init(
                zero_ft ? flexiv::tdk::ZeroFTSensor::Enable : flexiv::tdk::ZeroFTSensor::Disable);

            // Init() has already switched the control mode, so the contact limit can be set
            // before Start(). A real device should choose this from its own safety requirements.
            for (unsigned int i = 0; i < robot_sn_vec.size(); ++i) {
                teleop.SetRobotMaxContactWrench(i, kExampleMaxContactWrench);
            }

            // One command object per robot, in the same order as robot_sn_vec. These objects must
            // stay alive for the whole session; the library reads them from its realtime threads.
            std::vector<std::shared_ptr<flexiv::tdk::MotionControlCmds>> cmds;
            std::vector<std::array<double, flexiv::tdk::kPoseSize>> hold_poses;
            const std::array<double, flexiv::tdk::kCartDoF> zero_twist {};
            cmds.reserve(robot_sn_vec.size());
            hold_poses.reserve(robot_sn_vec.size());
            for (unsigned int i = 0; i < robot_sn_vec.size(); ++i) {
                auto pose = teleop.instance(i)->states().tcp_pose;
                auto cmd = std::make_shared<flexiv::tdk::MotionControlCmds>();
                cmd->write(pose, zero_twist, zero_twist);
                cmds.push_back(cmd);
                hold_poses.push_back(pose);
                LogInfo("Holding robot [" + robot_sn_vec[i] + "] at its current TCP pose");
            }

            teleop.Start(cmds);
            LogInfo("Control loop started. Updating the same command objects from this thread.");

            const auto deadline
                = std::chrono::steady_clock::now() + std::chrono::seconds(hold_seconds);
            auto next_status = std::chrono::steady_clock::now();
            while (!g_stop.load() && std::chrono::steady_clock::now() < deadline) {
                for (unsigned int i = 0; i < cmds.size(); ++i) {
                    cmds[i]->write(hold_poses[i], zero_twist, zero_twist);
                }
                if (std::chrono::steady_clock::now() >= next_status) {
                    for (unsigned int i = 0; i < robot_sn_vec.size(); ++i) {
                        if (teleop.fault(i)) {
                            LogWarn("Robot [" + robot_sn_vec[i]
                                    + "] is in fault. Other robots keep holding.");
                        }
                    }
                    next_status = std::chrono::steady_clock::now() + std::chrono::seconds(1);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // Stop once, then let teleop leave scope. A second Stop() from the destructor is safe.
            LogInfo("Stopping teleoperation.");
            teleop.Stop();
        } catch (...) {
            try {
                teleop.Stop();
            } catch (const std::exception& stop_error) {
                LogError(stop_error.what());
            }
            throw;
        }
    } catch (const std::exception& e) {
        LogError(e.what());
        return 1;
    }

    LogInfo("Teleoperation exiting.");
    return 0;
}
