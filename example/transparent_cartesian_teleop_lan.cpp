/**
 * @example transparent_cartesian_teleop_lan.cpp
 * @brief Example usage of Transparent Cartesian teleoperation under Local Area Network for
 * controlling a follower robot using a leader robot with transparent force feedback. Supports both
 * keyboard and digital input engage/disengage signal reading, with various axes lock modes, force
 * scaling, max contact wrench setting, and teleop status query, etc.
 * @note This program is provided only as an example. Users must adapt it to their own application
 * requirements, safety procedures, and software architecture before deployment.
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/tdk/data.hpp>
#include <flexiv/tdk/transparent_cartesian_teleop_lan.hpp>

#include <getopt.h>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace {
/** Nullspace to a preferred posture */
const std::vector<double> kPreferredJntPos
    = {60 * M_PI / 180.0, -60 * M_PI / 180.0, -85 * M_PI / 180.0, 115 * M_PI / 180.0,
        70 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0};

/** Nullspace to Home posture */
const std::vector<double> kHomeJntPos = {0 * M_PI / 180.0, -40 * M_PI / 180.0, 0 * M_PI / 180.0,
    90 * M_PI / 180.0, 0 * M_PI / 180.0, 40 * M_PI / 180.0, 0 * M_PI / 180.0};

/** Maximum contact wrench for soft contact */
const std::array<double, flexiv::tdk::kCartDoF> kDefaultMaxContactWrench
    = {5.0, 5.0, 5.0, 40.0, 40.0, 40.0};

/** Robot pair index used by this example */
constexpr unsigned int kIdx = 0;

/** Single-arm joint group controlled by this example */
constexpr auto kJointGroup = flexiv::rdk::JointGroup::ARM_1;

/** Atomic signal to stop console and DI reading tasks */
std::atomic<bool> g_running {true};

void LogInfo(const std::string& msg)
{
    std::cout << "[info] " << msg << std::endl;
}

void LogWarn(const std::string& msg)
{
    std::cerr << "[warn] " << msg << std::endl;
}

void LogError(const std::string& msg)
{
    std::cerr << "[error] " << msg << std::endl;
}

void PrintTeleopStatus(const flexiv::tdk::TeleopStatus& status)
{
    LogInfo("Teleop status: initialized=" + std::to_string(status.initialized)
            + " started=" + std::to_string(status.started)
            + " engaged=" + std::to_string(status.engaged)
            + " stopped=" + std::to_string(status.stopped)
            + " fault=" + std::to_string(status.fault)
            + " motion_restricted=" + std::to_string(status.motion_restricted));
    if (status.primary.code == flexiv::tdk::TeleopIssueCode::NONE) {
        LogInfo("No teleop restriction. Safe to continue.");
        return;
    }
    const auto& issue = status.primary;
    LogWarn(std::string("[")
            + flexiv::tdk::TeleopIssueLevelStr[static_cast<size_t>(issue.level)] + "] "
            + issue.title + " | " + issue.description + " | " + issue.suggestion);
    for (const auto& extra : status.issues) {
        if (extra.code == issue.code && extra.side == issue.side
            && extra.joint_index == issue.joint_index) {
            continue;
        }
        LogWarn(std::string("  also: [")
                + flexiv::tdk::TeleopIssueCodeStr[static_cast<size_t>(extra.code)] + "] "
                + extra.title);
    }
}

} // namespace

void PrintHelp()
{
    // clang-format off
    std::cout<<"Invalid program arguments!"<<std::endl;
    std::cout<<"     -l     [necessary] serial number of leader robot."<<std::endl;
    std::cout<<"     -f     [necessary] serial number of follower robot."<<std::endl;
    std::cout<<"     -D     [optional] Enable Digital Input reading task." << std::endl;
    std::cout<<"Usage: sudo ./transparent_cartesian_teleop_lan [-l leader_robot_sn] [-f follower_robot_sn] [-D]"<<std::endl;
    // clang-format on
}

const struct option kLongOptions[] = {
    // clang-format off
    {"leader SN",                                       required_argument,  0, 'l'},
    {"follower SN",                                     required_argument,  0, 'f'},
    {"enable digital input",                            no_argument,        0, 'D'},
    {0,                                                                 0,  0,  0 }
    // clang-format on
};

/**
 * @brief Task for monitoring DI signals and engaging teleop.
 */
void ReadDigitalInputTask(flexiv::tdk::TransparentCartesianTeleopLAN& teleop)
{
    while (g_running.load() && !teleop.any_fault()) {
        try {
            teleop.Engage(kIdx, kJointGroup, teleop.digital_inputs(kIdx).first[0]);
        } catch (const std::exception& e) {
            LogError(std::string("Exception in ReadDigitalInputTask: ") + e.what());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    LogInfo("ReadDigitalInputTask exiting.");
    return;
}
/**
 * @brief Task for calling TransparentCartesianTeleopLAN functions from console.
 */
void ConsoleTask(flexiv::tdk::TransparentCartesianTeleopLAN& teleop)
{
    auto PrintCommandMenu = []() {
        std::cout << R"(
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

  --- Repulsive Force ---
    a        : Set repulsive force to {5, 0, 0}
    A        : Set repulsive force to {-5, 0, 0}
    C        : Clear force 

  --- Start/Stop ---
    b        : Stop teleop
    B        : Start teleop


  --- Is teleop stopped or not ---
    s        : Query if teleop stopped or not

  --- Teleop status ---
    h        : Print why teleop is restricted / paused and what to do next

  --- Help ---
    Any other key to show this help menu
        )" << std::endl;
    };

    flexiv::tdk::AxisLock cmd;
    teleop.GetAxisLockState(kIdx, kJointGroup, cmd);

    while (g_running.load() && !teleop.any_fault()) {

        std::string userInput {};

        // Get user input
        std::getline(std::cin, userInput);

        if (userInput.empty()) {
            LogWarn("Empty command!");
            PrintCommandMenu();
            continue;
        }

        try {
            switch (userInput[0]) {
                case 'x':
                    cmd.lock_trans_axis[0] = !cmd.lock_trans_axis[0];
                    cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                    teleop.SetAxisLockCmd(kIdx, kJointGroup, cmd);
                    break;
                case 'y':
                    cmd.lock_trans_axis[1] = !cmd.lock_trans_axis[1];
                    cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                    teleop.SetAxisLockCmd(kIdx, kJointGroup, cmd);
                    break;
                case 'z':
                    cmd.lock_trans_axis[2] = !cmd.lock_trans_axis[2];
                    cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                    teleop.SetAxisLockCmd(kIdx, kJointGroup, cmd);
                    break;
                case 'q':
                    cmd.lock_ori_axis[0] = !cmd.lock_ori_axis[0];
                    cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                    teleop.SetAxisLockCmd(kIdx, kJointGroup, cmd);
                    break;
                case 'w':
                    cmd.lock_ori_axis[1] = !cmd.lock_ori_axis[1];
                    cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                    teleop.SetAxisLockCmd(kIdx, kJointGroup, cmd);
                    break;
                case 'e':
                    cmd.lock_ori_axis[2] = !cmd.lock_ori_axis[2];
                    cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                    teleop.SetAxisLockCmd(kIdx, kJointGroup, cmd);
                    break;

                case 'X':
                    cmd.lock_trans_axis[0] = !cmd.lock_trans_axis[0];
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    teleop.SetAxisLockCmd(kIdx, kJointGroup, cmd);
                    break;
                case 'Y':
                    cmd.lock_trans_axis[1] = !cmd.lock_trans_axis[1];
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    teleop.SetAxisLockCmd(kIdx, kJointGroup, cmd);
                    break;
                case 'Z':
                    cmd.lock_trans_axis[2] = !cmd.lock_trans_axis[2];
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    teleop.SetAxisLockCmd(kIdx, kJointGroup, cmd);
                    break;
                case 'Q':
                    cmd.lock_ori_axis[0] = !cmd.lock_ori_axis[0];
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    teleop.SetAxisLockCmd(kIdx, kJointGroup, cmd);
                    break;
                case 'W':
                    cmd.lock_ori_axis[1] = !cmd.lock_ori_axis[1];
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    teleop.SetAxisLockCmd(kIdx, kJointGroup, cmd);
                    break;
                case 'E':
                    cmd.lock_ori_axis[2] = !cmd.lock_ori_axis[2];
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    teleop.SetAxisLockCmd(kIdx, kJointGroup, cmd);
                    break;

                case 'r':
                    teleop.Engage(kIdx, kJointGroup, true);
                    break;
                case 'R':
                    teleop.Engage(kIdx, kJointGroup, false);
                    break;

                case 't':
                    teleop.SetWrenchFeedbackScalingFactor(kIdx, kJointGroup, 0.5);
                    break;
                case 'T':
                    teleop.SetWrenchFeedbackScalingFactor(kIdx, kJointGroup, 2);
                    break;

                case 'u':
                    cmd.lock_ori_axis = {false, false, false};
                    cmd.lock_trans_axis = {false, false, false};
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    teleop.SetAxisLockCmd(kIdx, kJointGroup, cmd);
                    break;
                case 'U':
                    cmd.lock_ori_axis = {true, true, true};
                    cmd.lock_trans_axis = {true, true, true};
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    teleop.SetAxisLockCmd(kIdx, kJointGroup, cmd);
                    break;
                case 'i':
                    teleop.SetLeaderNullSpacePosture(kIdx, kJointGroup, kPreferredJntPos);
                    break;
                case 'I':
                    teleop.SetLeaderNullSpacePosture(kIdx, kJointGroup, kHomeJntPos);
                    break;
                case 'o':
                    teleop.SetFollowerNullSpacePosture(kIdx, kJointGroup, kPreferredJntPos);
                    break;
                case 'O':
                    teleop.SetFollowerNullSpacePosture(kIdx, kJointGroup, kHomeJntPos);
                    break;
                case 'p':
                    teleop.SetFollowerMaxContactWrench(kIdx, kJointGroup, kDefaultMaxContactWrench);
                    break;

                case 'a':
                    teleop.SetRepulsiveForce(kIdx, kJointGroup, {5, 0, 0});
                    break;
                case 'A':
                    teleop.SetRepulsiveForce(kIdx, kJointGroup, {-5, 0, 0});
                    break;
                case 'b':
                    teleop.Stop();
                    break;
                case 'B':
                    teleop.Init();
                    teleop.Start();
                    break;
                case 'c':
                    teleop.SetWrenchFeedbackScalingFactor(kIdx, kJointGroup, 1);
                    break;
                case 'C':
                    teleop.SetRepulsiveForce(kIdx, kJointGroup, {0, 0, 0});
                    break;
                case 's':
                    LogInfo(teleop.stopped(kIdx) ? "Teleop pair 0 stopped" : "Teleop pair 0 started");
                    break;
                case 'h':
                    PrintTeleopStatus(teleop.GetTeleopStatus(kIdx, kJointGroup));
                    break;
                default:
                    LogWarn("Invalid command!");
                    PrintCommandMenu();
                    break;
            }
        } catch (const std::exception& e) {
            LogError(std::string("Exception in ConsoleTask: ") + e.what());
            g_running.store(false);
            return;
        }
    }
    LogInfo("Console thread exiting.");
    return;
}

int main(int argc, char* argv[])
{
    std::string follower_sn {};
    std::string leader_sn {};
    int opt = 0;
    int longIndex = 0;
    bool enable_digital_input = false;

    while ((opt = getopt_long_only(argc, argv, "f:l:D", kLongOptions, &longIndex)) != -1) {
        switch (opt) {
            case 'f':
                follower_sn = std::string(optarg);
                break;
            case 'l':
                leader_sn = std::string(optarg);
                break;
            case 'D':
                enable_digital_input = true;
                break;
            default:
                PrintHelp();
                return 1;
        }
    }
    if (leader_sn.empty() || follower_sn.empty()) {
        PrintHelp();
        return 1;
    }
    try {

        // Create teleop control interface
        flexiv::tdk::TransparentCartesianTeleopLAN tctl({{leader_sn, follower_sn}});

        // Run initialization sequence
        tctl.Init();

        // Start control loop
        tctl.Start();

        // Start console and pedal input threads
        std::thread console_thread(std::bind(ConsoleTask, std::ref(tctl)));

        // Start pedal_thread based on the new flag or the role
        std::optional<std::thread> pedal_thread;

        if (enable_digital_input) {
            LogInfo("Starting ReadDigitalInputTask thread.");
            pedal_thread.emplace(ReadDigitalInputTask, std::ref(tctl));
        } else {
            LogInfo("ReadDigitalInputTask thread NOT started (-D flag not provided).");
        }

        // Wait for threads to finish
        console_thread.join();

        // Stop all threads, notify other threads exit
        g_running.store(false);

        // Wait for digital reading task exit
        if (pedal_thread && pedal_thread->joinable()) {
            pedal_thread->join();
        }

        // Exit high transparency teleop
        tctl.Stop();

    } catch (const std::exception& e) {
        LogError(e.what());
        return 1;
    }

    return 0;
}