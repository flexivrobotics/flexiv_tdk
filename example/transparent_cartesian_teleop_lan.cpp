/**
 * @example transparent_cartesian_teleop_lan.cpp
 * @brief Example usage of Transparent Cartesian teleoperation under Local Area Network for
 * controlling a follower robot using a leader robot with transparent force feedback. Supports both
 * keyboard and digital input engage/disengage signal reading, with various axes lock modes, force
 * scaling, and max contact wrench setting, etc.
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
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <functional>

namespace {
/** Nullspace to a preferred posture */
std::vector<double> kPreferredJntPos = {60 * M_PI / 180.0, -60 * M_PI / 180.0, -85 * M_PI / 180.0,
    115 * M_PI / 180.0, 70 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0};

/** Nullspace to Home posture */
std::vector<double> kHomeJntPos = {0 * M_PI / 180.0, -40 * M_PI / 180.0, 0 * M_PI / 180.0,
    90 * M_PI / 180.0, 0 * M_PI / 180.0, 40 * M_PI / 180.0, 0 * M_PI / 180.0};

/** Maximum contact wrench for soft contact*/
const std::array<double, flexiv::tdk::kCartDoF> kDefaultMaxContactWrench
    = {25.0, 25.0, 25.0, 10.0, 10.0, 10.0};

/** Atomic signal to stop console, fault monitor and DI reading tasks */
std::atomic<bool> g_running {true};

std::string BoolStr(bool v)
{
    return v ? "true" : "false";
}

std::string FmtFixed(double v, int prec)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(prec) << v;
    return oss.str();
}

std::string VecStr(const std::vector<double>& vec, int prec)
{
    std::string out = "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        out += (i == 0 ? "" : ", ") + FmtFixed(vec[i], prec);
    }
    return out + "]";
}

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

} // namespace

void PrintTeleopStatus(unsigned int idx, const flexiv::tdk::TeleopStatus& status)
{
    LogInfo(
        "Teleop pair " + std::to_string(idx) + " status: initialized=" + BoolStr(status.initialized)
        + " started=" + BoolStr(status.started) + " engaged=" + BoolStr(status.engaged)
        + " stopped=" + BoolStr(status.stopped) + " fault=" + BoolStr(status.fault)
        + " motion_restricted=" + BoolStr(status.motion_restricted) + " latency="
        + FmtFixed(status.latency_ms, 1) + "/" + FmtFixed(status.latency_threshold_ms, 1) + " ms");
    if (status.primary.code == flexiv::tdk::TeleopIssueCode::NONE) {
        LogInfo("No teleop restriction. Safe to continue.");
        return;
    }
    const auto& issue = status.primary;
    LogWarn(std::string("[") + flexiv::tdk::TeleopIssueLevelStr[static_cast<size_t>(issue.level)]
            + "/" + flexiv::tdk::TeleopIssueSideStr[static_cast<size_t>(issue.side)] + "] "
            + issue.title + " | " + issue.description + " | " + issue.suggestion);
    for (const auto& extra : status.issues) {
        if (extra.code == issue.code && extra.side == issue.side
            && extra.joint_index == issue.joint_index) {
            continue;
        }
        LogWarn(std::string("  also: [")
                + flexiv::tdk::TeleopIssueCodeStr[static_cast<size_t>(extra.code)] + "/"
                + flexiv::tdk::TeleopIssueSideStr[static_cast<size_t>(extra.side)] + "] "
                + extra.title);
    }
}

void PrintHelp()
{
    // clang-format off
    std::cout<<"Invalid program arguments!"<<std::endl;
    std::cout<<"     -l     [necessary] serial number of leader robot."<<std::endl;
    std::cout<<"     -f     [necessary] serial number of follower robot."<<std::endl;
    std::cout<<"     -A     [optional] LAN IPv4 address of the NIC connected to the robot." << std::endl;
    std::cout<<"     -D     [optional] Enable Digital Input reading task." << std::endl;
    std::cout<<"Usage: sudo ./transparent_cartesian_teleop_lan -l <leader_sn> -f <follower_sn> [-A <lan_ipv4>] [-D]"<<std::endl;
    // clang-format on
}

const struct option kLongOptions[] = {
    // clang-format off
    {"leader SN",                   required_argument,  0, 'l'},
    {"follower SN",                 required_argument,  0, 'f'},
    {"lan whitelist ip",            required_argument,  0, 'A'},
    {"enable digital input",        no_argument,        0, 'D'},
    {0,                                             0,  0,  0 }
    // clang-format on
};

/**
 * @brief Task for monitoring DI signals and engaging teleop.
 */
void ReadDigitalInputTask(flexiv::tdk::TransparentCartesianTeleopLAN& teleop)
{
    bool logged_idle = false;
    std::string last_error;
    while (g_running.load()) {
        try {
            const auto status = teleop.GetTeleopStatus(0);
            // Robot fault / Stop() drops teleop back to not-started. Engage() is
            // invalid in that state and must not be polled every cycle.
            if (!status.started || status.stopped) {
                if (!logged_idle) {
                    LogWarn(
                        "ReadDigitalInputTask: teleop is not started, pause Engage "
                        "(call Init + Start to resume)");
                    logged_idle = true;
                }
            } else {
                logged_idle = false;
                teleop.Engage(0, teleop.digital_inputs(0).first[0]);
            }
            last_error.clear();
        } catch (const std::exception& e) {
            if (last_error != e.what()) {
                LogError(std::string("Exception in ReadDigitalInputTask: ") + e.what());
                last_error = e.what();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    LogInfo("ReadDigitalInputTask exiting.");
    return;
}

/**
 * @brief Task for monitoring robot fault and prompting the operator to recover.
 */
void FaultMonitorTask(flexiv::tdk::TransparentCartesianTeleopLAN& teleop)
{
    bool last_fault = false;
    std::string last_error;
    while (g_running.load()) {
        try {
            const bool fault = teleop.any_fault();
            const bool fault_changed = (fault != last_fault);
            last_fault = fault;
            if (fault_changed && fault) {
                LogError("Fault detected. Press 'h' for details, 'F' to clear fault, then 'B' to "
                         "restart teleop.");
                PrintTeleopStatus(0, teleop.GetTeleopStatus(0));
            } else if (fault_changed) {
                LogInfo("Fault cleared. Press 'B' to restart teleop.");
            }
            last_error.clear();
        } catch (const std::exception& e) {
            if (last_error != e.what()) {
                LogError(std::string("Exception in FaultMonitorTask: ") + e.what());
                last_error = e.what();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    LogInfo("FaultMonitorTask exiting.");
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
    C        : Clear repulsive force

  --- Start/Stop ---
    b        : Stop teleop
    B        : Start teleop

  --- Is teleop stopped or not ---
    s        : Query if teleop stopped or not

  --- Teleop status ---
    h        : Print why teleop is restricted / paused and what to do next

  --- Fault recovery ---
    F        : Clear fault (blocking, up to 30 s), then press 'B' to restart teleop

  --- Home all robots ---
    H        : Stop teleop and home all robots. Confirm the workspace is clear first.

  --- Print action/states via instance ---
    P        : Print follower action and state

  --- Help ---
    Any other key to show this help menu
        )" << std::endl;
    };

    unsigned int index = 0;
    flexiv::tdk::AxisLock cmd;
    teleop.GetAxisLockState(index, cmd);

    while (g_running.load()) {

        std::string userInput {};

        // Get user input, exit when stdin is closed
        if (!std::getline(std::cin, userInput)) {
            LogInfo("Console input closed.");
            break;
        }

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
                    teleop.SetAxisLockCmd(index, cmd);
                    break;
                case 'y':
                    cmd.lock_trans_axis[1] = !cmd.lock_trans_axis[1];
                    cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                    teleop.SetAxisLockCmd(index, cmd);
                    break;
                case 'z':
                    cmd.lock_trans_axis[2] = !cmd.lock_trans_axis[2];
                    cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                    teleop.SetAxisLockCmd(index, cmd);
                    break;
                case 'q':
                    cmd.lock_ori_axis[0] = !cmd.lock_ori_axis[0];
                    cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                    teleop.SetAxisLockCmd(index, cmd);
                    break;
                case 'w':
                    cmd.lock_ori_axis[1] = !cmd.lock_ori_axis[1];
                    cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                    teleop.SetAxisLockCmd(index, cmd);
                    break;
                case 'e':
                    cmd.lock_ori_axis[2] = !cmd.lock_ori_axis[2];
                    cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                    teleop.SetAxisLockCmd(index, cmd);
                    break;

                case 'X':
                    cmd.lock_trans_axis[0] = !cmd.lock_trans_axis[0];
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    teleop.SetAxisLockCmd(index, cmd);
                    break;
                case 'Y':
                    cmd.lock_trans_axis[1] = !cmd.lock_trans_axis[1];
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    teleop.SetAxisLockCmd(index, cmd);
                    break;
                case 'Z':
                    cmd.lock_trans_axis[2] = !cmd.lock_trans_axis[2];
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    teleop.SetAxisLockCmd(index, cmd);
                    break;
                case 'Q':
                    cmd.lock_ori_axis[0] = !cmd.lock_ori_axis[0];
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    teleop.SetAxisLockCmd(index, cmd);
                    break;
                case 'W':
                    cmd.lock_ori_axis[1] = !cmd.lock_ori_axis[1];
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    teleop.SetAxisLockCmd(index, cmd);
                    break;
                case 'E':
                    cmd.lock_ori_axis[2] = !cmd.lock_ori_axis[2];
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    teleop.SetAxisLockCmd(index, cmd);
                    break;

                case 'r':
                    teleop.Engage(index, true);
                    break;
                case 'R':
                    teleop.Engage(index, false);
                    break;

                case 't':
                    teleop.SetWrenchFeedbackScalingFactor(index, 0.5);
                    break;
                case 'T':
                    teleop.SetWrenchFeedbackScalingFactor(index, 2);
                    break;

                case 'u':
                    cmd.lock_ori_axis = {false, false, false};
                    cmd.lock_trans_axis = {false, false, false};
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    teleop.SetAxisLockCmd(index, cmd);
                    break;
                case 'U':
                    cmd.lock_ori_axis = {true, true, true};
                    cmd.lock_trans_axis = {true, true, true};
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    teleop.SetAxisLockCmd(index, cmd);
                    break;
                case 'i':
                    teleop.SetLeaderNullSpacePosture(index, kPreferredJntPos);
                    break;
                case 'I':
                    teleop.SetLeaderNullSpacePosture(index, kHomeJntPos);
                    break;
                case 'o':
                    teleop.SetFollowerNullSpacePosture(index, kPreferredJntPos);
                    break;
                case 'O':
                    teleop.SetFollowerNullSpacePosture(index, kHomeJntPos);
                    break;
                case 'p':
                    teleop.SetFollowerMaxContactWrench(index, kDefaultMaxContactWrench);
                    break;

                case 'a':
                    teleop.SetRepulsiveForce(index, {5, 0, 0});
                    break;
                case 'A':
                    teleop.SetRepulsiveForce(index, {-5, 0, 0});
                    break;
                case 'b':
                    teleop.Stop();
                    break;
                case 'B':
                    teleop.Init();
                    teleop.Start();
                    break;
                case 'c':
                    teleop.SetWrenchFeedbackScalingFactor(index, 1);
                    break;
                case 'C':
                    teleop.SetRepulsiveForce(index, {0, 0, 0});
                    break;
                case 's':
                    LogInfo(teleop.stopped(0) ? "Teleop pair 0 stopped" : "Teleop pair 0 started");
                    break;
                case 'h':
                    PrintTeleopStatus(0, teleop.GetTeleopStatus(0));
                    break;
                case 'F': {
                    LogInfo("Clearing fault, this may take up to 30 seconds...");
                    const auto results = teleop.ClearFault();
                    for (size_t i = 0; i < results.size(); ++i) {
                        LogInfo("Teleop pair " + std::to_string(i)
                                + " clear fault: leader=" + BoolStr(results[i].first)
                                + " follower=" + BoolStr(results[i].second));
                    }
                    break;
                }
                case 'H':
                    teleop.Stop();
                    teleop.HomeAll();
                    LogInfo("Teleop stopped for homing");
                    break;
                case 'P': {
                    const auto follower = teleop.instances(index).second;
                    LogInfo("Follower q: " + VecStr(follower->states().q, 2));
                    LogInfo("Follower q_d: " + VecStr(follower->actions().q_d, 2));
                    break;
                }
                default:
                    LogWarn("Invalid command!");
                    PrintCommandMenu();
                    break;
            }
        } catch (const std::exception& e) {
            LogError(std::string("Command '") + userInput[0] + "' failed: " + e.what());
            try {
                PrintTeleopStatus(0, teleop.GetTeleopStatus(0));
            } catch (const std::exception& status_e) {
                LogError(std::string("Failed to query teleop status: ") + status_e.what());
            }
        }
    }
    LogInfo("Console thread exiting.");
    return;
}

int main(int argc, char* argv[])
{
    std::string follower_sn {};
    std::string leader_sn {};
    std::string whitelist_ip {};
    std::vector<std::string> network_interface_whitelist {};
    int opt = 0;
    int longIndex = 0;
    bool enable_digital_input = false;

    while ((opt = getopt_long_only(argc, argv, "f:l:A:D", kLongOptions, &longIndex)) != -1) {
        switch (opt) {
            case 'f':
                follower_sn = std::string(optarg);
                break;
            case 'l':
                leader_sn = std::string(optarg);
                break;
            case 'A':
                whitelist_ip = std::string(optarg);
                network_interface_whitelist.emplace_back(whitelist_ip);
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
    if (network_interface_whitelist.empty()) {
        LogWarn("network_interface_whitelist is not provided, will search all network interfaces.");
    }

    try {

        // Create teleop control interface
        flexiv::tdk::TransparentCartesianTeleopLAN tctl(
            {{leader_sn, follower_sn}}, network_interface_whitelist);

        // Run initialization sequence
        tctl.Init();

        // Start control loop
        tctl.Start();

        // Start console, fault monitor and pedal input threads
        std::thread console_thread(std::bind(ConsoleTask, std::ref(tctl)));
        std::thread fault_monitor_thread(FaultMonitorTask, std::ref(tctl));

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

        // Wait for fault monitor and digital reading task exit
        fault_monitor_thread.join();
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