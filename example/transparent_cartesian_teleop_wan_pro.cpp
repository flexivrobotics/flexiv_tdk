/**
 * @example transparent_cartesian_teleop_wan_pro.cpp
 * @brief Example usage of Transparent Cartesian teleoperation over WAN (TDK Professional Edition,
 * TDK Server credential). Controls a follower robot from a leader robot with transparent force
 * feedback. Supports keyboard and digital input engage/disengage, message latency query, nullspace
 * posture tuning, max contact wrench setting, and teleop status query. Optionally enables
 * GripperRemoteControl when a follower gripper device name is provided.
 * @note This program is provided only as an example. Users must adapt it to their own application
 * requirements, safety procedures, and software architecture before deployment.
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/tdk/data.hpp>
#include <flexiv/tdk/gripper_remote_control.hpp>
#include <flexiv/tdk/transparent_cartesian_teleop_wan.hpp>

#include <getopt.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

/** Nullspace to a preferred posture */
std::vector<double> kPreferredJntPos = {60 * M_PI / 180.0, -60 * M_PI / 180.0, -85 * M_PI / 180.0,
    115 * M_PI / 180.0, 70 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0};

/** Nullspace to Home posture */
std::vector<double> kHomeJntPos = {0 * M_PI / 180.0, -40 * M_PI / 180.0, 0 * M_PI / 180.0,
    90 * M_PI / 180.0, 0 * M_PI / 180.0, 40 * M_PI / 180.0, 0 * M_PI / 180.0};

/** Maximum contact wrench for soft contact */
const std::array<double, flexiv::tdk::kCartDoF> kDefaultMaxContactWrench
    = {50.0, 50.0, 50.0, 40.0, 40.0, 40.0};

/** Robot pair index used by this example */
constexpr unsigned int kIdx = 0;

/** Atomic signal to stop console and DI reading tasks */
std::atomic<bool> g_running {true};

/** Teleop role, assigned from [-r] */
flexiv::tdk::Role g_role = flexiv::tdk::Role::UNKNOWN;

/** Name of the gripper device configured in Elements -> Device on the follower robot */
std::string g_gripper_name;

}

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

void PrintTeleopStatus(unsigned int idx, const flexiv::tdk::TeleopStatus& status)
{
    LogInfo("Teleop pair " + std::to_string(idx) + " status: initialized="
            + BoolStr(status.initialized) + " started=" + BoolStr(status.started)
            + " engaged=" + BoolStr(status.engaged) + " stopped=" + BoolStr(status.stopped)
            + " fault=" + BoolStr(status.fault)
            + " motion_restricted=" + BoolStr(status.motion_restricted) + " latency="
            + FmtFixed(status.latency_ms, 1) + "/" + FmtFixed(status.latency_threshold_ms, 1)
            + " ms");
    if (status.primary.code == flexiv::tdk::TeleopIssueCode::NONE) {
        LogInfo("No teleop restriction. Safe to continue.");
        return;
    }
    const auto& issue = status.primary;
    LogWarn(std::string("[")
            + flexiv::tdk::TeleopIssueLevelStr[static_cast<size_t>(issue.level)] + "/"
            + flexiv::tdk::TeleopIssueSideStr[static_cast<size_t>(issue.side)] + "] "
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

void PrintGripperStates(flexiv::tdk::GripperRemoteControl& gripper)
{
    try {
        auto states = gripper.states(kIdx);
        LogInfo("Gripper states: width = " + FmtFixed(states.width, 4) + " m, force = "
                + FmtFixed(states.force, 2) + " N, is_moving = " + BoolStr(states.is_moving));
    } catch (const std::exception& e) {
        LogWarn(std::string("Gripper states not available yet: ") + e.what());
    }
}

void PrintHelp()
{
    // clang-format off
    std::cout<<"Invalid program arguments!"<<std::endl;
    std::cout<<"     -l     [necessary] serial number of leader robot."<<std::endl;
    std::cout<<"     -f     [necessary] serial number of follower robot."<<std::endl;
    std::cout<<"     -r     [necessary] Role of participants in teleop. can be [follower] or [leader]"<<std::endl;
    std::cout<<"     -c     [necessary] Path to client.conf from the TDK Server credential package."<<std::endl;
    std::cout<<"     -A     [optional]  LAN IPv4 address of the NIC connected to the robot." << std::endl;
    std::cout<<"     -n     [optional]  Follower gripper device name configured in Elements -> Device." << std::endl;
    std::cout<<"                       To enable GripperRemoteControl, both leader and follower sides must provide the same gripper name." << std::endl;
    std::cout<<"     -D     [optional] Enable Digital Input reading task." << std::endl;
    std::cout<<"                       Leader DI0 engages arm teleop; with [-n], DI1 toggles the follower gripper." << std::endl;
    std::cout<<"Usage: sudo ./transparent_cartesian_teleop_wan_pro -l <leader_sn> -f <follower_sn> -r leader|follower -c <client.conf> [-A <lan_ipv4>] [-n gripper_name] [-D]"<<std::endl;
    // clang-format on
}

const struct option kLongOptions[] = {
    // clang-format off
    {"leader SN",                   required_argument,  0, 'l'},
    {"follower SN",                 required_argument,  0, 'f'},
    {"role",                        required_argument,  0, 'r'},
    {"client config",               required_argument,  0, 'c'},
    {"lan whitelist ip",            required_argument,  0, 'A'},
    {"gripper name",                required_argument,  0, 'n'},
    {"enable digital input",        no_argument,        0, 'D'},
    {0,                             0,                  0,  0 }
    // clang-format on
};

/**
 * @brief Task for reading digital input and engaging teleop.
 * Leader DI0 follows the deadman switch. With [-n], a rising edge on DI1 toggles the follower
 * gripper open/closed.
 */
void ReadDigitalInputTask(flexiv::tdk::TransparentCartesianTeleopWAN& teleop,
    std::optional<flexiv::tdk::GripperRemoteControl>& gripper)
{
    bool logged_idle = false;
    bool di1_stable = false;
    int di1_counter = 0;
    bool gripper_opened = false;
    std::string last_error;

    while (g_running.load()) {
        try {
            const auto status = teleop.GetTeleopStatus(kIdx);
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
                const auto digital_inputs = teleop.digital_inputs(kIdx);
                teleop.Engage(kIdx, digital_inputs[0]);

                if (gripper.has_value() && digital_inputs.size() > 1) {
                    const bool di1_raw = digital_inputs[1];
                    if (di1_raw != di1_stable) {
                        if (++di1_counter >= 3) {
                            di1_stable = di1_raw;
                            di1_counter = 0;
                            if (di1_stable) {
                                try {
                                    auto params = gripper->params(kIdx);
                                    if (!gripper_opened) {
                                        double width = std::clamp(
                                            0.5 * params.max_width, params.min_width, params.max_width);
                                        double velocity = std::clamp(
                                            0.5 * params.max_vel, params.min_vel, params.max_vel);
                                        double force_limit = std::clamp(
                                            0.5 * params.max_force, params.min_force, params.max_force);
                                        gripper->Move(kIdx, width, velocity, force_limit);
                                        gripper_opened = true;
                                        LogInfo("DI1 pressed: opening gripper to width = "
                                                + FmtFixed(width, 4) + " m");
                                    } else {
                                        double force
                                            = std::clamp(40.0, params.min_force, params.max_force);
                                        gripper->Grasp(kIdx, force);
                                        gripper_opened = false;
                                        LogInfo("DI1 pressed: closing gripper with force = "
                                                + FmtFixed(force, 2) + " N");
                                    }
                                } catch (const std::exception& e) {
                                    LogWarn(std::string("DI1 gripper action failed: ") + e.what());
                                }
                            }
                        }
                    } else {
                        di1_counter = 0;
                    }
                }
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
}

/**
 * @brief Task for calling TransparentCartesianTeleopWAN and optional GripperRemoteControl
 * functions from console.
 */
void ConsoleTask(flexiv::tdk::TransparentCartesianTeleopWAN& teleop,
    std::optional<flexiv::tdk::GripperRemoteControl>& gripper)
{
    auto PrintCommandMenu = [has_gripper = gripper.has_value()]() {
        std::cout << R"(
  --- Axis Lock (leader only) ---
    x/y/z    : Toggle translation lock in WORLD coord (X/Y/Z)
    q/w/e    : Toggle orientation lock in WORLD coord (Rx/Ry/Rz)
    X/Y/Z    : Toggle translation lock in TCP coord (X/Y/Z)
    Q/W/E    : Toggle orientation lock in TCP coord (Rx/Ry/Rz)
    a        : Unlock all axes (TCP coord)
    A        : Lock all axes (TCP coord)

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

  --- Tcp message latency ---
    l        : print current message latency (disconnected / clock mismatch / over limit)

  --- Teleop status ---
    h        : print why teleop is restricted / paused and what to do next

  --- Identity ---
    n        : print role() and robot_pair_sn()
        )" << std::endl;

        if (has_gripper) {
            std::cout << R"(
  --- Gripper Lifecycle (Leader only, via RPC) ---
    G        : Enable gripper on follower robot
    D        : Disable gripper on follower robot
    N        : Trigger initialization of the enabled gripper

  --- Gripper Motion (Leader only, via reliable topic) ---
    o        : Open gripper (Move to max width)
    c        : Close gripper (Grasp with moderate force)
    s        : Stop and hold gripper

  --- Gripper States ---
    t        : Print latest gripper states
    P        : Print gripper params (valid command ranges)
        )" << std::endl;
        }

        std::cout << R"(
  --- Help ---
    Any other key to show this help menu
        )" << std::endl;
    };

    const bool is_leader = (teleop.role() == flexiv::tdk::Role::WAN_TELEOP_LEADER);
    flexiv::tdk::AxisLock cmd;
    if (is_leader) {
        teleop.GetAxisLockState(kIdx, cmd);
    }

    auto apply_axis_lock = [&]() {
        if (!is_leader) {
            LogWarn("Axis lock is only available on the leader");
            return;
        }
        teleop.SetAxisLockCmd(kIdx, cmd);
    };

    while (g_running.load() && !teleop.fault(kIdx)) {

        std::string user_input {};
        std::getline(std::cin, user_input);

        if (user_input.empty()) {
            LogWarn("Empty command!");
            PrintCommandMenu();
            continue;
        }

        try {
            const char command = user_input[0];
            if (std::string("GDNocstP").find(command) != std::string::npos
                && !gripper.has_value()) {
                LogWarn("Gripper commands require [-n gripper_name].");
                continue;
            }
            if (gripper.has_value() && g_role != flexiv::tdk::Role::WAN_TELEOP_LEADER
                && std::string("GDNocs").find(command) != std::string::npos) {
                LogWarn(std::string("Command '") + command + "' is only available on the leader");
                continue;
            }

            switch (command) {
                case 'x':
                    cmd.lock_trans_axis[0] = !cmd.lock_trans_axis[0];
                    cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                    apply_axis_lock();
                    break;
                case 'y':
                    cmd.lock_trans_axis[1] = !cmd.lock_trans_axis[1];
                    cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                    apply_axis_lock();
                    break;
                case 'z':
                    cmd.lock_trans_axis[2] = !cmd.lock_trans_axis[2];
                    cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                    apply_axis_lock();
                    break;
                case 'q':
                    cmd.lock_ori_axis[0] = !cmd.lock_ori_axis[0];
                    cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                    apply_axis_lock();
                    break;
                case 'w':
                    cmd.lock_ori_axis[1] = !cmd.lock_ori_axis[1];
                    cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                    apply_axis_lock();
                    break;
                case 'e':
                    cmd.lock_ori_axis[2] = !cmd.lock_ori_axis[2];
                    cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                    apply_axis_lock();
                    break;
                case 'X':
                    cmd.lock_trans_axis[0] = !cmd.lock_trans_axis[0];
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    apply_axis_lock();
                    break;
                case 'Y':
                    cmd.lock_trans_axis[1] = !cmd.lock_trans_axis[1];
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    apply_axis_lock();
                    break;
                case 'Z':
                    cmd.lock_trans_axis[2] = !cmd.lock_trans_axis[2];
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    apply_axis_lock();
                    break;
                case 'Q':
                    cmd.lock_ori_axis[0] = !cmd.lock_ori_axis[0];
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    apply_axis_lock();
                    break;
                case 'W':
                    cmd.lock_ori_axis[1] = !cmd.lock_ori_axis[1];
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    apply_axis_lock();
                    break;
                case 'E':
                    cmd.lock_ori_axis[2] = !cmd.lock_ori_axis[2];
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    apply_axis_lock();
                    break;
                case 'a':
                    cmd.lock_ori_axis = {false, false, false};
                    cmd.lock_trans_axis = {false, false, false};
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    apply_axis_lock();
                    break;
                case 'A':
                    cmd.lock_ori_axis = {true, true, true};
                    cmd.lock_trans_axis = {true, true, true};
                    cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                    apply_axis_lock();
                    break;
                case 'r':
                    teleop.Engage(kIdx, true);
                    break;
                case 'R':
                    teleop.Engage(kIdx, false);
                    break;
                case 'i':
                    teleop.SetNullSpacePosture(kIdx, kPreferredJntPos);
                    break;
                case 'I':
                    teleop.SetNullSpacePosture(kIdx, kHomeJntPos);
                    break;
                case 'p':
                    teleop.SetMaxContactWrench(kIdx, kDefaultMaxContactWrench);
                    break;
                case 'u':
                    teleop.Init();
                    teleop.Start();
                    break;
                case 'U':
                    teleop.Stop();
                    break;
                case 'l': {
                    double latency_ms {};
                    const bool ok = teleop.CheckTeleopConnectionLatency(kIdx, latency_ms);
                    if (ok) {
                        LogInfo("pair " + std::to_string(kIdx) + " message latency: "
                                + FmtFixed(latency_ms, 1) + " ms (within limit)");
                    } else if (latency_ms < 0.0) {
                        LogWarn("pair " + std::to_string(kIdx) + " clock mismatch: latency "
                                + FmtFixed(latency_ms, 1) + " ms");
                    } else if (latency_ms > 1.0e12) {
                        LogWarn("pair " + std::to_string(kIdx) + " disconnected: latency "
                                + FmtFixed(latency_ms, 1) + " ms");
                    } else {
                        LogWarn("pair " + std::to_string(kIdx) + " latency over limit: "
                                + FmtFixed(latency_ms, 1) + " ms");
                    }
                    break;
                }
                case 'h':
                    PrintTeleopStatus(kIdx, teleop.GetTeleopStatus(kIdx));
                    break;
                case 'n': {
                    const auto pair = teleop.robot_pair_sn(kIdx);
                    LogInfo("pair " + std::to_string(kIdx) + " role="
                            + flexiv::tdk::RoleTypeStr[static_cast<size_t>(teleop.role())]
                            + " leader_sn=" + pair.first + " follower_sn=" + pair.second);
                    break;
                }
                case 'G':
                    gripper->Enable(kIdx, g_gripper_name);
                    LogInfo("Follower gripper [" + g_gripper_name + "] enabled");
                    break;
                case 'D':
                    gripper->Disable(kIdx);
                    LogInfo("Follower gripper disabled");
                    break;
                case 'N':
                    gripper->Init(kIdx);
                    LogInfo("Follower gripper initialization triggered");
                    break;
                case 'o': {
                    auto params = gripper->params(kIdx);
                    double velocity
                        = std::clamp(0.5 * params.max_vel, params.min_vel, params.max_vel);
                    double force_limit
                        = std::clamp(0.5 * params.max_force, params.min_force, params.max_force);
                    gripper->Move(kIdx, params.max_width, velocity, force_limit);
                    LogInfo("Open command sent: width = " + FmtFixed(params.max_width, 4)
                            + " m, velocity = " + FmtFixed(velocity, 4)
                            + " m/s, force_limit = " + FmtFixed(force_limit, 2) + " N");
                    break;
                }
                case 'c': {
                    auto params = gripper->params(kIdx);
                    double force
                        = std::clamp(0.5 * params.max_force, params.min_force, params.max_force);
                    gripper->Grasp(kIdx, force);
                    LogInfo("Grasp command sent: force = " + FmtFixed(force, 2) + " N");
                    break;
                }
                case 's':
                    gripper->Stop(kIdx);
                    LogInfo("Stop command sent");
                    break;
                case 't':
                    PrintGripperStates(*gripper);
                    break;
                case 'P': {
                    try {
                        auto params = gripper->params(kIdx);
                        LogInfo("Gripper params: width = [" + FmtFixed(params.min_width, 4)
                                + ", " + FmtFixed(params.max_width, 4) + "] m, velocity = ["
                                + FmtFixed(params.min_vel, 4) + ", " + FmtFixed(params.max_vel, 4)
                                + "] m/s, force = [" + FmtFixed(params.min_force, 2) + ", "
                                + FmtFixed(params.max_force, 2) + "] N");
                    } catch (const std::exception& e) {
                        LogWarn(std::string("Gripper params not available yet: ") + e.what());
                    }
                    break;
                }
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
}

int main(int argc, char* argv[])
{
    std::string follower_sn, leader_sn, teleop_role, client_config_file, lan_ip;
    std::vector<std::string> lan_interface_whitelist {};
    bool enable_digital_input = false;

    int opt = 0;
    while ((opt = getopt_long_only(argc, argv, "l:f:r:c:A:n:D", kLongOptions, nullptr)) != -1) {
        switch (opt) {
            case 'f':
                follower_sn = std::string(optarg);
                break;
            case 'l':
                leader_sn = std::string(optarg);
                break;
            case 'r':
                teleop_role = std::string(optarg);
                break;
            case 'c':
                client_config_file = std::string(optarg);
                break;
            case 'A':
                lan_ip = std::string(optarg);
                lan_interface_whitelist.push_back(lan_ip);
                break;
            case 'n':
                g_gripper_name = std::string(optarg);
                break;
            case 'D':
                enable_digital_input = true;
                break;
            default:
                PrintHelp();
                return 1;
        }
    }
    if (follower_sn.empty() || leader_sn.empty() || teleop_role.empty()
        || client_config_file.empty()) {
        PrintHelp();
        return 1;
    }
    if (lan_interface_whitelist.empty()) {
        LogWarn("LAN whitelist is not provided, RDK will search all network interfaces.");
    }

    if (teleop_role == "follower") {
        g_role = flexiv::tdk::Role::WAN_TELEOP_FOLLOWER;
    } else if (teleop_role == "leader") {
        g_role = flexiv::tdk::Role::WAN_TELEOP_LEADER;
    } else {
        LogError("Valid inputs for [-r] are: follower, leader");
        return 1;
    }

    flexiv::tdk::NetworkCfgPro network_cfg;
    network_cfg.client_config_file = client_config_file;
    network_cfg.lan_interface_whitelist = lan_interface_whitelist;

    std::vector<std::pair<std::string, std::string>> robot_sn_pairs {};
    robot_sn_pairs.push_back({leader_sn, follower_sn});
    try {
        flexiv::tdk::TransparentCartesianTeleopWAN tctw(robot_sn_pairs, g_role, network_cfg);

        const auto pair_sn = tctw.robot_pair_sn(kIdx);
        LogInfo(std::string("This instance role=")
                + flexiv::tdk::RoleTypeStr[static_cast<size_t>(tctw.role())]
                + " leader_sn=" + pair_sn.first + " follower_sn=" + pair_sn.second);

        // Allocate the gripper remote control object on top of the arm teleop object only if
        // [-n] is provided. It reuses the role, robot serial number pairs and rdk::Robot
        // connection of the arm teleop. Without [-n] the program runs as basic arm teleop only.
        std::optional<flexiv::tdk::GripperRemoteControl> gripper;
        if (!g_gripper_name.empty()) {
            gripper.emplace(tctw, network_cfg);
            LogInfo("Gripper remote control is ENABLED, target gripper device: [" + g_gripper_name
                    + "]");
        } else {
            LogInfo("Gripper remote control is DISABLED (no [-n gripper_name] provided).");
        }

        tctw.Init();
        tctw.Start();
        tctw.SetMaxContactWrench(kIdx, kDefaultMaxContactWrench);

        // Follower side: enable the configured gripper locally so the leader can see states and
        // send commands without waiting for the Enable RPC.
        if (g_role == flexiv::tdk::Role::WAN_TELEOP_FOLLOWER && gripper.has_value()) {
            try {
                gripper->EnableLocal(kIdx, g_gripper_name);
            } catch (const std::exception& e) {
                LogError(std::string("Failed to auto-enable gripper [") + g_gripper_name
                        + "] on follower: " + e.what()
                        + ". The leader can still retry via the 'G' console command.");
            }
        }

        std::thread console_thread(std::bind(ConsoleTask, std::ref(tctw), std::ref(gripper)));

        std::optional<std::thread> pedal_thread;
        if (g_role == flexiv::tdk::Role::WAN_TELEOP_LEADER && enable_digital_input) {
            LogInfo(std::string(
                        "Starting ReadDigitalInputTask thread as role is 'leader' and requested by -D "
                        "flag. DI0: arm teleop engagement; DI1: ")
                    + (gripper.has_value() ? "toggles follower gripper open/close"
                                           : "unused (gripper feature disabled, no -n)")
                    + ".");
            pedal_thread.emplace(ReadDigitalInputTask, std::ref(tctw), std::ref(gripper));
        } else {
            LogInfo(
                "ReadDigitalInputTask thread NOT started (role is not 'leader' or -D flag not "
                "provided).");
        }

        console_thread.join();
        g_running.store(false);

        if (pedal_thread && pedal_thread->joinable()) {
            pedal_thread->join();
        }

        tctw.Stop();

    } catch (const std::exception& e) {
        LogError(e.what());
        return 1;
    }

    return 0;
}
