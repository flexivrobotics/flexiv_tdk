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
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

/** Nullspace to a preferred posture */
const std::vector<double> kPreferredJntPos = {60 * M_PI / 180.0, -60 * M_PI / 180.0,
    -85 * M_PI / 180.0, 115 * M_PI / 180.0, 70 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0};

/** Nullspace to Home posture */
const std::vector<double> kHomeJntPos = {0 * M_PI / 180.0, -40 * M_PI / 180.0, 0 * M_PI / 180.0,
    90 * M_PI / 180.0, 0 * M_PI / 180.0, 40 * M_PI / 180.0, 0 * M_PI / 180.0};

/** Maximum contact wrench for soft contact */
const std::array<double, flexiv::tdk::kCartDoF> kDefaultMaxContactWrench
    = {50.0, 50.0, 50.0, 40.0, 40.0, 40.0};

/** Robot pair index used by this example */
constexpr unsigned int kIdx = 0;

/** Single-arm joint group controlled by this example */
constexpr auto kJointGroup = flexiv::rdk::JointGroup::ARM_1;

/** Atomic signal to stop console and DI reading tasks */
std::atomic<bool> g_running {true};

/** Teleop role, assigned from [-r] */
flexiv::tdk::Role g_role = flexiv::tdk::Role::UNKNOWN;

/** Name of the gripper device configured in Elements -> Device on the follower robot */
std::string g_gripper_name;

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
    LogInfo("Teleop status: initialized=" + std::to_string(status.initialized) + " started="
            + std::to_string(status.started) + " engaged=" + std::to_string(status.engaged)
            + " stopped=" + std::to_string(status.stopped)
            + " fault=" + std::to_string(status.fault)
            + " motion_restricted=" + std::to_string(status.motion_restricted)
            + " latency=" + std::to_string(status.latency_ms) + "/"
            + std::to_string(status.latency_threshold_ms) + " ms");
    if (status.primary.code == flexiv::tdk::TeleopIssueCode::NONE) {
        LogInfo("No teleop restriction. Safe to continue.");
        return;
    }
    const auto& issue = status.primary;
    LogWarn(std::string("[") + flexiv::tdk::TeleopIssueLevelStr[static_cast<size_t>(issue.level)]
            + "] " + issue.title + " | " + issue.description + " | " + issue.suggestion);
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

void PrintGripperStates(flexiv::tdk::GripperRemoteControl& gripper)
{
    try {
        auto states = gripper.states(kIdx, kJointGroup);
        LogInfo("Gripper states: width = " + std::to_string(states.width)
                + " m, force = " + std::to_string(states.force)
                + " N, is_moving = " + std::string(states.is_moving ? "true" : "false"));
    } catch (const std::exception& e) {
        LogWarn(std::string("Gripper states not available yet: ") + e.what());
    }
}

} // namespace

void PrintHelp()
{
    // clang-format off
    std::cout<<"Invalid program arguments!"<<std::endl;
    std::cout<<"     -l     [necessary] serial number of leader robot."<<std::endl;
    std::cout<<"     -f     [necessary] serial number of follower robot."<<std::endl;
    std::cout<<"     -r     [necessary] Role of participants in teleop. can be [follower] or [leader]"<<std::endl;
    std::cout<<"     -c     [necessary] Path to client.conf from the TDK Server credential package."<<std::endl;
    std::cout<<"     -n     [optional]  Follower gripper device name configured in Elements -> Device." << std::endl;
    std::cout<<"                       To enable GripperRemoteControl, both leader and follower sides must provide the same gripper name." << std::endl;
    std::cout<<"     -D     [optional] Enable Digital Input reading task." << std::endl;
    std::cout<<"                       Leader DI0 engages arm teleop; with [-n], DI1 toggles the follower gripper." << std::endl;
    std::cout<<"Usage: sudo ./transparent_cartesian_teleop_wan_pro [-l leader_robot_serial_number] [-f follower_robot_serial_number] [-r leader/follower] [-c client.conf] [-n gripper_name] [-D]"<<std::endl;
    // clang-format on
}

const struct option kLongOptions[] = {
    // clang-format off
    {"leader SN",                   required_argument,  0, 'l'},
    {"follower SN",                 required_argument,  0, 'f'},
    {"role",                        required_argument,  0, 'r'},
    {"client config",               required_argument,  0, 'c'},
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
    bool di1_stable = false;
    int di1_counter = 0;
    bool gripper_opened = false;

    while (g_running.load() && !teleop.fault(kIdx)) {
        try {
            const auto digital_inputs = teleop.digital_inputs(kIdx);
            teleop.Engage(kIdx, kJointGroup, digital_inputs[0]);

            if (gripper.has_value() && digital_inputs.size() > 1) {
                const bool di1_raw = digital_inputs[1];
                if (di1_raw != di1_stable) {
                    if (++di1_counter >= 3) {
                        di1_stable = di1_raw;
                        di1_counter = 0;
                        if (di1_stable) {
                            try {
                                auto params = gripper->params(kIdx, kJointGroup);
                                if (!gripper_opened) {
                                    double width = std::clamp(
                                        0.5 * params.max_width, params.min_width, params.max_width);
                                    double velocity = std::clamp(
                                        0.5 * params.max_vel, params.min_vel, params.max_vel);
                                    double force_limit = std::clamp(
                                        0.5 * params.max_force, params.min_force, params.max_force);
                                    gripper->Move(kIdx, kJointGroup, width, velocity, force_limit);
                                    gripper_opened = true;
                                    LogInfo("DI1 pressed: opening gripper to width = "
                                            + std::to_string(width) + " m");
                                } else {
                                    double force
                                        = std::clamp(40.0, params.min_force, params.max_force);
                                    gripper->Grasp(kIdx, kJointGroup, force);
                                    gripper_opened = false;
                                    LogInfo("DI1 pressed: closing gripper with force = "
                                            + std::to_string(force) + " N");
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
        } catch (const std::exception& e) {
            LogError(std::string("Exception in ReadDigitalInputTask: ") + e.what());
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
    l        : print current message latency in milliseconds

  --- Teleop status ---
    h        : print why teleop is restricted / paused and what to do next
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
                case 'r':
                    teleop.Engage(kIdx, kJointGroup, true);
                    break;
                case 'R':
                    teleop.Engage(kIdx, kJointGroup, false);
                    break;
                case 'i':
                    teleop.SetNullSpacePosture(kIdx, kJointGroup, kPreferredJntPos);
                    break;
                case 'I':
                    teleop.SetNullSpacePosture(kIdx, kJointGroup, kHomeJntPos);
                    break;
                case 'p':
                    teleop.SetMaxContactWrench(kIdx, kJointGroup, kDefaultMaxContactWrench);
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
                    if (teleop.CheckTeleopConnectionLatency(kIdx, latency_ms)) {
                        LogInfo("Current message latency is: " + std::to_string(latency_ms) + "ms");
                    } else {
                        LogWarn("WAN teleop is disconnected.");
                    }
                    break;
                }
                case 'h':
                    PrintTeleopStatus(teleop.GetTeleopStatus(kIdx, kJointGroup));
                    break;
                case 'G':
                    gripper->Enable(kIdx, kJointGroup, g_gripper_name);
                    LogInfo("Follower gripper [" + g_gripper_name + "] enabled");
                    break;
                case 'D':
                    gripper->Disable(kIdx, kJointGroup);
                    LogInfo("Follower gripper disabled");
                    break;
                case 'N':
                    gripper->Init(kIdx, kJointGroup);
                    LogInfo("Follower gripper initialization triggered");
                    break;
                case 'o': {
                    auto params = gripper->params(kIdx, kJointGroup);
                    double velocity
                        = std::clamp(0.5 * params.max_vel, params.min_vel, params.max_vel);
                    double force_limit
                        = std::clamp(0.5 * params.max_force, params.min_force, params.max_force);
                    gripper->Move(kIdx, kJointGroup, params.max_width, velocity, force_limit);
                    LogInfo("Open command sent: width = " + std::to_string(params.max_width)
                            + " m, velocity = " + std::to_string(velocity)
                            + " m/s, force_limit = " + std::to_string(force_limit) + " N");
                    break;
                }
                case 'c': {
                    auto params = gripper->params(kIdx, kJointGroup);
                    double force
                        = std::clamp(0.5 * params.max_force, params.min_force, params.max_force);
                    gripper->Grasp(kIdx, kJointGroup, force);
                    LogInfo("Grasp command sent: force = " + std::to_string(force) + " N");
                    break;
                }
                case 's':
                    gripper->Stop(kIdx, kJointGroup);
                    LogInfo("Stop command sent");
                    break;
                case 't':
                    PrintGripperStates(*gripper);
                    break;
                case 'P': {
                    try {
                        auto params = gripper->params(kIdx, kJointGroup);
                        LogInfo("Gripper params: width = [" + std::to_string(params.min_width)
                                + ", " + std::to_string(params.max_width) + "] m, velocity = ["
                                + std::to_string(params.min_vel) + ", "
                                + std::to_string(params.max_vel) + "] m/s, force = ["
                                + std::to_string(params.min_force) + ", "
                                + std::to_string(params.max_force) + "] N");
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
    std::string follower_sn, leader_sn, teleop_role, client_config_file;
    bool enable_digital_input = false;

    int opt = 0;
    while ((opt = getopt_long_only(argc, argv, "l:f:r:c:n:D", kLongOptions, nullptr)) != -1) {
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

    std::vector<std::pair<std::string, std::string>> robot_sn_pairs {};
    robot_sn_pairs.push_back({leader_sn, follower_sn});
    try {
        flexiv::tdk::TransparentCartesianTeleopWAN tctw(robot_sn_pairs, g_role, network_cfg);

        const auto pair_sn = tctw.robot_pair_sn(kIdx);
        LogInfo(std::string("role=") + flexiv::tdk::RoleTypeStr[static_cast<size_t>(tctw.role())]
                + " robot_pair_sn=(" + pair_sn.first + ", " + pair_sn.second + ")");

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
        tctw.SetMaxContactWrench(kIdx, kJointGroup, kDefaultMaxContactWrench);

        // Follower side: enable the configured gripper locally so the leader can see states and
        // send commands without waiting for the Enable RPC.
        if (g_role == flexiv::tdk::Role::WAN_TELEOP_FOLLOWER && gripper.has_value()) {
            try {
                gripper->EnableLocal(kIdx, kJointGroup, g_gripper_name);
            } catch (const std::exception& e) {
                LogError(std::string("Failed to auto-enable gripper [") + g_gripper_name
                         + "] on follower: " + e.what()
                         + ". The leader can still retry via the 'G' console command.");
            }
        }

        std::thread console_thread(std::bind(ConsoleTask, std::ref(tctw), std::ref(gripper)));

        std::optional<std::thread> pedal_thread;
        if (g_role == flexiv::tdk::Role::WAN_TELEOP_LEADER && enable_digital_input) {
            LogInfo(
                "Starting ReadDigitalInputTask thread as role is 'leader' and requested by -D "
                "flag. DI0: arm teleop engagement; DI1: "
                + std::string(gripper.has_value() ? "toggles follower gripper open/close"
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
