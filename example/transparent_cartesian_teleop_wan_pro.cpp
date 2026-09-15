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

#include <spdlog/spdlog.h>

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

void PrintTeleopStatus(unsigned int idx, const flexiv::tdk::TeleopStatus& status)
{
    spdlog::info(
        "Teleop pair {} status: initialized={} started={} engaged={} stopped={} fault={} "
        "motion_restricted={} latency={:.1f}/{:.1f} ms",
        idx, status.initialized, status.started, status.engaged, status.stopped, status.fault,
        status.motion_restricted, status.latency_ms, status.latency_threshold_ms);
    if (status.primary.code == flexiv::tdk::TeleopIssueCode::NONE) {
        spdlog::info("No teleop restriction. Safe to continue.");
        return;
    }
    const auto& issue = status.primary;
    spdlog::warn("[{}/{}] {} | {} | {}",
        flexiv::tdk::TeleopIssueLevelStr[static_cast<size_t>(issue.level)],
        flexiv::tdk::TeleopIssueSideStr[static_cast<size_t>(issue.side)], issue.title,
        issue.description, issue.suggestion);
    for (const auto& extra : status.issues) {
        if (extra.code == issue.code && extra.side == issue.side
            && extra.joint_index == issue.joint_index) {
            continue;
        }
        spdlog::warn("  also: [{}/{}] {}",
            flexiv::tdk::TeleopIssueCodeStr[static_cast<size_t>(extra.code)],
            flexiv::tdk::TeleopIssueSideStr[static_cast<size_t>(extra.side)], extra.title);
    }
}

void PrintGripperStates(flexiv::tdk::GripperRemoteControl& gripper)
{
    try {
        auto states = gripper.states(kIdx);
        spdlog::info("Gripper states: width = {:.4f} m, force = {:.2f} N, is_moving = {}",
            states.width, states.force, states.is_moving);
    } catch (const std::exception& e) {
        spdlog::warn("Gripper states not available yet: {}", e.what());
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
                    spdlog::warn(
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
                                        spdlog::info(
                                            "DI1 pressed: opening gripper to width = {:.4f} m",
                                            width);
                                    } else {
                                        double force
                                            = std::clamp(40.0, params.min_force, params.max_force);
                                        gripper->Grasp(kIdx, force);
                                        gripper_opened = false;
                                        spdlog::info(
                                            "DI1 pressed: closing gripper with force = {:.2f} N",
                                            force);
                                    }
                                } catch (const std::exception& e) {
                                    spdlog::warn("DI1 gripper action failed: {}", e.what());
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
                spdlog::error("Exception in ReadDigitalInputTask: {}", e.what());
                last_error = e.what();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    spdlog::info("ReadDigitalInputTask exiting.");
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
            spdlog::warn("Axis lock is only available on the leader");
            return;
        }
        teleop.SetAxisLockCmd(kIdx, cmd);
    };

    while (g_running.load() && !teleop.fault(kIdx)) {

        std::string user_input {};
        std::getline(std::cin, user_input);

        if (user_input.empty()) {
            spdlog::warn("Empty command!");
            PrintCommandMenu();
            continue;
        }

        try {
            const char command = user_input[0];
            if (std::string("GDNocstP").find(command) != std::string::npos
                && !gripper.has_value()) {
                spdlog::warn("Gripper commands require [-n gripper_name].");
                continue;
            }
            if (gripper.has_value() && g_role != flexiv::tdk::Role::WAN_TELEOP_LEADER
                && std::string("GDNocs").find(command) != std::string::npos) {
                spdlog::warn("Command '{}' is only available on the leader", command);
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
                        spdlog::info(
                            "pair {} message latency: {} ms (within limit)", kIdx, latency_ms);
                    } else if (latency_ms < 0.0) {
                        spdlog::warn("pair {} clock mismatch: latency {} ms", kIdx, latency_ms);
                    } else if (latency_ms > 1.0e12) {
                        spdlog::warn("pair {} disconnected: latency {} ms", kIdx, latency_ms);
                    } else {
                        spdlog::warn("pair {} latency over limit: {} ms", kIdx, latency_ms);
                    }
                    break;
                }
                case 'h':
                    PrintTeleopStatus(kIdx, teleop.GetTeleopStatus(kIdx));
                    break;
                case 'n': {
                    const auto pair = teleop.robot_pair_sn(kIdx);
                    spdlog::info("pair {} role={} leader_sn={} follower_sn={}", kIdx,
                        flexiv::tdk::RoleTypeStr[static_cast<size_t>(teleop.role())], pair.first,
                        pair.second);
                    break;
                }
                case 'G':
                    gripper->Enable(kIdx, g_gripper_name);
                    spdlog::info("Follower gripper [{}] enabled", g_gripper_name);
                    break;
                case 'D':
                    gripper->Disable(kIdx);
                    spdlog::info("Follower gripper disabled");
                    break;
                case 'N':
                    gripper->Init(kIdx);
                    spdlog::info("Follower gripper initialization triggered");
                    break;
                case 'o': {
                    auto params = gripper->params(kIdx);
                    double velocity
                        = std::clamp(0.5 * params.max_vel, params.min_vel, params.max_vel);
                    double force_limit
                        = std::clamp(0.5 * params.max_force, params.min_force, params.max_force);
                    gripper->Move(kIdx, params.max_width, velocity, force_limit);
                    spdlog::info(
                        "Open command sent: width = {:.4f} m, velocity = {:.4f} m/s, "
                        "force_limit = {:.2f} N",
                        params.max_width, velocity, force_limit);
                    break;
                }
                case 'c': {
                    auto params = gripper->params(kIdx);
                    double force
                        = std::clamp(0.5 * params.max_force, params.min_force, params.max_force);
                    gripper->Grasp(kIdx, force);
                    spdlog::info("Grasp command sent: force = {:.2f} N", force);
                    break;
                }
                case 's':
                    gripper->Stop(kIdx);
                    spdlog::info("Stop command sent");
                    break;
                case 't':
                    PrintGripperStates(*gripper);
                    break;
                case 'P': {
                    try {
                        auto params = gripper->params(kIdx);
                        spdlog::info(
                            "Gripper params: width = [{:.4f}, {:.4f}] m, velocity = [{:.4f}, "
                            "{:.4f}] m/s, force = [{:.2f}, {:.2f}] N",
                            params.min_width, params.max_width, params.min_vel, params.max_vel,
                            params.min_force, params.max_force);
                    } catch (const std::exception& e) {
                        spdlog::warn("Gripper params not available yet: {}", e.what());
                    }
                    break;
                }
                default:
                    spdlog::warn("Invalid command!");
                    PrintCommandMenu();
                    break;
            }
        } catch (const std::exception& e) {
            spdlog::error("Exception in ConsoleTask: {}", e.what());
            g_running.store(false);
            return;
        }
    }
    spdlog::info("Console thread exiting.");
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
        spdlog::warn("LAN whitelist is not provided, RDK will search all network interfaces.");
    }

    if (teleop_role == "follower") {
        g_role = flexiv::tdk::Role::WAN_TELEOP_FOLLOWER;
    } else if (teleop_role == "leader") {
        g_role = flexiv::tdk::Role::WAN_TELEOP_LEADER;
    } else {
        spdlog::error("Valid inputs for [-r] are: follower, leader");
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
        spdlog::info("This instance role={} leader_sn={} follower_sn={}",
            flexiv::tdk::RoleTypeStr[static_cast<size_t>(tctw.role())], pair_sn.first,
            pair_sn.second);

        // Allocate the gripper remote control object on top of the arm teleop object only if
        // [-n] is provided. It reuses the role, robot serial number pairs and rdk::Robot
        // connection of the arm teleop. Without [-n] the program runs as basic arm teleop only.
        std::optional<flexiv::tdk::GripperRemoteControl> gripper;
        if (!g_gripper_name.empty()) {
            gripper.emplace(tctw, network_cfg);
            spdlog::info(
                "Gripper remote control is ENABLED, target gripper device: [{}]", g_gripper_name);
        } else {
            spdlog::info("Gripper remote control is DISABLED (no [-n gripper_name] provided).");
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
                spdlog::error(
                    "Failed to auto-enable gripper [{}] on follower: {}. The leader can still "
                    "retry via the 'G' console command.",
                    g_gripper_name, e.what());
            }
        }

        std::thread console_thread(std::bind(ConsoleTask, std::ref(tctw), std::ref(gripper)));

        std::optional<std::thread> pedal_thread;
        if (g_role == flexiv::tdk::Role::WAN_TELEOP_LEADER && enable_digital_input) {
            spdlog::info(
                "Starting ReadDigitalInputTask thread as role is 'leader' and requested by -D "
                "flag. DI0: arm teleop engagement; DI1: {}.",
                gripper.has_value() ? "toggles follower gripper open/close"
                                    : "unused (gripper feature disabled, no -n)");
            pedal_thread.emplace(ReadDigitalInputTask, std::ref(tctw), std::ref(gripper));
        } else {
            spdlog::info(
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
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}
