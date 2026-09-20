/**
 * @example transparent_cartesian_teleop_wan.cpp
 * @brief Example usage of Transparent Cartesian teleoperation cross Wide Area Network for
 * controlling a follower robot using a leader robot with transparent force feedback. Supports both
 * keyboard and digital input engage/disengage signal reading, with axis lock, message latency
 * query, nullspace posture tuning, and max contact wrench setting, etc.
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/tdk/data.hpp>
#include <flexiv/tdk/transparent_cartesian_teleop_wan.hpp>

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

namespace {

/** Nullspace to a preferred posture */
std::vector<double> kPreferredJntPos = {60 * M_PI / 180.0, -60 * M_PI / 180.0, -85 * M_PI / 180.0,
    115 * M_PI / 180.0, 70 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0};

/** Nullspace to Home posture */
std::vector<double> kHomeJntPos = {0 * M_PI / 180.0, -40 * M_PI / 180.0, 0 * M_PI / 180.0,
    90 * M_PI / 180.0, 0 * M_PI / 180.0, 40 * M_PI / 180.0, 0 * M_PI / 180.0};

/** Maximum contact wrench for soft contact*/
const std::array<double, flexiv::tdk::kCartDoF> kDefaultMaxContactWrench
    = {50.0, 50.0, 50.0, 40.0, 40.0, 40.0};

/** Atomic signal to stop console and DI reading tasks */
std::atomic<bool> g_running {true};

/** Teleop role */
flexiv::tdk::Role kRole;
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

void PrintHelp()
{
    // clang-format off
    std::cout<<"Invalid program arguments!"<<std::endl;
    std::cout<<"     -l     [necessary] serial number of leader robot."<<std::endl;
    std::cout<<"     -f     [necessary] serial number of follower robot."<<std::endl;
    std::cout<<"     -r     [necessary] Role of participants in teleop. can be [follower] or [leader]"<<std::endl;
    std::cout<<"     -t     [necessary] Role in the TCP connection, can be [server] or [client]."<<std::endl;
    std::cout<<"     -i     [necessary] Public IPV4 address of the machine that functions as TCP server."<<std::endl;
    std::cout<<"     -p     [necessary] Listening port of the TCP server machine."<<std::endl;
    std::cout<<"     -A     [optional] LAN IPv4 address of the NIC connected to the robot." << std::endl;
    std::cout<<"     -W     [optional] WAN interface name, e.g. wlo1 or enp3s0. Repeatable." << std::endl;
    std::cout<<"     -D     [optional] Enable Digital Input reading task." << std::endl;
    std::cout<<"Usage: sudo ./transparent_cartesian_teleop_wan -l <leader_sn> -f <follower_sn> -r leader|follower -t server|client -i <public_ip> -p <port> [-A <lan_ipv4>] [-W <iface>] [-D]"<<std::endl;
    // clang-format on
}

const struct option kLongOptions[] = {
    // clang-format off
    {"leader SN",                   required_argument,  0, 'l'},
    {"follower SN",                 required_argument,  0, 'f'},
    {"role",                        required_argument,  0, 'r'},
    {"tcp role",                    required_argument,  0, 't'},
    {"public ipv4 address",         required_argument,  0, 'i'},
    {"port",                        required_argument,  0, 'p'},
    {"lan whitelist ip",            required_argument,  0, 'A'},
    {"wan interface name",          required_argument,  0, 'W'},
    {"enable digital input",        no_argument,        0, 'D'},
    {0,                             0,                  0,  0 }
    // clang-format on
};

/**
 * @brief Task for reading digital input and engaging teleop.
 */
void ReadDigitalInputTask(flexiv::tdk::TransparentCartesianTeleopWAN& teleop)
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
                teleop.Engage(0, teleop.digital_inputs(0)[0]);
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
 * @brief Task for calling TransparentCartesianTeleopLAN functions from console.
 */
void ConsoleTask(flexiv::tdk::TransparentCartesianTeleopWAN& teleop)
{
    auto PrintCommandMenu = []() {
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

  --- Help ---
    Any other key to show this help menu
        )" << std::endl;
    };

    unsigned int index = 0;
    const bool is_leader = (teleop.role() == flexiv::tdk::Role::WAN_TELEOP_LEADER);
    flexiv::tdk::AxisLock cmd;
    if (is_leader) {
        teleop.GetAxisLockState(index, cmd);
    }

    auto apply_axis_lock = [&]() {
        if (!is_leader) {
            LogWarn("Axis lock is only available on the leader");
            return;
        }
        teleop.SetAxisLockCmd(index, cmd);
    };

    while (g_running.load() && !teleop.fault(0)) {

        std::string user_input {};

        std::getline(std::cin, user_input);

        if (user_input.empty()) {
            LogWarn("Empty command!");
            PrintCommandMenu();
            continue;
        }

        try {
            switch (user_input[0]) {
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
                    teleop.Engage(0, true);
                    break;
                case 'R':
                    teleop.Engage(0, false);
                    break;
                case 'i':
                    teleop.SetNullSpacePosture(0, kPreferredJntPos);
                    break;
                case 'I':
                    teleop.SetNullSpacePosture(0, kHomeJntPos);
                    break;
                case 'p':
                    teleop.SetMaxContactWrench(0, kDefaultMaxContactWrench);
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
                    const bool ok = teleop.CheckTeleopConnectionLatency(0, latency_ms);
                    if (ok) {
                        LogInfo("pair 0 message latency: " + FmtFixed(latency_ms, 1)
                                + " ms (within limit)");
                    } else if (latency_ms < 0.0) {
                        LogWarn("pair 0 clock mismatch: latency " + FmtFixed(latency_ms, 1) + " ms");
                    } else if (latency_ms > 1.0e12) {
                        LogWarn("pair 0 disconnected: latency " + FmtFixed(latency_ms, 1) + " ms");
                    } else {
                        LogWarn("pair 0 latency over limit: " + FmtFixed(latency_ms, 1) + " ms");
                    }
                    break;
                }
                case 'h':
                    PrintTeleopStatus(0, teleop.GetTeleopStatus(0));
                    break;
                case 'n': {
                    const auto pair = teleop.robot_pair_sn(0);
                    LogInfo(std::string("pair 0 role=")
                            + flexiv::tdk::RoleTypeStr[static_cast<size_t>(teleop.role())]
                            + " leader_sn=" + pair.first + " follower_sn=" + pair.second);
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
    return;
}

int main(int argc, char* argv[])
{
    std::string follower_sn, leader_sn, teleop_role, tcp_role, public_server_ip, lan_ip, wan_iface;
    unsigned int server_port = 0;
    std::vector<std::string> lan_interface_whitelist {};
    std::vector<std::string> wan_interface_whitelist {};
    bool enable_digital_input = false;

    int opt = 0;
    while ((opt = getopt_long_only(argc, argv, "l:f:r:t:i:p:A:W:D", kLongOptions, nullptr)) != -1) {
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
            case 't':
                tcp_role = std::string(optarg);
                break;
            case 'i':
                public_server_ip = std::string(optarg);
                break;
            case 'p':
                try {
                    server_port = std::stoi(optarg);
                } catch (...) {
                    LogError(std::string("Invalid port number: ") + optarg);
                    return 1;
                }
                break;
            case 'A':
                lan_ip = std::string(optarg);
                lan_interface_whitelist.push_back(lan_ip);
                break;
            case 'W':
                wan_iface = std::string(optarg);
                wan_interface_whitelist.push_back(wan_iface);
                break;
            case 'D':
                enable_digital_input = true;
                break;
            default:
                PrintHelp();
                return 1;
        }
    }
    if (follower_sn.empty() || leader_sn.empty() || tcp_role.empty() || teleop_role.empty()
        || public_server_ip.empty() || server_port == 0) {
        PrintHelp();
        return 1;
    }
    if (lan_interface_whitelist.empty()) {
        LogWarn("LAN whitelist is not provided, will search all network interfaces.");
    }
    if (wan_interface_whitelist.empty()) {
        LogWarn(
            "WAN interface whitelist is not provided, will search all network interfaces.");
    }

    // Whether this is a TCP server or client
    bool is_tcp_server;
    if (tcp_role == "server") {
        is_tcp_server = true;
    } else if (tcp_role == "client") {
        is_tcp_server = false;
    } else {
        LogError("Valid inputs for [-t] are: server, client");
        return 1;
    }

    // Whether this is leader or follower
    if (teleop_role == "follower") {
        kRole = flexiv::tdk::Role::WAN_TELEOP_FOLLOWER;
    } else if (teleop_role == "leader") {
        kRole = flexiv::tdk::Role::WAN_TELEOP_LEADER;
    } else {
        LogError("Valid inputs for [-r] are: follower, leader");
        return 1;
    }

    // Network configuration
    flexiv::tdk::NetworkCfgStd network_cfg;
    network_cfg.is_tcp_server = is_tcp_server;
    network_cfg.public_ipv4_address = public_server_ip;
    network_cfg.listening_port = server_port;
    network_cfg.lan_interface_whitelist = lan_interface_whitelist;
    network_cfg.wan_interface_whitelist = wan_interface_whitelist;

    std::vector<std::pair<std::string, std::string>> robot_sn_pairs {};
    robot_sn_pairs.push_back({leader_sn, follower_sn});
    try {

        // Allocate tdk object (Standard Edition TCP peer-to-peer)
        flexiv::tdk::TransparentCartesianTeleopWAN tctw(robot_sn_pairs, kRole, network_cfg);
        const auto pair_sn = tctw.robot_pair_sn(0);
        LogInfo(std::string("This instance role=")
                + flexiv::tdk::RoleTypeStr[static_cast<size_t>(tctw.role())]
                + " leader_sn=" + pair_sn.first + " follower_sn=" + pair_sn.second);

        // Init high transparency teleop
        tctw.Init();

        // Start high transparency teleop
        tctw.Start();

        // Set max contact wrench
        tctw.SetMaxContactWrench(0, kDefaultMaxContactWrench);

        // Start console_thread
        std::thread console_thread(std::bind(ConsoleTask, std::ref(tctw)));

        // Start pedal_thread based on the new flag or the role
        std::optional<std::thread> pedal_thread;
        // Modified condition: Start if -D is given OR if role is leader (original behavior)
        if (teleop_role == "leader" && enable_digital_input) {
            LogInfo(
                "Starting ReadDigitalInputTask thread as role is 'leader' and requested by -D "
                "flag.");
            pedal_thread.emplace(ReadDigitalInputTask, std::ref(tctw));
        } else {
            LogInfo(
                "ReadDigitalInputTask thread NOT started (role is not 'leader' or -D flag not "
                "provided).");
        }

        // Wait for console_thread to finish
        console_thread.join();

        // Stop all threads, notify other threads exit
        g_running = false;

        // Wait for digital reading task exit
        if (pedal_thread && pedal_thread->joinable()) {
            pedal_thread->join();
        }

        // Exit high transparency teleop
        tctw.Stop();

    } catch (const std::exception& e) {
        LogError(e.what());
        return 1;
    }

    return 0;
}