/**
 * @example transparent_cartesian_teleop_wan.cpp
 * @brief Example usage of Transparent Cartesian teleoperation over WAN (TDK Standard Edition,
 * peer-to-peer TCP). Controls a follower robot from a leader robot with transparent force
 * feedback. Supports keyboard and digital input engage/disengage, message latency query, nullspace
 * posture tuning, max contact wrench setting, and teleop status query.
 * @note This program is provided only as an example. Users must adapt it to their own application
 * requirements, safety procedures, and software architecture before deployment.
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
#include <functional>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <utility>
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
    = {50.0, 50.0, 50.0, 40.0, 40.0, 40.0};

/** Robot pair index used by this example */
constexpr unsigned int kIdx = 0;

/** Single-arm joint group controlled by this example */
constexpr auto kJointGroup = flexiv::rdk::JointGroup::ARM_1;

/** Atomic signal to stop console and DI reading tasks */
std::atomic<bool> g_running {true};

/** Teleop role, assigned from [-r] */
flexiv::tdk::Role g_role = flexiv::tdk::Role::UNKNOWN;

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
            + " motion_restricted=" + std::to_string(status.motion_restricted) + " latency="
            + std::to_string(status.latency_ms) + "/" + std::to_string(status.latency_threshold_ms)
            + " ms");
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

} // namespace

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
    std::cout<<"     -W     [optional]  OS-level name(s) of the network interface(s) that connect to the internet." << std::endl;
    std::cout<<"     -D     [optional] Enable Digital Input reading task." << std::endl;
    std::cout<<"Usage: sudo ./transparent_cartesian_teleop_wan [-l leader_robot_serial_number] [-f follower_robot_serial_number] [-r leader/follower] [-t server/client] [-i server_public_ip] [-p server_port] [-W wan_interface] [-D]"<<std::endl;
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
    {"wan whitelist NIC name",      optional_argument,  0, 'W'},
    {"enable digital input",        no_argument,        0, 'D'},
    {0,                             0,                  0,  0 }
    // clang-format on
};

/**
 * @brief Task for reading digital input and engaging teleop.
 */
void ReadDigitalInputTask(flexiv::tdk::TransparentCartesianTeleopWAN& teleop)
{
    while (g_running.load() && !teleop.fault(kIdx)) {
        try {
            teleop.Engage(kIdx, kJointGroup, teleop.digital_inputs(kIdx)[0]);
        } catch (const std::exception& e) {
            LogError(std::string("Exception in ReadDigitalInputTask: ") + e.what());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    LogInfo("ReadDigitalInputTask exiting.");
}

/**
 * @brief Task for calling TransparentCartesianTeleopWAN functions from console.
 */
void ConsoleTask(flexiv::tdk::TransparentCartesianTeleopWAN& teleop)
{
    auto PrintCommandMenu = []() {
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
            switch (user_input[0]) {
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
    std::string follower_sn, leader_sn, teleop_role, tcp_role, public_server_ip, nic_name;
    unsigned int server_port = 0;
    std::vector<std::string> wan_interface_whitelist {};
    bool enable_digital_input = false;

    int opt = 0;
    while ((opt = getopt_long_only(argc, argv, "l:f:r:t:i:p:W:D", kLongOptions, nullptr)) != -1) {
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
            case 'W':
                nic_name = std::string(optarg);
                wan_interface_whitelist.push_back(nic_name);
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
    if (wan_interface_whitelist.empty()) {
        LogWarn("WAN interface whitelist is not provided, will search all network interfaces.");
    }

    bool is_tcp_server = false;
    if (tcp_role == "server") {
        is_tcp_server = true;
    } else if (tcp_role == "client") {
        is_tcp_server = false;
    } else {
        LogError("Valid inputs for [-t] are: server, client");
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

    flexiv::tdk::NetworkCfgStd network_cfg;
    network_cfg.is_tcp_server = is_tcp_server;
    network_cfg.public_ipv4_address = public_server_ip;
    network_cfg.listening_port = server_port;
    network_cfg.wan_interface_whitelist = wan_interface_whitelist;

    std::vector<std::pair<std::string, std::string>> robot_sn_pairs {};
    robot_sn_pairs.push_back({leader_sn, follower_sn});
    try {
        flexiv::tdk::TransparentCartesianTeleopWAN tctw(robot_sn_pairs, g_role, network_cfg);

        const auto pair_sn = tctw.robot_pair_sn(kIdx);
        LogInfo(std::string("role=") + flexiv::tdk::RoleTypeStr[static_cast<size_t>(tctw.role())]
                + " robot_pair_sn=(" + pair_sn.first + ", " + pair_sn.second + ")");

        tctw.Init();
        tctw.Start();
        tctw.SetMaxContactWrench(kIdx, kJointGroup, kDefaultMaxContactWrench);

        std::thread console_thread(std::bind(ConsoleTask, std::ref(tctw)));

        std::optional<std::thread> pedal_thread;
        if (g_role == flexiv::tdk::Role::WAN_TELEOP_LEADER && enable_digital_input) {
            LogInfo("Starting ReadDigitalInputTask thread as role is 'leader' and requested by -D "
                    "flag.");
            pedal_thread.emplace(ReadDigitalInputTask, std::ref(tctw));
        } else {
            LogInfo("ReadDigitalInputTask thread NOT started (role is not 'leader' or -D flag not "
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
