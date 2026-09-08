/**
 * @example transparent_cartesian_teleop_wan.cpp
 * @brief Example usage of Transparent Cartesian teleoperation cross Wide Area Network for
 * controlling a follower robot using a leader robot with transparent force feedback. Supports both
 * keyboard and digital input engage/disengage signal reading, with message latency query, nullspace
 * posture tuning, and max contact wrench setting, etc.
 * @note This program is provided only as an example. Users must adapt it to their own application
 * requirements, safety procedures, and software architecture before deployment.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
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

/** Single-arm joint group controlled by this example */
constexpr auto kJointGroup = flexiv::rdk::JointGroup::ARM_1;

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
    std::cout<<"Usage: sudo ./transparent_cartesian_teleop_wan [-l leader_robot_serial_number] [-f follower_robot_serial_number] [-r leader/follower] [-t server/client] [-i server_public_ip] [-p server_port] [-A lan_interface_ip] [-W wan_interface_ip] [-D]"<<std::endl;
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
    while (g_running.load() && !teleop.fault(0)) {
        try {
            teleop.Engage(0, kJointGroup, teleop.digital_inputs(0)[0]);
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
    l       : print current message latency in milliseconds

  --- Help ---
    Any other key to show this help menu
        )" << std::endl;
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
                case 'r':
                    teleop.Engage(0, kJointGroup, true);
                    break;
                case 'R':
                    teleop.Engage(0, kJointGroup, false);
                    break;
                case 'i':
                    teleop.SetNullSpacePosture(0, kJointGroup, kPreferredJntPos);
                    break;
                case 'I':
                    teleop.SetNullSpacePosture(0, kJointGroup, kHomeJntPos);
                    break;
                case 'p':
                    teleop.SetMaxContactWrench(0, kJointGroup, kDefaultMaxContactWrench);
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
                    if (teleop.CheckTeleopConnectionLatency(0, latency_ms)) {
                        LogInfo("Current message latency is: " + std::to_string(latency_ms) + "ms");
                    } else {
                        LogWarn("WAN teleop is disconnected.");
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
    return;
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
    network_cfg.wan_interface_whitelist = wan_interface_whitelist;

    std::vector<std::pair<std::string, std::string>> robot_sn_pairs {};
    robot_sn_pairs.push_back({leader_sn, follower_sn});
    try {

        // Allocate tdk object
        flexiv::tdk::TransparentCartesianTeleopWAN tctw(robot_sn_pairs, kRole, network_cfg);

        // Init high transparency teleop
        tctw.Init();

        // Start high transparency teleop
        tctw.Start();

        // Set max contact wrench
        tctw.SetMaxContactWrench(0, kJointGroup, kDefaultMaxContactWrench);

        // Start console_thread
        std::thread console_thread(std::bind(ConsoleTask, std::ref(tctw)));

        // Start pedal_thread based on the new flag or the role
        std::optional<std::thread> pedal_thread;
        // Modified condition: Start if -D is given OR if role is leader (original behavior)
        if (teleop_role == "leader" && enable_digital_input) {
            LogInfo("Starting ReadDigitalInputTask thread as role is 'leader' and requested by -D "
                    "flag.");
            pedal_thread.emplace(ReadDigitalInputTask, std::ref(tctw));
        } else {
            LogInfo("ReadDigitalInputTask thread NOT started (role is not 'leader' or -D flag not "
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