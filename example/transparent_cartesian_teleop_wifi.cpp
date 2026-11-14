/**
 * @example transparent_cartesian_teleop_wifi.cpp
 * @brief Example usage of single- or dual-arm transparent Cartesian teleoperation between two
 * computers on the same WiFi network. Each computer is wired to its own robots (the leader computer
 * to the leader robots, the follower computer to the follower robots) and runs one
 * TransparentCartesianTeleopWAN instance with one or two robot pairs. Only the teleop messages
 * between the two computers travel over WiFi, through one TCP connection (Standard Edition), so no
 * public IP address or port forwarding is needed. A single pedal (DI0 on the first leader robot)
 * engages all pairs. Uses the TDK default Cartesian stiffness and sets no max contact wrench limit.
 * Optionally homes all local robots after connecting. Also supports per-pair restart, message
 * latency monitoring, nullspace posture tuning and fault recovery.
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/tdk/data.hpp>
#include <flexiv/tdk/transparent_cartesian_teleop_wan.hpp>

#include <getopt.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

/** Maximum number of leader-follower robot pairs: 1 for single-arm, 2 for dual-arm teleop */
constexpr unsigned int kMaxPairs = 2;

/** Nullspace to a preferred posture, one per robot pair. Adjust to how each arm is mounted. */
const std::array<std::vector<double>, kMaxPairs> kPreferredJntPos
    = {std::vector<double> {60 * M_PI / 180.0, -60 * M_PI / 180.0, -85 * M_PI / 180.0,
           115 * M_PI / 180.0, 70 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0},
        std::vector<double> {60 * M_PI / 180.0, -60 * M_PI / 180.0, -85 * M_PI / 180.0,
            115 * M_PI / 180.0, 70 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0}};

/** Nullspace to Home posture */
const std::vector<double> kHomeJntPos = {0 * M_PI / 180.0, -40 * M_PI / 180.0, 0 * M_PI / 180.0,
    90 * M_PI / 180.0, 0 * M_PI / 180.0, 40 * M_PI / 180.0, 0 * M_PI / 180.0};

/** Upper bound of the latency limit accepted by SetTeleopLatencyLimit() */
constexpr double kMaxLatencyLimitMs = 350.0;

/** Atomic signal to stop console, fault monitor, link monitor and DI reading tasks */
std::atomic<bool> g_running {true};

/** Whether the link monitor task prints message latency every second */
std::atomic<bool> g_print_latency {false};
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

bool HasActiveIssue(const flexiv::tdk::TeleopStatus& status, flexiv::tdk::TeleopIssueCode code)
{
    return std::any_of(status.issues.begin(), status.issues.end(),
        [code](const flexiv::tdk::TeleopIssue& issue) { return issue.code == code; });
}

/**
 * @brief Whether [ip] is an IPv4 address in a private range (10/8, 172.16/12, 192.168/16), i.e. an
 * address assigned by a local WiFi router.
 */
bool IsPrivateIPv4(const std::string& ip)
{
    unsigned int a = 0, b = 0, c = 0, d = 0;
    char tail = 0;
    if (std::sscanf(ip.c_str(), "%u.%u.%u.%u%c", &a, &b, &c, &d, &tail) != 4) {
        return false;
    }
    if (a > 255 || b > 255 || c > 255 || d > 255) {
        return false;
    }
    return a == 10 || (a == 172 && b >= 16 && b <= 31) || (a == 192 && b == 168);
}

/**
 * @brief Robot pairs a console command applies to: all pairs by default, or the single pair given
 * by an index suffix (e.g. "u1"). Returns empty for an invalid suffix.
 */
std::vector<unsigned int> TargetPairs(const std::string& input, unsigned int num_pairs)
{
    if (input.size() == 1) {
        std::vector<unsigned int> all(num_pairs);
        std::iota(all.begin(), all.end(), 0u);
        return all;
    }
    if (input.size() == 2 && input[1] >= '0' && input[1] < static_cast<char>('0' + num_pairs)) {
        return {static_cast<unsigned int>(input[1] - '0')};
    }
    return {};
}

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

/**
 * @brief Print the message latency of the specified robot pair and why it is out of range, if so.
 */
void PrintLatency(const flexiv::tdk::TransparentCartesianTeleopWAN& teleop, unsigned int idx)
{
    const std::string tag = "pair " + std::to_string(idx);
    double latency_ms {};
    const bool ok = teleop.CheckTeleopConnectionLatency(idx, latency_ms);
    if (ok) {
        LogInfo(tag + " message latency: " + FmtFixed(latency_ms, 1) + " ms (within limit)");
    } else if (HasActiveIssue(teleop.GetTeleopStatus(idx),
                   flexiv::tdk::TeleopIssueCode::NETWORK_DISCONNECTED)) {
        LogWarn(tag + " disconnected from the other computer");
    } else if (latency_ms < 0.0) {
        LogWarn(tag + " clock mismatch: latency " + FmtFixed(latency_ms, 1)
                + " ms. Check 'chronyc tracking' on both computers");
    } else {
        LogWarn(tag + " latency over limit: " + FmtFixed(latency_ms, 1) + " ms");
    }
}

void PrintHelp()
{
    // clang-format off
    std::cout<<"Invalid program arguments!"<<std::endl;
    std::cout<<"     -l     [necessary] Serial number of the 1st leader robot."<<std::endl;
    std::cout<<"     -f     [necessary] Serial number of the 1st follower robot."<<std::endl;
    std::cout<<"     -L     [optional] Serial number of the 2nd leader robot, for dual-arm teleop."<<std::endl;
    std::cout<<"     -F     [optional] Serial number of the 2nd follower robot, for dual-arm teleop."<<std::endl;
    std::cout<<"     -r     [necessary] Role of participants in teleop, can be [leader] or [follower]."<<std::endl;
    std::cout<<"     -t     [necessary] Role in the TCP connection, can be [server] or [client]."<<std::endl;
    std::cout<<"     -i     [necessary] WiFi IPv4 address of the machine that functions as TCP server."<<std::endl;
    std::cout<<"     -p     [necessary] Listening port of the TCP server machine."<<std::endl;
    std::cout<<"     -A     [optional] LAN IPv4 address of the NIC wired to the robots. Repeatable."<<std::endl;
    std::cout<<"     -W     [optional] WiFi interface name, e.g. wlo1. Repeatable."<<std::endl;
    std::cout<<"     -T     [optional] Maximum acceptable message latency in ms, range [0, 350]. Default 200."<<std::endl;
    std::cout<<"     -D     [optional] Leader only. DI0 on the 1st leader robot engages all pairs."<<std::endl;
    std::cout<<"     -H     [optional] Move all local robots to home posture after connecting, before teleop starts."<<std::endl;
    std::cout<<"Pass -L and -F together for dual-arm teleop, or neither for single-arm teleop."<<std::endl;
    std::cout<<"Both computers must pass the same serial numbers in the same order, and the same -i and -p."<<std::endl;
    std::cout<<"Pass -H on both computers to home every arm; each computer homes only its own robots."<<std::endl;
    std::cout<<"Usage: sudo ./transparent_cartesian_teleop_wifi -l <sn> -f <sn> [-L <sn> -F <sn>] -r leader|follower -t server|client -i <server_wifi_ip> -p <port> [-A <lan_ipv4>] [-W <wifi_iface>] [-T <ms>] [-D] [-H]"<<std::endl;
    // clang-format on
}

/**
 * @brief Print the network setup that teleop over a local WiFi network depends on.
 */
void PrintWiFiChecklist(bool is_tcp_server, unsigned int port)
{
    std::cout << R"(
  --- WiFi teleop checklist ---
    1. Both computers are on the same WiFi network with no AP/client isolation, and the WiFi
       subnet does not overlap the robot subnet.
    2. WiFi power saving is off on both computers: sudo iw dev <wifi_iface> set power_save off
    3. Clocks are synchronized: 'chronyc tracking' shows System time < 1 ms. Over local WiFi the
       real latency is only a few ms, so a small clock offset already reads as negative latency
       and forces teleop to disengage.
)";
    if (is_tcp_server) {
        std::cout << "    4. TCP port " << port
                  << " is open in this computer's firewall: sudo ufw allow " << port << "/tcp"
                  << std::endl;
    }
    std::cout << std::endl;
}

const struct option kLongOptions[] = {
    // clang-format off
    {"leader SN",                   required_argument,  0, 'l'},
    {"follower SN",                 required_argument,  0, 'f'},
    {"2nd leader SN",               required_argument,  0, 'L'},
    {"2nd follower SN",             required_argument,  0, 'F'},
    {"role",                        required_argument,  0, 'r'},
    {"tcp role",                    required_argument,  0, 't'},
    {"server wifi ipv4 address",    required_argument,  0, 'i'},
    {"port",                        required_argument,  0, 'p'},
    {"lan whitelist ip",            required_argument,  0, 'A'},
    {"wifi interface name",         required_argument,  0, 'W'},
    {"latency limit ms",            required_argument,  0, 'T'},
    {"enable digital input",        no_argument,        0, 'D'},
    {"home on start",               no_argument,        0, 'H'},
    {0,                             0,                  0,  0 }
    // clang-format on
};

/**
 * @brief Task for reading the pedal (DI0 on the 1st leader robot) and engaging all pairs.
 */
void ReadDigitalInputTask(
    flexiv::tdk::TransparentCartesianTeleopWAN& teleop, unsigned int num_pairs)
{
    std::array<bool, kMaxPairs> logged_idle {};
    std::string last_error;
    while (g_running.load()) {
        try {
            const bool pedal = teleop.digital_inputs(0)[0];
            for (unsigned int idx = 0; idx < num_pairs; ++idx) {
                const auto status = teleop.GetTeleopStatus(idx);
                // Robot fault / Stop() drops the pair back to not-started. Engage() is invalid in
                // that state and must not be polled every cycle.
                if (!status.started || status.stopped) {
                    if (!logged_idle[idx]) {
                        LogWarn("ReadDigitalInputTask: pair " + std::to_string(idx)
                                + " is not started, pause Engage (press 'u" + std::to_string(idx)
                                + "' to resume)");
                        logged_idle[idx] = true;
                    }
                    continue;
                }
                logged_idle[idx] = false;
                teleop.Engage(idx, pedal);
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
 * @brief Task for monitoring robot fault of all pairs and prompting the operator to recover.
 */
void FaultMonitorTask(flexiv::tdk::TransparentCartesianTeleopWAN& teleop, unsigned int num_pairs)
{
    std::array<bool, kMaxPairs> last_fault {};
    std::string last_error;
    while (g_running.load()) {
        try {
            for (unsigned int idx = 0; idx < num_pairs; ++idx) {
                const bool fault = teleop.fault(idx);
                if (fault == last_fault[idx]) {
                    continue;
                }
                last_fault[idx] = fault;
                const std::string i = std::to_string(idx);
                if (fault) {
                    LogError("Fault detected on pair " + i + ". Press 'h" + i
                             + "' for details, 'F' to clear fault, then 'u" + i
                             + "' to restart the pair.");
                    PrintTeleopStatus(idx, teleop.GetTeleopStatus(idx));
                } else {
                    LogInfo(
                        "Fault cleared on pair " + i + ". Press 'u" + i + "' to restart the pair.");
                }
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
}

/**
 * @brief Task for printing the message latency of all pairs every second while enabled with 'm'.
 */
void LinkMonitorTask(
    const flexiv::tdk::TransparentCartesianTeleopWAN& teleop, unsigned int num_pairs)
{
    std::string last_error;
    while (g_running.load()) {
        if (g_print_latency.load()) {
            try {
                for (unsigned int idx = 0; idx < num_pairs; ++idx) {
                    PrintLatency(teleop, idx);
                }
                last_error.clear();
            } catch (const std::exception& e) {
                if (last_error != e.what()) {
                    LogError(std::string("Exception in LinkMonitorTask: ") + e.what());
                    last_error = e.what();
                }
            }
        }
        // Sleep 1 s in short steps so the task exits promptly
        for (int i = 0; i < 10 && g_running.load(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    LogInfo("LinkMonitorTask exiting.");
}

/**
 * @brief Task for calling TransparentCartesianTeleopWAN functions from console.
 */
void ConsoleTask(flexiv::tdk::TransparentCartesianTeleopWAN& teleop, unsigned int num_pairs)
{
    auto PrintCommandMenu = [num_pairs]() {
        if (num_pairs > 1) {
            std::cout << R"(
  Commands act on all pairs. Append a pair index to act on one pair only, e.g. "u1".)";
        }
        std::cout << R"(
  --- Teleop Engagement (leader only) ---
    r        : Engage teleop
    R        : Disengage teleop

  --- Null Space Posture ---
    i/I      : Set local nullspace to Preferred/Home posture

  --- Start/Stop ---
    u        : Init and start teleop
    U        : Stop teleop
    H        : Move all local robots to home posture (stop teleop first)

  --- Message latency ---
    l        : Print current message latency (disconnected / clock mismatch / over limit)
    m        : Toggle printing message latency every second

  --- Teleop status ---
    h        : Print why teleop is restricted / paused and what to do next

  --- Fault recovery ---
    F        : Clear fault on all local robots (blocking, up to 30 s), then press 'u' to restart

  --- Identity ---
    n        : Print role() and robot_pair_sn()

  --- Exit ---
    quit     : Stop teleop and exit the program

  --- Help ---
    Any other key to show this help menu
        )" << std::endl;
    };

    while (g_running.load()) {

        std::string user_input {};

        // Get user input, exit when stdin is closed
        if (!std::getline(std::cin, user_input)) {
            LogInfo("Console input closed.");
            break;
        }

        if (user_input.empty()) {
            LogWarn("Empty command!");
            PrintCommandMenu();
            continue;
        }

        if (user_input == "quit") {
            LogInfo("Quit requested.");
            break;
        }

        const auto targets = TargetPairs(user_input, num_pairs);
        if (targets.empty()) {
            LogWarn(num_pairs == 1
                        ? "Invalid command! Single-arm teleop takes no pair index."
                        : "Invalid command! Valid pair index: 0-" + std::to_string(num_pairs - 1));
            PrintCommandMenu();
            continue;
        }
        const bool all_pairs = (targets.size() == num_pairs);

        try {
            switch (user_input[0]) {
                case 'r':
                    for (auto idx : targets) {
                        teleop.Engage(idx, true);
                    }
                    break;
                case 'R':
                    for (auto idx : targets) {
                        teleop.Engage(idx, false);
                    }
                    break;
                case 'i':
                    for (auto idx : targets) {
                        teleop.SetNullSpacePosture(idx, kPreferredJntPos[idx]);
                    }
                    break;
                case 'I':
                    for (auto idx : targets) {
                        teleop.SetNullSpacePosture(idx, kHomeJntPos);
                    }
                    break;
                case 'u':
                    if (all_pairs) {
                        teleop.Init();
                        teleop.Start();
                    } else {
                        teleop.InitWithIdx(targets[0]);
                        teleop.StartWithIdx(targets[0]);
                    }
                    // Engagement survives Stop/Init/Start, so restart disengaged
                    if (teleop.role() == flexiv::tdk::Role::WAN_TELEOP_LEADER) {
                        for (auto idx : targets) {
                            teleop.Engage(idx, false);
                        }
                    }
                    break;
                case 'U':
                    if (all_pairs) {
                        teleop.Stop();
                    } else {
                        teleop.StopWithIdx(targets[0]);
                    }
                    break;
                case 'H':
                    if (!all_pairs) {
                        LogWarn("'H' moves all local robots home; it takes no pair index");
                        break;
                    }
                    LogInfo("Moving all local robots to home posture...");
                    teleop.HomeAll();
                    LogInfo("All local robots are at home posture. Press 'u' to restart teleop.");
                    break;
                case 'l':
                    for (auto idx : targets) {
                        PrintLatency(teleop, idx);
                    }
                    break;
                case 'm':
                    g_print_latency = !g_print_latency.load();
                    LogInfo(std::string("Periodic latency print ")
                            + (g_print_latency.load() ? "enabled" : "disabled"));
                    break;
                case 'h':
                    for (auto idx : targets) {
                        PrintTeleopStatus(idx, teleop.GetTeleopStatus(idx));
                    }
                    break;
                case 'F': {
                    LogInfo("Clearing fault, this may take up to 30 seconds...");
                    const auto results = teleop.ClearFault();
                    for (size_t i = 0; i < results.size(); ++i) {
                        LogInfo("Teleop pair " + std::to_string(i)
                                + " clear fault: " + BoolStr(results[i]));
                    }
                    break;
                }
                case 'n':
                    for (auto idx : targets) {
                        const auto pair = teleop.robot_pair_sn(idx);
                        LogInfo("pair " + std::to_string(idx) + " role="
                                + flexiv::tdk::RoleTypeStr[static_cast<size_t>(teleop.role())]
                                + " leader_sn=" + pair.first + " follower_sn=" + pair.second);
                    }
                    break;

                default:
                    LogWarn("Invalid command!");
                    PrintCommandMenu();
                    break;
            }
        } catch (const std::exception& e) {
            LogError(std::string("Command '") + user_input + "' failed: " + e.what());
            try {
                for (auto idx : targets) {
                    PrintTeleopStatus(idx, teleop.GetTeleopStatus(idx));
                }
            } catch (const std::exception& status_e) {
                LogError(std::string("Failed to query teleop status: ") + status_e.what());
            }
        }
    }
    LogInfo("Console thread exiting.");
}

int main(int argc, char* argv[])
{
    std::array<std::string, kMaxPairs> leader_sn {};
    std::array<std::string, kMaxPairs> follower_sn {};
    std::string teleop_role, tcp_role, server_ip;
    unsigned int server_port = 0;
    std::optional<double> latency_limit_ms;
    std::vector<std::string> lan_interface_whitelist {};
    std::vector<std::string> wan_interface_whitelist {};
    bool enable_digital_input = false;
    bool home_on_start = false;

    int opt = 0;
    while ((opt = getopt_long_only(argc, argv, "l:f:L:F:r:t:i:p:A:W:T:DH", kLongOptions, nullptr))
           != -1) {
        switch (opt) {
            case 'l':
                leader_sn[0] = std::string(optarg);
                break;
            case 'f':
                follower_sn[0] = std::string(optarg);
                break;
            case 'L':
                leader_sn[1] = std::string(optarg);
                break;
            case 'F':
                follower_sn[1] = std::string(optarg);
                break;
            case 'r':
                teleop_role = std::string(optarg);
                break;
            case 't':
                tcp_role = std::string(optarg);
                break;
            case 'i':
                server_ip = std::string(optarg);
                break;
            case 'p': {
                const std::string port_str(optarg);
                unsigned long port = 0;
                size_t parsed = 0;
                try {
                    port = std::stoul(port_str, &parsed);
                } catch (...) {
                    parsed = 0;
                }
                if (port_str.empty() || port_str[0] == '-' || parsed != port_str.size() || port < 1
                    || port > 65535) {
                    LogError("Invalid port number: " + port_str + ", valid range is 1-65535");
                    return 1;
                }
                server_port = static_cast<unsigned int>(port);
                break;
            }
            case 'A':
                lan_interface_whitelist.push_back(std::string(optarg));
                break;
            case 'W':
                wan_interface_whitelist.push_back(std::string(optarg));
                break;
            case 'T': {
                const std::string limit_str(optarg);
                double limit = -1.0;
                size_t parsed = 0;
                try {
                    limit = std::stod(limit_str, &parsed);
                } catch (...) {
                    parsed = 0;
                }
                if (limit_str.empty() || parsed != limit_str.size()
                    || !(limit >= 0.0 && limit <= kMaxLatencyLimitMs)) {
                    LogError("Invalid latency limit: " + limit_str + " ms, valid range is 0-"
                             + FmtFixed(kMaxLatencyLimitMs, 0));
                    return 1;
                }
                latency_limit_ms = limit;
                break;
            }
            case 'D':
                enable_digital_input = true;
                break;
            case 'H':
                home_on_start = true;
                break;
            default:
                PrintHelp();
                return 1;
        }
    }
    if (leader_sn[0].empty() || follower_sn[0].empty() || teleop_role.empty() || tcp_role.empty()
        || server_ip.empty() || server_port == 0) {
        PrintHelp();
        return 1;
    }

    // Single-arm teleop with one pair, or dual-arm teleop when the 2nd pair is given
    if (leader_sn[1].empty() != follower_sn[1].empty()) {
        LogError("[-L] and [-F] must be given together for dual-arm teleop");
        return 1;
    }
    const unsigned int num_pairs = leader_sn[1].empty() ? 1 : 2;

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
    flexiv::tdk::Role role;
    if (teleop_role == "follower") {
        role = flexiv::tdk::Role::WAN_TELEOP_FOLLOWER;
    } else if (teleop_role == "leader") {
        role = flexiv::tdk::Role::WAN_TELEOP_LEADER;
    } else {
        LogError("Valid inputs for [-r] are: follower, leader");
        return 1;
    }

    // Network sanity checks, warnings only
    if (!IsPrivateIPv4(server_ip)) {
        LogWarn("[-i] " + server_ip
                + " is not a private IPv4 address. For teleop over a local WiFi network, use the "
                  "WiFi IPv4 address of the TCP server computer (see 'ip -4 addr show "
                  "<wifi_iface>').");
    }
    if (lan_interface_whitelist.empty()) {
        LogWarn(
            "LAN whitelist [-A] is not provided, RDK will search all network interfaces "
            "including WiFi. Pass the IPv4 address of the NIC wired to the robots.");
    }
    if (wan_interface_whitelist.empty()) {
        LogWarn(
            "WiFi interface [-W] is not provided, the teleop link may use any network "
            "interface. Pass the WiFi interface name, e.g. wlo1.");
    }
    if (enable_digital_input && role != flexiv::tdk::Role::WAN_TELEOP_LEADER) {
        LogWarn("[-D] only applies to the leader and is ignored.");
    }
    PrintWiFiChecklist(is_tcp_server, server_port);

    // Network configuration: the TCP server's WiFi address replaces the public IPv4 address
    flexiv::tdk::NetworkCfgStd network_cfg;
    network_cfg.is_tcp_server = is_tcp_server;
    network_cfg.public_ipv4_address = server_ip;
    network_cfg.listening_port = server_port;
    network_cfg.lan_interface_whitelist = lan_interface_whitelist;
    network_cfg.wan_interface_whitelist = wan_interface_whitelist;

    std::vector<std::pair<std::string, std::string>> robot_sn_pairs {};
    for (unsigned int idx = 0; idx < num_pairs; ++idx) {
        robot_sn_pairs.push_back({leader_sn[idx], follower_sn[idx]});
    }
    LogInfo(num_pairs == 1 ? "Single-arm teleop" : "Dual-arm teleop");

    try {
        // Allocate tdk object with all robot pairs (Standard Edition TCP peer-to-peer)
        flexiv::tdk::TransparentCartesianTeleopWAN tctw(robot_sn_pairs, role, network_cfg);
        for (unsigned int idx = 0; idx < num_pairs; ++idx) {
            const auto pair_sn = tctw.robot_pair_sn(idx);
            LogInfo("Pair " + std::to_string(idx)
                    + " role=" + flexiv::tdk::RoleTypeStr[static_cast<size_t>(tctw.role())]
                    + " leader_sn=" + pair_sn.first + " follower_sn=" + pair_sn.second);
        }

        // Set message latency limit if specified
        if (latency_limit_ms) {
            for (unsigned int idx = 0; idx < num_pairs; ++idx) {
                tctw.SetTeleopLatencyLimit(idx, *latency_limit_ms);
            }
            LogInfo("Message latency limit set to " + FmtFixed(*latency_limit_ms, 1) + " ms");
        }

        // Optionally move all local robots to home posture first. HomeAll() needs enabled robots,
        // so enable them with an Init() that skips sensor zeroing; the Init() below then zeroes the
        // force/torque sensors at the home posture and switches back to teleop control.
        if (home_on_start) {
            LogWarn("Moving all local robots to home posture, make sure the workspace is clear");
            tctw.Init(true, flexiv::tdk::ZeroFTSensor::Disable);
            tctw.HomeAll();
            LogInfo("All local robots are at home posture");
        }

        // Init high transparency teleop for all pairs
        tctw.Init();

        // Start high transparency teleop for all pairs
        tctw.Start();

        // Start console, fault monitor and link monitor threads
        std::thread console_thread(ConsoleTask, std::ref(tctw), num_pairs);
        std::thread fault_monitor_thread(FaultMonitorTask, std::ref(tctw), num_pairs);
        std::thread link_monitor_thread(LinkMonitorTask, std::cref(tctw), num_pairs);

        // Start pedal_thread only if role is leader AND -D is given
        std::optional<std::thread> pedal_thread;
        if (role == flexiv::tdk::Role::WAN_TELEOP_LEADER && enable_digital_input) {
            LogInfo(
                "Starting ReadDigitalInputTask: DI0 on the 1st leader robot engages all "
                "pairs.");
            pedal_thread.emplace(ReadDigitalInputTask, std::ref(tctw), num_pairs);
        } else {
            LogInfo(
                "ReadDigitalInputTask NOT started (role is not 'leader' or -D flag not "
                "provided).");
        }

        // Wait for console_thread to finish
        console_thread.join();

        // Stop all threads, notify other threads exit
        g_running = false;

        // Wait for monitor and digital reading tasks to exit
        fault_monitor_thread.join();
        link_monitor_thread.join();
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
