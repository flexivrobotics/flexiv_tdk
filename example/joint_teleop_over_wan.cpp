/**
 * @example joint_teleop_over_wan.cpp
 * @brief joint-space robot-robot teleoperation over WAN (Wide Area Network, i.e. Internet)
 * connection.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/tdk/joint_teleop_wan.hpp>

#include <spdlog/spdlog.h>

#include <getopt.h>
#include <iostream>

namespace {
const std::vector<double> kJointStiffnessRatio = {0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02};
constexpr double kLastJointShapedInertia = 0.05;
}

void PrintHelp()
{
    // clang-format off
    std::cout << "Usage: sudo ./test_joint_teleop_over_wan [-s serial_num] [-r server/client] [-i ip] [-p port]" << std::endl;
    std::cout << "  -s  --serial-number    Serial number of the local robot." << std::endl;
    std::cout << "  -r  --tcp-role         Role in the TCP connection, [server] or [client]." << std::endl;
    std::cout << "  -i  --ip               Public IPv4 address of the TCP server machine." << std::endl;
    std::cout << "  -p  --port             Listening port of the TCP server machine." << std::endl;
    // clang-format on
}

const struct option kLongOptions[] = {
    // clang-format off
    {"serial number",               required_argument,  0, 's'},
    {"tcp role",                    required_argument,  0, 'r'},
    {"public ipv4 address",         required_argument,  0, 'i'},
    {"port",                        required_argument,  0, 'p'},
    {"lan whitelist ip",            optional_argument,  0, 'l'},
    {"wan whitelist ip",            optional_argument,  0, 'w'},
    {0,                             0,                  0,  0 }
    // clang-format on
};

int main(int argc, char* argv[])
{
    std::string local_sn, tcp_role, public_server_ip, lan_ip, wan_ip;
    unsigned int server_port = 0;
    std::vector<std::string> lan_interface_whitelist {};
    std::vector<std::string> wan_interface_whitelist {};

    int opt = 0;
    while ((opt = getopt_long_only(argc, argv, "s:r:i:p:l:w:", kLongOptions, nullptr)) != -1) {
        switch (opt) {
            case 's':
                local_sn = std::string(optarg);
                break;
            case 'r':
                tcp_role = std::string(optarg);
                break;
            case 'i':
                public_server_ip = std::string(optarg);
                break;
            case 'p':
                try {
                    server_port = std::stoi(optarg);
                } catch (...) {
                    spdlog::error("Invalid port number: {}", optarg);
                    return 1;
                }
                break;
            case 'l':
                lan_ip = std::string(optarg);
                lan_interface_whitelist.push_back(lan_ip);
                break;
            case 'w':
                wan_ip = std::string(optarg);
                wan_interface_whitelist.push_back(wan_ip);
                break;
            default:
                PrintHelp();
                return 1;
        }
    }
    if (local_sn.empty() || tcp_role.empty() || public_server_ip.empty() || server_port == 0) {
        PrintHelp();
        return 1;
    }
    if (lan_interface_whitelist.empty() || wan_interface_whitelist.empty()) {
        spdlog::warn("LAN or WAN whitelist is not provided, will search all network interfaces.");
    }

    // Whether this is a TCP server or client
    bool is_tcp_server;
    if (tcp_role == "server") {
        is_tcp_server = true;
    } else if (tcp_role == "client") {
        is_tcp_server = false;
    } else {
        spdlog::error("Valid inputs for [-r] are: server, client");
        return 1;
    }

    // Network configuration
    flexiv::tdk::NetworkCfg network_cfg;
    network_cfg.is_tcp_server = is_tcp_server;
    network_cfg.public_ipv4_address = public_server_ip;
    network_cfg.listening_port = server_port;
    network_cfg.lan_interface_whitelist = lan_interface_whitelist;
    network_cfg.wan_interface_whitelist = wan_interface_whitelist;

    spdlog::info("Robot serial number: {}", local_sn);
    spdlog::info("Start as {} on {}:{}", tcp_role, public_server_ip, server_port);

    try {
        // Instantiate robot node
        flexiv::tdk::JointTeleopWAN joint_teleop(local_sn, network_cfg);

        // Run initialization sequence
        joint_teleop.Init();

        if (is_tcp_server) {
            // Set 20 degrees soft limit for only one side of teleoperation
            joint_teleop.SetSoftLimit(20.0);

            // Server stays at current pose
            joint_teleop.SyncPose(false, {});
        } else {
            // Client syncs pose with server
            joint_teleop.SyncPose(true);
        }

        // Enable inertia shaping for the last joint
        std::vector<std::pair<bool, double>> shaped_joint_inertia;
        shaped_joint_inertia.resize(joint_teleop.DoF(), {false, 1.0});
        shaped_joint_inertia.back() = {true, kLastJointShapedInertia};
        joint_teleop.SetInertiaShaping(shaped_joint_inertia);

        // Start control loop
        joint_teleop.Start();

        // Set joint impedance
        joint_teleop.SetJointImpedance(kJointStiffnessRatio);

        // Block until faulted
        bool last_pedal_input = false;
        while (!joint_teleop.fault()) {
            // Server is activated by pedal, client will auto sync the activation
            if (is_tcp_server) {
                bool pedal_input = joint_teleop.digital_inputs()[0];
                if (pedal_input != last_pedal_input) {
                    joint_teleop.Activate(pedal_input);
                    last_pedal_input = pedal_input;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Fault occurred, stop the teleoperation
        joint_teleop.Stop();

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}
