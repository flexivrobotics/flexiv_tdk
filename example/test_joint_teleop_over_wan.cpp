/**
 * @file test_joint_teleop_over_wan.cpp
 * @date 2024-10-15
 */

#include <flexiv/tdk/joint_teleop_wan.hpp>

#include <spdlog/spdlog.h>
#include <getopt.h>
#include <iostream>
#include <thread>

namespace {
const struct option kLongOptions[] = {{"serial-number", required_argument, 0, 's'},
    {"ip", required_argument, 0, 'i'}, {"port", required_argument, 0, 'p'}, {0, 0, 0, 0}};
}

void PrintHelp()
{
    // clang-format off
    std::cout << "Usage: ./test_joint_teleop_over_wan [-s serial_num] [-r server/client] [-i ip] [-p port]" << std::endl;
    std::cout << "  -s  --serial-number    Serial number of the local robot." << std::endl;
    std::cout << "  -r  --tcp-role         Role in the TCP connection, [server] or [client]." << std::endl;
    std::cout << "  -i  --ip               Public IPv4 address of the TCP server machine." << std::endl;
    std::cout << "  -p  --port             Listening port of the TCP server machine." << std::endl;
    // clang-format on
}

int main(int argc, char* argv[])
{
    std::string local_sn, tcp_role, server_ip;
    unsigned int server_port = 0;
    int opt = 0;
    while ((opt = getopt_long(argc, argv, "s:r:i:p:", kLongOptions, nullptr)) != -1) {
        switch (opt) {
            case 's':
                local_sn = std::string(optarg);
                break;
            case 'r':
                tcp_role = std::string(optarg);
                break;
            case 'i':
                server_ip = std::string(optarg);
                break;
            case 'p':
                server_port = std::stoi(optarg);
                break;
            default:
                PrintHelp();
                return 1;
        }
    }
    if (local_sn.empty() || tcp_role.empty() || server_ip.empty() || server_port == 0) {
        PrintHelp();
        return 1;
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

    try {
        // Instantiate robot node
        flexiv::tdk::JointTeleopWAN joint_teleop(local_sn, is_tcp_server, server_ip, server_port);

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

        // Start control loop
        joint_teleop.Start();

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
