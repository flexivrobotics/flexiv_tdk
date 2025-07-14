/**
 * @example transparent_cartesian_teleop_lan.cpp
 * @brief Example of using TransparentCartesianTeleopLAN class.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/tdk/data.hpp>
#include <flexiv/tdk/transparent_cartesian_teleop_lan.hpp>

#include <spdlog/spdlog.h>

#include <getopt.h>
#include <atomic>
#include <chrono>
#include <iostream>
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
    = {5.0, 5.0, 5.0, 40.0, 40.0, 40.0};

/** Atomic signal to stop console and DI reading tasks */
std::atomic<bool> g_running {true};

} // namespace

void PrintHelp()
{
    // clang-format off
    std::cout<<"Invalid program arguments!"<<std::endl;
    std::cout<<"     -l     [necessary] serial number of leader robot."<<std::endl;
    std::cout<<"     -f     [necessary] serial number of follower robot."<<std::endl;
    std::cout<<"     -i     [optional] The ip address of the network card connected to the robot." << std::endl;
    std::cout<<"Usage: sudo ./transparent_cartesian_teleop_lan [-l leader_robot_sn] [-f follower_robot_sn] [-i white_list_ip_of_network_interface]"<<std::endl;
    // clang-format on
}

const struct option kLongOptions[] = {
    // clang-format off
    {"leader SN",                                       required_argument,  0, 'l'},
    {"follower SN",                                     required_argument,  0, 'f'},
    {"ip address of whitelisted network interface",     optional_argument,  0, 'i'},
    {0,                                                                 0,  0,  0 }
    // clang-format on
};

/**
 * @brief Task for monitoring DI signals and engaging teleop.
 */
void ReadDigitalInputTask(flexiv::tdk::TransparentCartesianTeleopLAN& teleop)
{
    while (g_running.load() && !teleop.any_fault()) {
        try {
            teleop.Engage(0, teleop.digital_inputs(0).first[0]);
        } catch (const std::exception& e) {
            spdlog::error("Exception in ReadDigitalInputTask: {}", e.what());
            g_running.store(false);
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    spdlog::info("ReadDigitalInputTask exiting.");
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
    A        : Clear repulsive force

  --- Help ---
    Any other key to show this help menu
        )" << std::endl;
    };

    unsigned int index = 0;
    flexiv::tdk::AxisLock cmd;
    teleop.GetAxisLockState(index, cmd);

    while (g_running.load() && !teleop.any_fault()) {

        std::string userInput {};

        // Get user input
        std::getline(std::cin, userInput);

        if (userInput.empty()) {
            spdlog::warn("Empty command!");
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
                    teleop.SetRepulsiveForce(index, {0, 0, 0});
                    break;

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
    while ((opt = getopt_long_only(argc, argv, "f:l:i:", kLongOptions, &longIndex)) != -1) {
        switch (opt) {
            case 'f':
                follower_sn = std::string(optarg);
                break;
            case 'l':
                leader_sn = std::string(optarg);
                break;
            case 'i':
                whitelist_ip = std::string(optarg);
                network_interface_whitelist.emplace_back(whitelist_ip);
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
        spdlog::warn(
            "network_interface_whitelist is not provided, will search all network interfaces.");
    }

    try {

        // Create teleop control interface
        flexiv::tdk::TransparentCartesianTeleopLAN tctl(
            {{leader_sn, follower_sn}}, network_interface_whitelist);

        // Run initialization sequence
        tctl.Init();

        // Start control loop
        tctl.Start();

        // Start console and pedal input threads
        std::thread console_thread(std::bind(ConsoleTask, std::ref(tctl)));
        std::thread pedal_thread(std::bind(ReadDigitalInputTask, std::ref(tctl)));

        // Wait for threads to finish
        console_thread.join();
        pedal_thread.join();

        // Exit high transparency teleop
        tctl.Stop();

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}
