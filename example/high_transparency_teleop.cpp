/**
 * @file high_transparency_teleop.cpp
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 */

// Flexiv
#include <atomic>
#include <chrono>
#include <spdlog/spdlog.h>
#include <flexiv/tdk/transparent_cartesian_teleop_lan.hpp>
#include <flexiv/tdk/data.hpp>
#include <getopt.h>
#include <iostream>
#include <thread>
namespace {
std::vector<double> kPreferredJntPos
    = {60 * M_PI / 180.0, -60 * M_PI / 180.0, -85 * M_PI / 180.0, 115 * M_PI / 180.0,
        70 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0}; ///< Preferred joint position
std::vector<double> kHomeJntPos
    = {0 * M_PI / 180.0, -40 * M_PI / 180.0, 0 * M_PI / 180.0, 90 * M_PI / 180.0, 0 * M_PI / 180.0,
        40 * M_PI / 180.0, 0 * M_PI / 180.0}; ///< Preferred joint position

const std::array<double, flexiv::tdk::kCartDoF> kDefaultMaxContactWrench
    = {5.0, 5.0, 5.0, 40.0, 40.0, 40.0}; ///< Maximum contact wrench

std::atomic<bool> g_stop_sched = {false}; ///< Atomic signal to stop scheduler tasks
} // namespace

void printHelp()
{
    // clang-format off
    spdlog::error("Invalid program arguments");
    spdlog::info("     -l     [necessary] serial number of local robot.");
    spdlog::info("     -r     [necessary] serial number of remote robot.");
    spdlog::info("Usage: ./cart_teleop -l Rizon4s-123456 -r Rizon4s-654321 ");
    // clang-format on
}

struct option kLongOptions[] = {
    // clang-format off
    {"local SN",               required_argument,  0, 'l'},
    {"remote SN",              required_argument,  0, 'r'},
    {0,                      0,                    0,  0 }
    // clang-format on
};

/**
 * @brief function for test
 */
void ConsoleTask(flexiv::tdk::TransparentCartesianTeleopLAN& teleop)
{
    unsigned int index = 0;
    flexiv::tdk::AxisLock cmd;
    teleop.GetAxisLockState(index, cmd);

    while (!teleop.any_fault()) {

        std::string userInput;
        std::getline(std::cin, userInput);

        switch (userInput[0]) {
            case 'x':
                cmd.lock_trans_axis[0] = !cmd.lock_trans_axis[0];
                cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                break;
            case 'y':
                cmd.lock_trans_axis[1] = !cmd.lock_trans_axis[1];
                cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                break;
            case 'z':
                cmd.lock_trans_axis[2] = !cmd.lock_trans_axis[2];
                cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                break;
            case 'q':
                cmd.lock_ori_axis[0] = !cmd.lock_ori_axis[0];
                cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                break;
            case 'w':
                cmd.lock_ori_axis[1] = !cmd.lock_ori_axis[1];
                cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                break;
            case 'e':
                cmd.lock_ori_axis[2] = !cmd.lock_ori_axis[2];
                cmd.coord = flexiv::tdk::CoordType::COORD_WORLD;
                break;

            case 'X':
                cmd.lock_trans_axis[0] = !cmd.lock_trans_axis[0];
                cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                break;
            case 'Y':
                cmd.lock_trans_axis[1] = !cmd.lock_trans_axis[1];
                cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                break;
            case 'Z':
                cmd.lock_trans_axis[2] = !cmd.lock_trans_axis[2];
                cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                break;
            case 'Q':
                cmd.lock_ori_axis[0] = !cmd.lock_ori_axis[0];
                cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                break;
            case 'W':
                cmd.lock_ori_axis[1] = !cmd.lock_ori_axis[1];
                cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                break;
            case 'E':
                cmd.lock_ori_axis[2] = !cmd.lock_ori_axis[2];
                cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                break;
            case 'r':
                teleop.Engage(index, true);
                break;
            case 'R':
                teleop.Engage(index, false);
                break;
            case 's':
                cmd.lock_ori_axis = {false, false, false};
                cmd.lock_trans_axis = {false, false, false};
                cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                break;
            case 'S':
                cmd.lock_ori_axis = {true, true, true};
                cmd.lock_trans_axis = {true, true, true};
                cmd.coord = flexiv::tdk::CoordType::COORD_TCP;
                break;
            case 't':
                teleop.SetRepulsiveForce(index, {5, 0, 0});
                break;
            case 'T':
                teleop.SetWrenchFeedbackScalingFactor(index, 1.5);
                break;
            case 'i':
                teleop.SetLocalNullSpacePosture(index, kPreferredJntPos);
                break;
            case 'I':
                teleop.SetLocalNullSpacePosture(index, kHomeJntPos);
                break;
            case 'o':
                teleop.SetRemoteNullSpacePosture(index, kPreferredJntPos);
                break;
            case 'O':
                teleop.SetRemoteNullSpacePosture(index, kHomeJntPos);
                break;
            case 'p':
                teleop.SetRemoteMaxContactWrench(index, kDefaultMaxContactWrench);
                break;
            case 'L':
                teleop.robot_states(0);
                break;
            case 'k':
                teleop.digital_inputs(0);
                break;

            default:
                spdlog::warn("Invalid command!");
                break;
        }
        teleop.SetAxisLockCmd(index, cmd);
    }
    return;
}

int main(int argc, char* argv[])
{
    std::string remote_sn;
    std::string local_sn;
    int opt = 0;
    int longIndex = 0;
    while ((opt = getopt_long_only(argc, argv, "", kLongOptions, &longIndex)) != -1) {
        switch (opt) {
            case 'r':
                remote_sn = std::string(optarg);
                break;
            case 'l':
                local_sn = std::string(optarg);
                break;
            default:
                printHelp();
                return 1;
        }
    }
    if (local_sn.empty() || remote_sn.empty()) {
        printHelp();
        return 1;
    }

    try {

        // Allocate tdk object
        flexiv::tdk::TransparentCartesianTeleopLAN htt({{local_sn, remote_sn}});

        // Init high transparency teleop
        htt.Init();

        // Start high transparency teleop
        htt.Start();

        // Helper thread
        std::thread helper_thread(std::bind(ConsoleTask, std::ref(htt)));

        // Block main with helper thead
        helper_thread.join();

        // Exit high transparency teleop
        htt.Stop();

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}
