/**
 * @file cart_teleop.cpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @brief This is an example program for cartesian space teleoperation
 * @date 2024-07-18
 */

#include <atomic>
#include <chrono>
#include <spdlog/spdlog.h>
#include <flexiv/tdk/robot_robot_teleop.hpp>
#include <flexiv/tdk/data.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/data.hpp>
#include <thread>
namespace {
std::vector<double> kPreferredJntPos
    = {60 * M_PI / 180.0, -60 * M_PI / 180.0, -85 * M_PI / 180.0, 115 * M_PI / 180.0,
        70 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0}; ///< Preferred joint position

const std::array<double, flexiv::rdk::kCartDoF> kDefaultMaxContactWrench
    = {200.0, 200.0, 200.0, 40.0, 40.0, 40.0}; ///< Maximum contact wrench

std::atomic<bool> g_stop_sched = {false}; ///< Atomic signal to stop scheduler tasks
} // namespace

/**
 * @brief Helper function to print program usage help
 */
void printHelp()
{
    // clang-format off
    spdlog::error("Invalid program arguments");
    spdlog::info("  serial number of local robot");
    spdlog::info("  serial number of remote robot");
    spdlog::info("  license config file path");
    spdlog::info("Usage: sudo ./basic1_cart_teleop Rizon4s-123456 Rizon4s-654321 <absolute path to licenseCfg.json>");
    // clang-format on
}

/**
 * @brief 1000Hz callback function for main teleop task
 */
void PeriodicTeleopTask(flexiv::tdk::Robot2RobotTeleop& teleop)
{

    try {
        // Monitor fault on the teleop robots
        if (!teleop.operational()) {
            throw std::runtime_error(
                "PeriodicTeleopTask: Fault occurred during teleoperation, exiting ...");
        }
        // Step teleop
        teleop.Step();
    } catch (const std::exception& e) {
        spdlog::error(e.what());
        g_stop_sched = true;
    }
}

/**
 * @brief 1Hz callback function for several API test
 */
void PeriodicConsoleTask(flexiv::tdk::Robot2RobotTeleop& teleop)
{
    flexiv::tdk::AxisLockStatus cmd;
    teleop.GetLocalAxisLockState(cmd);

    while (!g_stop_sched) {

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
                teleop.Engage(true);
                break;
            case 'R':
                teleop.Engage(false);
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
                teleop.SetRepulsiveWrench({0, 0, 0, 0, 0, 0});
                break;
            default:
                spdlog::warn("Invalid command!");
                break;
        }
        teleop.SetLocalAxisLockCmd(cmd);
    }
    return;
}

int main(int argc, char* argv[])
{
    std::string remote_sn;
    std::string local_sn;
    std::string path_to_lic_jsn;

    if (argc != 4) {
        printHelp();
        return 1;
    }
    local_sn = argv[1];
    remote_sn = argv[2];
    path_to_lic_jsn = argv[3];

    if (local_sn.empty() || remote_sn.empty() || path_to_lic_jsn.empty()) {
        printHelp();
        return 1;
    }

    try {

        flexiv::tdk::Robot2RobotTeleop teleop(local_sn, remote_sn, path_to_lic_jsn);

        // Enable cart teleop
        teleop.Enable();

        // Init cart teleop
        teleop.Init();

        // Set preferred joint position to a better configuration
        teleop.SetLocalNullSpacePosture(kPreferredJntPos);
        teleop.SetRemoteNullSpacePosture(kPreferredJntPos);

        // Set max remote contact wrench
        teleop.SetMaxContactWrench(kDefaultMaxContactWrench);

        // Create real-time scheduler to step periodic tasks
        flexiv::rdk::Scheduler scheduler;

        // Wait for elbow posture ready
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.AddTask(std::bind(&PeriodicTeleopTask, std::ref(teleop)), "HP periodic teleop", 1,
            scheduler.max_priority());

        scheduler.AddTask(std::bind(&PeriodicConsoleTask, std::ref(teleop)), "LP Periodic console",
            1000, scheduler.min_priority());

        // Start all added tasks, this is by default a blocking method
        scheduler.Start();

        spdlog::info("Flexiv TDK example started ... ");

        // Block until signal received
        while (!g_stop_sched) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}