/**
 * @file cart_teleop.cpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @brief This is an example program for cartesian space teleoperation
 * @date 2024-07-18
 */

// Flexiv
#include <atomic>
#include <chrono>
#include <spdlog/spdlog.h>
#include <flexiv/teleop/robot_robot_teleop.hpp>
#include <flexiv/teleop/scheduler.hpp>
#include <flexiv/teleop/teleop_defs.hpp>
#include <getopt.h>
#include <iostream>
#include <thread>
namespace {
std::vector<double> k_preferredJntPos
    = {44 * M_PI / 180.0, -61 * M_PI / 180.0, -71 * M_PI / 180.0, 84 * M_PI / 180.0,
        59 * M_PI / 180.0, 12 * M_PI / 180.0, -31 * M_PI / 180.0}; ///< Preferred joint position
const std::array<double, flexiv::teleop::kCartDOF> k_defaultMaxRemoteWrench
    = {40.0, 40.0, 40.0, 24.0, 24.0, 24.0}; ///< Maximum contact wrench
std::atomic<bool> g_stop_sched = {false};   ///< Atomic signal to stop scheduler tasks
} // namespace

void printHelp()
{
    // clang-format off
    spdlog::error("Invalid program arguments");
    spdlog::info("     [necessary] serial number of local robot.");
    spdlog::info("     [necessary] serial number of remote robot.");
    spdlog::info("     [necessary] license config file path.");
    spdlog::info("Usage: ./basic1_cart_teleop Rizon4s-123456 Rizon4s-654321 <path/to/licensCfg.json>");
    // clang-format on
}

/**
 * @brief  Callback function for Omni Teleop task
 */
void PeriodicTeleopTask(flexiv::teleop::Robot2RobotTeleop& teleop)
{

    try {
        // Monitor fault on the teleop robots
        if (!teleop.IsOperational()) {
            throw std::runtime_error(
                "PeriodicTeleopTask: Fault occurred during "
                "teleoperation, exiting ...");
        }
        // Step Omni Teleop
        teleop.Step();
    } catch (const std::exception& e) {
        spdlog::error(e.what());
        g_stop_sched = true;
    }
}

/**
 * @brief Callback function for axis lock/unlock test
 */
void PeriodicConsoleTask(flexiv::teleop::Robot2RobotTeleop& teleop)
{
    flexiv::teleop::AxisLockDefs cmd;
    teleop.GetLocalAxisLockState(cmd);

    while (!g_stop_sched) {

        std::string userInput;
        std::getline(std::cin, userInput);

        switch (userInput[0]) {
            case 'm':
                spdlog::info(">>> Simple command line GUI for teleop robot axis lock <<<");
                spdlog::info("- x: lock/unlock translational motion along X axis in World frame.");
                spdlog::info("- y: lock/unlock translational motion along Y axis in World frame.");
                spdlog::info("- z: lock/unlock translational motion along Z axis in World frame.");

                spdlog::info("- q: lock/unlock rotational motion along X axis in World frame.");
                spdlog::info("- w: lock/unlock rotational motion along Y axis in World frame.");
                spdlog::info("- e: lock/unlock rotational motion along Z axis in World frame.");

                spdlog::info("- X: lock/unlock translational motion along X axis in TCP frame.");
                spdlog::info("- Y: lock/unlock translational motion along Y axis in TCP frame.");
                spdlog::info("- Z: lock/unlock translational motion along Z axis in TCP frame.");

                spdlog::info("- Q: lock/unlock rotational motion along X axis in TCP frame.");
                spdlog::info("- W: lock/unlock rotational motion along Y axis in TCP frame.");
                spdlog::info("- E: lock/unlock rotational motion along Z axis in TCP frame.");

                spdlog::info("- r: engage/disengage teleop.");

                spdlog::info("please input command >> ");
                break;
            case 'x':
                cmd.trans_axis_lock_list_[0] = !cmd.trans_axis_lock_list_[0];
                cmd.coord = flexiv::teleop::CoordType::CD_WORLD;

                break;
            case 'y':
                cmd.trans_axis_lock_list_[1] = !cmd.trans_axis_lock_list_[1];
                cmd.coord = flexiv::teleop::CoordType::CD_WORLD;
                break;
            case 'z':
                cmd.trans_axis_lock_list_[2] = !cmd.trans_axis_lock_list_[2];
                cmd.coord = flexiv::teleop::CoordType::CD_WORLD;
                break;
            case 'q':
                cmd.ori_axis_lock_list_[0] = !cmd.ori_axis_lock_list_[0];
                cmd.coord = flexiv::teleop::CoordType::CD_WORLD;
                break;
            case 'w':
                cmd.ori_axis_lock_list_[1] = !cmd.ori_axis_lock_list_[1];
                cmd.coord = flexiv::teleop::CoordType::CD_WORLD;
                break;
            case 'e':
                cmd.ori_axis_lock_list_[2] = !cmd.ori_axis_lock_list_[2];
                cmd.coord = flexiv::teleop::CoordType::CD_WORLD;
                break;

            case 'X':
                cmd.trans_axis_lock_list_[0] = !cmd.trans_axis_lock_list_[0];
                cmd.coord = flexiv::teleop::CoordType::CD_TCP;
                break;
            case 'Y':
                cmd.trans_axis_lock_list_[1] = !cmd.trans_axis_lock_list_[1];
                cmd.coord = flexiv::teleop::CoordType::CD_TCP;
                break;
            case 'Z':
                cmd.trans_axis_lock_list_[2] = !cmd.trans_axis_lock_list_[2];
                cmd.coord = flexiv::teleop::CoordType::CD_TCP;
                break;
            case 'Q':
                cmd.ori_axis_lock_list_[0] = !cmd.ori_axis_lock_list_[0];
                cmd.coord = flexiv::teleop::CoordType::CD_TCP;
                break;
            case 'W':
                cmd.ori_axis_lock_list_[1] = !cmd.ori_axis_lock_list_[1];
                cmd.coord = flexiv::teleop::CoordType::CD_TCP;
                break;
            case 'E':
                cmd.ori_axis_lock_list_[2] = !cmd.ori_axis_lock_list_[2];
                cmd.coord = flexiv::teleop::CoordType::CD_TCP;
                break;
            case 'r':
                teleop.Engage(true);
                break;
            case 'R':
                teleop.Engage(false);
                break;
            default:
                spdlog::warn("Invalid command, please enter \'m\' for help \n");
                break;
        }
        teleop.SetLocalAxisLockCmd(cmd);
        spdlog::info(" Axis [X] in [{}] frame locking status : [{}]",
            flexiv::teleop::CoordTypeStr[cmd.coord], cmd.trans_axis_lock_list_[0]);
        spdlog::info(" Axis [Y] in [{}] frame locking status : [{}]",
            flexiv::teleop::CoordTypeStr[cmd.coord], cmd.trans_axis_lock_list_[1]);
        spdlog::info(" Axis [Z] in [{}] frame locking status : [{}]",
            flexiv::teleop::CoordTypeStr[cmd.coord], cmd.trans_axis_lock_list_[2]);
        spdlog::info(" Axis [Rx] in [{}] frame locking status : [{}]",
            flexiv::teleop::CoordTypeStr[cmd.coord], cmd.ori_axis_lock_list_[0]);
        spdlog::info(" Axis [Ry] in [{}] frame locking status : [{}]",
            flexiv::teleop::CoordTypeStr[cmd.coord], cmd.ori_axis_lock_list_[1]);
        spdlog::info(" Axis [Rz] in [{}] frame locking status : [{}]",
            flexiv::teleop::CoordTypeStr[cmd.coord], cmd.ori_axis_lock_list_[2]);
    }
    return;
}

int main(int argc, char* argv[])
{
    std::string remoteSN;
    std::string localSN;
    std::string licCfgPath;

    if (argc != 4) {
        printHelp();
        return 1;
    }
    remoteSN = argv[1];
    localSN = argv[2];
    licCfgPath = argv[3];

    std::cout << "Flexiv Omni-Teleop example" << std::endl;
    std::cout << "Copyright (C) 2016-2024 Flexiv" << std::endl;
    std::cout << "All Rights Reserved." << std::endl << std::endl;

    std::cout << "Remote SN: " + remoteSN << std::endl;
    std::cout << "Local SN: " + localSN << std::endl;
    std::cout << "License config file: " + licCfgPath << std::endl;

    if (localSN.empty() || remoteSN.empty() || licCfgPath.empty()) {
        printHelp();
        return 1;
    }

    try {

        flexiv::teleop::Robot2RobotTeleop teleop(localSN, remoteSN, licCfgPath);

        // Enable cart teleop
        teleop.Enable();

        // Init cart teleop
        teleop.Init();

        // Set preferred joint position to a better configuration
        teleop.SetLocalNullSpacePosture(k_preferredJntPos);
        teleop.SetRemoteNullSpacePosture(k_preferredJntPos);

        // Set max remote contact wrench
        teleop.SetMaxContactWrench(k_defaultMaxRemoteWrench);

        // Create real-time scheduler to step periodic tasks
        flexiv::teleop::Scheduler scheduler;

        // Wait for elbow posture ready
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.AddTask(std::bind(&PeriodicTeleopTask, std::ref(teleop)), "HP periodic teleop", 1,
            scheduler.max_priority());

        scheduler.AddTask(std::bind(&PeriodicConsoleTask, std::ref(teleop)),
            "LP nonPeriodic console", 1000, scheduler.min_priority());

        // Start all added tasks, this is by default a blocking method
        scheduler.Start();

        spdlog::info("Omni-CartTeleop started ... ");

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
