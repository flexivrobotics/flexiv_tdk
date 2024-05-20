/**
 * @file basics1_cartesian_teleop.cpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @brief Basic Omni-Teleop example
 * @date 2024-05-10
 */

#include <flexiv/omni/teleop/Robot2RobotTeleop.hpp>
#include <flexiv/omni/teleop/TeleopDefs.hpp>
#include <flexiv/omni/teleop/Scheduler.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
namespace {
constexpr std::array<double, flexiv::omni::teleop::k_jointDOF> k_preferredJntPos
    = {-0.67, -0.98, 0.89, 1.55, -0.85, 0.54, 0.46}; ///< Preferred joint position
const std::array<double, flexiv::omni::teleop::k_cartDOF> k_defaultMaxRemoteWrench
    = {80.0, 80.0, 80.0, 24.0, 24.0, 24.0}; ///< Maximum contact wrench of remote robot
std::atomic<bool> g_stop_sched = {false};   ///< Atomic signal to stop scheduler tasks
}

void printHelp()
{
    // clang-format off
    std::cout << "Invalid program arguments"<<std::endl;
    std::cout << "Required arguments: [remote_robot_SN] [local_robot_SN] [path_to_omni_licenseCfg.json]" << std::endl;
    std::cout << "    remote_robot_SN: serial number of remote robot" << std::endl;
    std::cout << "    local_robot_SN: serial number of local robot" << std::endl;
    std::cout << "    path_to_omni_licenseCfg: path to the Omni license config json file" << std::endl;
    std::cout<<"Usage: ./basic1_cart_teleop  Rizon4s-123456  Rizon4s-654321 <path/to/licensCfg.json>"<<std::endl;
    // clang-format on
}

/**
 * @brief  Callback function for teleop task
 */
void periodicTeleopTask(flexiv::omni::teleop::Robot2RobotTeleop& teleop)
{

    try {
        // Monitor status of the teleop robots
        if (!teleop.isOperational()) {
            throw std::runtime_error(
                "periodicTeleopTask: Fault occurred during teleoperation, exiting ...");
        }
        // Step teleop
        teleop.step();
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        g_stop_sched = true;
    }
}

/**
 * @brief Callback function for axis lock/unlock test
 */
void periodicConsoleTask(flexiv::omni::teleop::Robot2RobotTeleop& teleop)
{
    flexiv::omni::teleop::AxisLockDefs cmd;
    teleop.getLocalAxisLockState(cmd);

    while (!g_stop_sched) {

        std::string userInput;
        std::getline(std::cin, userInput);

        switch (userInput[0]) {
            case 'm':
                std::cout << ">>> Simple command line GUI for teleop robot axis lock <<<"
                          << std::endl;
                std::cout << "- x: lock/unlock translational motion along X axis in World frame."
                          << std::endl;
                std::cout << "- y: lock/unlock translational motion along Y axis in World frame."
                          << std::endl;
                std::cout << "- z: lock/unlock translational motion along Z axis in World frame."
                          << std::endl;

                std::cout << "- q: lock/unlock rotational motion along X axis in World frame."
                          << std::endl;
                std::cout << "- w: lock/unlock rotational motion along Y axis in World frame."
                          << std::endl;
                std::cout << "- e: lock/unlock rotational motion along Z axis in World frame."
                          << std::endl;

                std::cout << "- X: lock/unlock translational motion along X axis in TCP frame."
                          << std::endl;
                std::cout << "- Y: lock/unlock translational motion along Y axis in TCP frame."
                          << std::endl;
                std::cout << "- Z: lock/unlock translational motion along Z axis in TCP frame."
                          << std::endl;

                std::cout << "- Q: lock/unlock rotational motion along X axis in TCP frame."
                          << std::endl;
                std::cout << "- W: lock/unlock rotational motion along Y axis in TCP frame."
                          << std::endl;
                std::cout << "- E: lock/unlock rotational motion along Z axis in TCP frame."
                          << std::endl;
                std::cout << "please input command >> " << std::endl;
                break;
            case 'x':
                cmd.trans_axis_lock_list_[0] = !cmd.trans_axis_lock_list_[0];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_WORLD;

                break;
            case 'y':
                cmd.trans_axis_lock_list_[1] = !cmd.trans_axis_lock_list_[1];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_WORLD;
                break;
            case 'z':
                cmd.trans_axis_lock_list_[2] = !cmd.trans_axis_lock_list_[2];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_WORLD;
                break;
            case 'q':
                cmd.ori_axis_lock_list_[0] = !cmd.ori_axis_lock_list_[0];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_WORLD;
                break;
            case 'w':
                cmd.ori_axis_lock_list_[1] = !cmd.ori_axis_lock_list_[1];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_WORLD;
                break;
            case 'e':
                cmd.ori_axis_lock_list_[2] = !cmd.ori_axis_lock_list_[2];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_WORLD;
                break;

            case 'X':
                cmd.trans_axis_lock_list_[0] = !cmd.trans_axis_lock_list_[0];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_TCP;
                break;
            case 'Y':
                cmd.trans_axis_lock_list_[1] = !cmd.trans_axis_lock_list_[1];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_TCP;
                break;
            case 'Z':
                cmd.trans_axis_lock_list_[2] = !cmd.trans_axis_lock_list_[2];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_TCP;
                break;
            case 'Q':
                cmd.ori_axis_lock_list_[0] = !cmd.ori_axis_lock_list_[0];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_TCP;
                break;
            case 'W':
                cmd.ori_axis_lock_list_[1] = !cmd.ori_axis_lock_list_[1];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_TCP;
                break;
            case 'E':
                cmd.ori_axis_lock_list_[2] = !cmd.ori_axis_lock_list_[2];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_TCP;
                break;

            default:
                std::cerr << "Invalid command, please enter \'m\' for help \n";
                break;
        }
        teleop.setLocalAxisLockCmd(cmd);
        std::cout << " Axis [X] in " << flexiv::omni::teleop::CoordTypeStr[cmd.coord]
                  << " frame locking status : " << cmd.trans_axis_lock_list_[0] << std::endl;
        std::cout << " Axis [Y] in  " << flexiv::omni::teleop::CoordTypeStr[cmd.coord]
                  << " frame locking status : " << flexiv::omni::teleop::CoordTypeStr[cmd.coord]
                  << cmd.trans_axis_lock_list_[1] << std::endl;
        std::cout << " Axis [Z] in  " << flexiv::omni::teleop::CoordTypeStr[cmd.coord]
                  << " frame locking status : " << flexiv::omni::teleop::CoordTypeStr[cmd.coord]
                  << cmd.trans_axis_lock_list_[2] << std::endl;
        std::cout << " Axis [Rx] in  " << flexiv::omni::teleop::CoordTypeStr[cmd.coord]
                  << " frame locking status : " << flexiv::omni::teleop::CoordTypeStr[cmd.coord]
                  << cmd.ori_axis_lock_list_[0] << std::endl;
        std::cout << " Axis [Ry] in  " << flexiv::omni::teleop::CoordTypeStr[cmd.coord]
                  << " frame locking status : " << flexiv::omni::teleop::CoordTypeStr[cmd.coord]
                  << cmd.ori_axis_lock_list_[1] << std::endl;
        std::cout << " Axis [Rz] in  " << flexiv::omni::teleop::CoordTypeStr[cmd.coord]
                  << " frame locking status : " << flexiv::omni::teleop::CoordTypeStr[cmd.coord]
                  << cmd.ori_axis_lock_list_[2] << std::endl;
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

    try {
        // Create Omni-Teleop
        flexiv::omni::teleop::Robot2RobotTeleop teleop(localSN, remoteSN, licCfgPath);

        // Enable teleop robots
        teleop.enable();

        // Init teleop robots
        teleop.init();

        // Set preferred joint position to a better configuration
        teleop.setLocalNullSpacePosture(k_preferredJntPos);
        teleop.setRemoteNullSpacePosture(k_preferredJntPos);

        // Set max remote contact wrench
        teleop.setRemoteMaxWrench(k_defaultMaxRemoteWrench);

        // Create real-time scheduler to step periodic tasks
        flexiv::omni::teleop::Scheduler scheduler;

        // Wait for elbow posture ready
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.AddTask(std::bind(&periodicTeleopTask, std::ref(teleop)), "HP periodic teleop", 1,
            scheduler.max_priority());

        scheduler.AddTask(std::bind(&periodicConsoleTask, std::ref(teleop)), "LP Periodic console",
            1000, scheduler.min_priority());

        // Start all added tasks, this is by default a blocking method
        scheduler.Start();

        std::cout << "Flexiv Omni-Teleop started ... ";

        // Wait a bit for any last-second robot log message to arrive and get printed
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Block until signal received
        while (!g_stop_sched) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

    } catch (const std::exception& e) {
        std::cerr << (e.what());
        return 1;
    }

    return 0;
}