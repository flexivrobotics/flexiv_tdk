/**
 * @file basic_teleop.cpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @brief Force feedback teleoperation example
 * @date 2023-08-22
 */

#include <flexiv/omni/teleop/Robot2RobotTeleop.hpp>
#include <flexiv/omni/teleop/TeleopDefs.hpp>
#include <getopt.h>
#include <iostream>
#include <thread>
#include <chrono>
namespace {
constexpr std::array<double, flexiv::omni::teleop::k_cartDOF> k_maxWrench
    = {20.0, 20.0, 20.0, 5.0, 5.0, 5.0}; ///< Maximum contact wrench of remote robot
}

void printHelp()
{
    // clang-format off
    std::cout<<"Invalid program arguments"<<std::endl;
    std::cout<<"     -l     [necessary] serial number of local robot."<<std::endl;
    std::cout<<"     -r     [necessary] serial number of remote robot."<<std::endl;
    std::cout<<"     -c     [necessary] license config file path."<<std::endl;
    std::cout<<"Usage: ./basic_teleop -l Rizon4s-123456 -r Rizon4s-654321 -c <path/to/licensCfg.json>"<<std::endl;
    // clang-format on
}

struct option k_longOptions[] = {
    // clang-format off
    {"local SN",               required_argument,  0, 'l'},
    {"remote SN",              required_argument,  0, 'r'},
    {"config file of license", required_argument,  0, 'c'},
    {0,                      0,                    0,  0 }
    // clang-format on
};

int main(int argc, char* argv[])
{
    std::string remoteSN;
    std::string localSN;
    std::string licCfgPath;
    bool isBlocking = true;
    int opt = 0;
    int longIndex = 0;
    while ((opt = getopt_long_only(argc, argv, "", k_longOptions, &longIndex)) != -1) {
        switch (opt) {
            case 'r':
                remoteSN = std::string(optarg);
                break;
            case 'l':
                localSN = std::string(optarg);
                break;
            case 'c':
                licCfgPath = std::string(optarg);
                break;
            default:
                printHelp();
                return 1;
        }
    }
    if (localSN.empty() || remoteSN.empty() || licCfgPath.empty()) {
        printHelp();
        return 1;
    }

    std::cout << "Force feedback teleoperation example" << std::endl;
    std::cout << "Copyright (C) 2016-2024 Flexiv" << std::endl;
    std::cout << "All Rights Reserved." << std::endl << std::endl;

    std::cout << "Remote SN: " + remoteSN << std::endl;
    std::cout << "Local SN: " + localSN << std::endl;
    std::cout << "License config file: " + licCfgPath << std::endl;

    try {

        flexiv::omni::teleop::Robot2RobotTeleop teleop(localSN, remoteSN, licCfgPath);

        // Enable teleop robots
        teleop.enable();

        // Init teleop robots
        teleop.init();

        // Get teleop robot current jnt position as preferred jnt pos
        auto localPreferredJntPos = teleop.getLocalInfo().qCurrent;
        auto remotePreferredJntPos = teleop.getRemoteInfo().qCurrent;

        // Set preferred joint position to a better configuration
        teleop.setLocalNullSpacePosture(localPreferredJntPos);
        teleop.setRemoteNullSpacePosture(remotePreferredJntPos);

        // Set remote max wrench, should be a relative low value such that human hand can handle
        teleop.setRemoteMaxWrench(k_maxWrench);

        // Wait for elbow posture ready
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // Run teleop
        std::cout << "Omni-Teleop will run in background ... " << std::endl;
        teleop.run(isBlocking);

    } catch (const std::exception& e) {
        std::cout << e.what() << std::endl;
        return 1;
    }

    return 0;
}
