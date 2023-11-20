/**
 * @example basic_free_space_motion_teleop.cpp
 * @author flexiv
 * @brief basic free space motion robot to robot teleoperation using flexiv Rizon4s
 * @copyright Copyright (c) 2023
 */

// Flexiv
#include <flexiv/omni/teleop/Teleoperation.hpp>
#include <getopt.h>
#include <iostream>
#include <thread>
#include <chrono>

using namespace flexiv;
/** @brief Print usage of example*/
void printHelp()
{
  // clang-format off
    std::cout<<"Invalid program arguments"<<std::endl;
    std::cout<<"     -l     [necessary] serial number of local robot."<<std::endl;
    std::cout<<"     -r     [necessary] serial number of remote robot."<<std::endl;
    std::cout<<"Usage: ./test_flexiv_omni_teleop -l Rizon4s-123456 -r Rizon4s-654321"<<std::endl;
  // clang-format on
}

struct option k_longOptions[] = {
    // clang-format off
    {"local SN",      required_argument,  0, 'l'},
    {"remote SN",       required_argument,  0, 'r'},
    {0,                0,                  0,  0 }
    // clang-format on
};

int main(int argc, char* argv[])
{
  std::string remoteSN;
  std::string localSN;
  bool isBlocking = false;
  int opt = 0;
  int longIndex = 0;
  while ((opt = getopt_long_only(argc, argv, "", k_longOptions, &longIndex)) != -1) {
    switch (opt) {
      case 'r':
        remoteSN = std::string(optarg);
        std::cout << "Remote SN: " + remoteSN << std::endl;
        break;
      case 'l':
        localSN = std::string(optarg);
        std::cout << "Local SN: " + localSN << std::endl;
        break;
      default:
        printHelp();
        return 1;
    }
  }
  if (localSN.empty() || remoteSN.empty()) {
    printHelp();
    return 1;
  }

  try {

    flexiv::omni::teleop::Teleoperation teleop(localSN, remoteSN);

    // Enable teleop robots
    teleop.enable();

    // Init teleop robots
    teleop.init();

    // Run teleop
    teleop.run(isBlocking);

    while (true) {
      std::cout << "Omni-Teleop is running in background... " << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  } catch (const std::exception& e) {
    std::cout << e.what() << std::endl;
    return 1;
  }

  return 0;
}
