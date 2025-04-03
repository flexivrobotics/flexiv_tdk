/**
 * @file gripper_ctrl_lan.hpp
 * @date 2025-04-02
 */
#include <flexiv/tdk/gripper_ctrl_lan.hpp>
#include <getopt.h>
#include <iostream>
#include <iomanip>
#include <spdlog/spdlog.h>
namespace {
/** Global flag: whether the gripper control tasks are finished */
std::atomic<bool> g_finished = {false};
struct option kLongOptions[] = {
    // clang-format off
    {"serial number of robot",                     required_argument,  0, 's'},
    {"name of the installed gripper",              required_argument,  0, 'n'},
    {0,                      0,                                        0,  0 }
    // clang-format on
};

}
void PrintHelp()
{
    // clang-format off
    std::cout << "Usage: ./test_gripper_ctrl_lan -s <robot serial number> -n <name>" << std::endl;
    // clang-format on
}

void PrintGripperStates(const flexiv::tdk::GripperCtrlLAN& gcl)
{
    while (!g_finished.load()) {
        // clang-format off
    std::cout<< std::fixed << std::setprecision(3)
            << "{\n\"width\": " << gcl.states().width
            << ", \n\"force\": " << gcl.states().force
            << ", \n\"is_moving\": " << gcl.states().is_moving
            << "\n}";
        // clang-format on
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main(int argc, char* argv[])
{

    std::string robot_sn;
    std::string gripper_name;
    int opt = 0;
    while ((opt = getopt_long(argc, argv, "s:n:", kLongOptions, nullptr)) != -1) {
        switch (opt) {
            case 's':
                robot_sn = std::string(optarg);
                break;
            case 'n':
                gripper_name = std::string(optarg);
                break;
            default:
                PrintHelp();
                return 1;
        }
    }
    if (robot_sn.empty() || gripper_name.empty()) {
        PrintHelp();
        return 1;
    }

    try {
        flexiv::tdk::GripperCtrlLAN gcl(robot_sn);

        // Enable gripper
        gcl.Enable(gripper_name);

        // Initialize gripper
        gcl.Init();

        // Wait for initialization finished, the gripper will open and close.
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // Print parameters of the enabled gripper
        spdlog::info("Gripper params:");
        std::cout << std::fixed << std::setprecision(3) << "{\n"
                  << "name: " << gcl.params().name << "\nmin_width: " << gcl.params().min_width
                  << "\nmax_width: " << gcl.params().max_width
                  << "\nmin_force: " << gcl.params().min_force
                  << "\nmax_force: " << gcl.params().max_force
                  << "\nmin_vel: " << gcl.params().min_vel << "\nmax_vel: " << gcl.params().max_vel
                  << "\n}" << std::endl;

        // Start a separate thread to print gripper states
        std::thread print_thread(PrintGripperStates, std::ref(gcl));
        // Position control
        spdlog::info("Closing gripper");
        gcl.Move(0.01, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        spdlog::info("Opening gripper");
        gcl.Move(0.09, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Stop
        spdlog::info("Closing gripper");
        gcl.Move(0.01, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        spdlog::info("Stopping gripper");
        gcl.Stop();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        spdlog::info("Closing gripper");
        gcl.Move(0.01, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        spdlog::info("Opening gripper");
        gcl.Move(0.09, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        spdlog::info("Stopping gripper");
        gcl.Stop();
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Force control, if available (sensed force is not zero)
        if (fabs(gcl.states().force) > std::numeric_limits<double>::epsilon()) {
            spdlog::info("Gripper running zero force control");
            gcl.Grasp(0);
            // Exit after 10 seconds
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }

        // Finished, exit all threads
        gcl.Stop();
        g_finished.store(true);
        spdlog::info("Program finished");
        print_thread.join();
    } catch (...) {
        return 1;
    }
    return 0;
}