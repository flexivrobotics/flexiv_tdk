/**
 * @file Robot2RobotTeleop.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */
#pragma once

#include <string>
#include <memory>
#include <flexiv/omni/teleop/TeleopDefs.hpp>
namespace flexiv {
namespace omni {
namespace teleop {

/**
 * @brief Main interface for robot-robot teleop, containing several function categories
 * and background services.
 */
class Robot2RobotTeleop
{
public:
    Robot2RobotTeleop() = delete;

    /**
     * @brief [Blocking] Create a flexiv::omni::teleop instance as the main teleoperation interface.
     * Robot2RobotTeleop services will initialize and connection with the robots will be
     * established.
     * @param[in] localSN Serial number of the local robot
     * @param[in] remoteSN Serial number of the remote robot
     * @param [in] licensePath Path to the license config json file.
     * @throw std::runtime_error if the initialization sequence failed.
     * @throw std::logic_error if the connected robot does not have a valid license; or this
     * Teleop library version is incompatible with the connected robot; or model of the connected
     * robot is not supported.
     * @warning This constructor blocks until the initialization sequence is successfully finished
     * and connection with the robot is established.
     */
    Robot2RobotTeleop(
        const std::string& localSN, const std::string& remoteSN, const std::string& licensePath);
    virtual ~Robot2RobotTeleop();

    /**
     * @brief [Blocking] Initialize teleoperation robots states and coordinate bias between local
     * and remote.
     * @throw std::logic_error if the robot is not connected.
     * @throw std::runtime_error if failed to execute the request.
     * @note This function blocks until the request is successfully executed.
     */
    void init(void);

    /**
     * @brief [Blocking] Enable the teleoperation, if all E-stop are released and there's no
     * fault, robots will release brakes, and becomes operational a few seconds later.
     * @throw std::logic_error if the robot is not connected.
     * @throw std::runtime_error if failed to execute the request.
     * @note This function blocks until the request is successfully executed.
     */
    void enable(void);

    /**
     * @brief [Blocking] Stop teleoperation.
     * @throw std::runtime_error if failed to stop teleoperation.
     * @note This function blocks until teleoperation comes to a complete stop.
     */
    void stop(void);

    /**
     * @brief [Non-blocking] Check if teleoperaion in fault state.
     * @return True: teleoperation has fault, false: everything normal.
     */
    bool isFault(void);

    /**
     * @brief [Blocking] Clear minor fault.
     * @throw std::runtime_error if failed to execute the request.
     * @note This function blocks until the request is successfully executed.
     */
    void clearFault(void);

    /**
     * @brief [Blocking/Non-blocking] Run teleoperation. The remote will always imitate the
     * movements of the local.
     * @note The remote pose will not exactly the same as that of the local. User can keep the
     * remote still by pressing the pedal, while the local can drag freely. When the local reaches
     * an appropriate pose, release the pedal and the remote will follow the local again.
     * @warning Please connect the pedal to the DI0 and 24V channel of the local control box. And
     * ensure that the circuit only connects when the pedal is pressed down.
     * @param [in] isBlocking True to block the whole program while running teleoperation, false to
     * set a non-blocking running behavior.
     * @throw std::runtime_error if failed to execute the request.
     */
    void run(bool isBlocking);

    /**
     * @brief [Non-blocking] Set local robot Cartesian motion stiffness.
     * @param[in] stiffness Cartesian motion stiffness: \f$ K_d \in \mathbb{R}^{6 \times 1} \f$.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear stiffness and \f$
     * \mathbb{R}^{3 \times 1} \f$ angular stiffness: \f$ [k_x, k_y, k_z, k_{Rx}, k_{Ry}, k_{Rz}]^T
     * \f$. Valid range: [0, RobotInfo::nominalK]. Unit: \f$ [N/m]~[Nm/rad] \f$.
     * @throw std::invalid_argument if [stiffness] contains any value outside the valid range.
     * @throw std::logic_error if local robot is not initialized.
     * @warning The local robot will automatically reset to its nominal stiffness upon re-entering
     * run.
     */
    void setLocalCartesianStiffness(const std::array<double, k_cartDOF>& stiffness);

    /**
     * @brief [Non-blocking] Set local robot Cartesian motion stiffness.
     * @param[in] stiffness Cartesian motion stiffness: \f$ K_d \in \mathbb{R}^{6 \times 1} \f$.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear stiffness and \f$
     * \mathbb{R}^{3 \times 1} \f$ angular stiffness: \f$ [k_x, k_y, k_z, k_{Rx}, k_{Ry}, k_{Rz}]^T
     * \f$. Valid range: [0, RobotInfo::nominalK]. Unit: \f$ [N/m]~[Nm/rad] \f$.
     * @throw std::invalid_argument if [stiffness] contains any value outside the valid range.
     * @throw std::logic_error if local robot is not initialized.
     * @warning The local robot will automatically reset to its nominal stiffness upon re-entering
     * run.
     */
    void setRemoteCartesianStiffness(const std::array<double, k_cartDOF>& stiffness);

    /**
     * @brief [Non-blocking] Set the local robot axis locking command.
     *
     * @param[in] cmd User input command to lock the motion of the specified axis in the reference
     * coordinate.
     */
    void setLocalAxisLockCmd(const AxisLockDefs& cmd);

    /**
     * @brief [Non-blocking] Get the local robot axis locking status
     *
     * @param[out] data Current axis locking state of local robot.
     */
    void getLocalAxisLockState(AxisLockDefs& data);

    /**
     * @brief [Non-blocking] Get the local robot axis locking status
     * @warning This fuction is less efficient than the other overloaded one as additional runtime
     * memory allocation and data copying are performed.
     * @return AxisLockDefs
     */
    AxisLockDefs getLocalAxisLockState(void);

private:
    class Impl;
    std::unique_ptr<Impl> m_pImpl;
};

} // namespace teleop
} // namespace omni
} // namespace flexiv
