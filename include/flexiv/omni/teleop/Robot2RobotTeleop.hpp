/**
 * @file Robot2RobotTeleop.hpp
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 */
#pragma once

#include <string>
#include <memory>

namespace flexiv {
namespace omni {
namespace teleop {

/** Robot Cartesian-space degrees of freedom \f$ m \f$ */
constexpr size_t k_cartDOF = 6;

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
     * @throw std::runtime_error if the initialization sequence failed.
     * @throw std::logic_error if the connected robot does not have a valid license; or this
     * Teleop library version is incompatible with the connected robot; or model of the connected
     * robot is not supported.
     * @warning This constructor blocks until the initialization sequence is successfully finished
     * and connection with the robot is established.
     */
    Robot2RobotTeleop(const std::string& localSN, const std::string& remoteSN);
    virtual ~Robot2RobotTeleop();

    /**
     * @brief [Blocking] First, the remote robot will go to the home pose, and then the local will
     * go to the same pose as the remote
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
     * @brief [Blocking/Non-blocking] Run teleoperation. The remote will always maintain the same
     * pose as the local while the user operates the local.
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

private:
    class Impl;
    std::unique_ptr<Impl> m_pImpl;
};

} // namespace teleop
} // namespace omni
} // namespace flexiv
