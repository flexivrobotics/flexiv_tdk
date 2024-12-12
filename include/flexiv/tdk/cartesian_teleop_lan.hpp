/**
 * @file cartesian_teleop_lan.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */
#pragma once

#include "data.hpp"
#include <string>
#include <memory>

namespace flexiv {
namespace tdk {

/**
 * @brief Teleoperation control interface to run Cartesian-space robot-robot teleoperation for one
 * or more pairs of robots connected to the same LAN.
 */
class CartesianTeleopLAN
{
public:
    /**
     * @brief [Blocking] Create an instance of the control interface. More than one pair of
     * teleoperated robots can be controlled at the same time, see parameter [robot_pairs_sn].
     * @param[in] robot_pairs_sn Serial number of all robot pairs to run teleoperation on. Each pair
     * in the vector represents a pair of bilaterally teleoperated robots. For example, provide 2
     * pairs of robot serial numbers to start a dual-arm teleoperation that involves 2 pairs of
     * robots. The accepted formats are: "Rizon 4s-123456" and "Rizon4s-123456".
     * @param[in] network_interface_whitelist Limit the network interface(s) that can be used to try
     * to establish connection with the specified robot. The whitelisted network interface is
     * defined by its associated IPv4 address. For example, {"10.42.0.1", "192.168.2.102"}. If left
     * empty, all available network interfaces will be tried when searching for the specified robot.
     * @throw std::invalid_argument if the format of any element in [robot_pairs_sn] is invalid.
     * @throw std::runtime_error if error occurred during construction.
     * @throw std::logic_error if one of the connected robots does not have a valid TDK license; or
     * the version of this TDK library is incompatible with one of the connected robots; or model of
     * any connected robot is not supported.
     * @warning This constructor blocks until the initialization sequence is successfully finished
     * and connection with all robots is established.
     */
    CartesianTeleopLAN(const std::vector<std::pair<std::string, std::string>>& robot_pairs_sn,
        const std::vector<std::string>& network_interface_whitelist = {});
    virtual ~CartesianTeleopLAN();

    /**
     * @brief [Blocking] Get all robots ready for teleoperation. The following actions will
     * happen in sequence: a) enable robot, b) zero force/torque sensors.
     * @throw std::runtime_error if the initialization sequence failed.
     * @note This function blocks until the initialization sequence is finished.
     * @warning This process involves sensor zeroing, please make sure the robot is not in contact
     * with anything during the process.
     */
    void Init();

    /**
     * @brief [Blocking] Sync pose of the specified robot pair.
     * @param[in] idx Index of the robot pair to sync pose. This index is the same as the index of
     * the constructor parameter [robot_pairs_sn].
     * @param[in] sync_positions Joint positions for both robots in pair [idx] to move to: \f$ q_0
     * \in \mathbb{R}^{n \times 1} \f$. If left empty, the first robot in the pair will stay at its
     * current pose, and the second robot in the pair will move to the first robot's joint
     * positions. Unit: \f$ [rad] \f$.
     * @throw std::invalid_argument if [idx] exceeds total number of robot pairs.
     * @throw std::invalid_argument if size of [sync_positions] does not match robot DoF.
     * @throw std::logic_error if initialization sequence hasn't been triggered yet using Init().
     * @note This function blocks until the sync is finished.
     */
    void SyncPose(unsigned int idx, const std::vector<double>& sync_positions = {});

    /**
     * @brief [Blocking] Start the teleoperation control loop.
     * @throw std::runtime_error if failed to start the teleoperation control loop.
     * @throw std::logic_error if initialization sequence hasn't been triggered yet using Init().
     * @throw std::logic_error if pose of any robot pair has not been synced yet using SyncPose().
     * @note This function blocks until the control loop has started running. The user might need to
     * implement further blocking after this function returns.
     * @note None of the teleoperation participants will move until both sides are started.
     */
    void Start();

    /**
     * @brief [Blocking] Stop the teleoperation control loop and make all robots hold their pose.
     * @throw std::runtime_error if failed to stop the teleoperation control loop.
     * @note This function blocks until the control loop has stopped running and all robots in hold.
     */
    void Stop();

    /**
     * @brief [Non-blocking] Activate/deactivate teleoperation for the specified robot pair.
     * @param[in] idx Index of the robot pair to set flag for. This index is the same as the index
     * of the constructor parameter [robot_pairs_sn].
     * @param[in] activated True: allow this robot pair to move; false: hold this robot pair.
     * @throw std::invalid_argument if [idx] exceeds total number of robot pairs.
     * @note The teleoperation is deactivated by default.
     */
    void Activate(unsigned int idx, bool activated);

    /**
     * @brief [Blocking] Set Cartesian impedance properties for the specified robot pair.
     * @param[in] idx Index of the robot pair to set properties for. This index is the same as the
     * index of the constructor parameter [robot_pairs_sn].
     * @param[in] K_x_ratio Cartesian stiffness ratio. Actual K_x = K_x_ratio * K_x_nom.
     * Valid range: [0.0, 1.0].
     * @param[in] Z_x Cartesian damping ratio. Valid range: [0.3, 0.8]. The nominal (safe) value is
     * provided as default.
     * @throw std::invalid_argument if [idx] exceeds total number of robot pairs.
     * @throw std::invalid_argument if [K_x_ratio] or [Z_x] contains any value outside the valid
     * range.
     * @throw std::logic_error if teleoperation control loop is not started yet.
     * @throw std::runtime_error if failed to deliver the request to the connected robots.
     * @note This function blocks until the request is successfully delivered.
     * @note This function cannot be called before Start().
     * @warning Changing damping ratio [Z_x] to a non-nominal value may lead to performance and
     * stability issues, please use with caution.
     */
    void SetCartesianImpedance(unsigned int idx, const std::array<double, kCartDoF>& K_x_ratio,
        const std::array<double, kCartDoF>& Z_x = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7});

    /**
     * @brief [Blocking] Set reference joint positions used in the robot's null-space posture
     * control module for the specified robot pair. Call this only after Start() is triggered.
     * @param[in] idx Index of the robot pair to set null-space posture for. This index is the same
     * as the index of the constructor parameter [robot_pairs_sn].
     * @param[in] ref_positions Reference joint positions for the null-space posture control of both
     * robots in the pair: \f$ q_{ns} \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad] \f$.
     * @throw std::invalid_argument if [idx] exceeds total number of robot pairs.
     * @throw std::invalid_argument if [ref_positions] contains any value outside joint limits or
     * size of input vector does not match robot DoF.
     * @throw std::logic_error if the teleoperation control loop is not started.
     * @throw std::runtime_error if failed to deliver the request to the connected robots.
     * @note This function blocks until the request is successfully delivered.
     * @par Null-space posture control
     * Similar to human arm, a robotic arm with redundant joint-space degree(s) of freedom (DoF > 6)
     * can change its overall posture without affecting the ongoing primary task. This is achieved
     * through a technique called "null-space control". After the reference joint positions of a
     * desired robot posture are set using this function, the robot's null-space control module will
     * try to pull the arm as close to this posture as possible without affecting the primary
     * Cartesian motion-force control task.
     */
    void SetNullSpacePostures(
        unsigned int idx, const std::pair<std::vector<double>, std::vector<double>>& ref_positions);

    /**
     * @brief [Non-blocking] Set Cartesian inertia shaping for the specified robot pair.
     * @param[in] idx Index of the robot pair to set inertia shaping for. This index is the same as
     * the index of the constructor parameter [robot_pairs_sn].
     * @param[in] shaped_cart_inertia Flag to enable/disable inertia shaping and the corresponding
     * shaped inertia value for each Cartesian axis in the specified robot pair, see below for more
     * details. Valid range: > 0. Unit: \f$ [kg]:[kg·m^2] \f$.
     * @throw std::invalid_argument if [idx] exceeds total number of robot pairs.
     * @throw std::invalid_argument if [shaped_cart_inertia] contains any value outside the valid
     * range.
     * @warning Robot stability is not guaranteed if inertia shaping is enabled and the values are
     * not fine tuned, please use with caution.
     * @par Inertia Shaping
     * Cartesian-space inertia shaping algorithm utilizes sensor data to boost physical input from
     * the operator, such that the robot TCP behave as if its inertia (linear and angular) becomes
     * smaller/larger than the actual value. A small shaped inertia makes the robot TCP feel light,
     * whereas a large shaped inertia makes the robot TCP feel heavy.
     */
    void SetInertiaShaping(
        unsigned int idx, const std::array<std::pair<bool, double>, kCartDoF>& shaped_cart_inertia);

    /**
     * @brief [Non-blocking] Robot states of the specified robot pair.
     * @param[in] idx Index of the robot pair to get states for. This index is the same as the
     * index of the constructor parameter [robot_pairs_sn].
     * @return RobotStates value copy of the first and second robot respectively in the robot pair.
     * @throw std::invalid_argument if [idx] exceeds total number of robot pairs.
     */
    const std::pair<RobotStates, RobotStates> robot_states(unsigned int idx) const;

    /**
     * @brief [Non-blocking] Fault state of the specified robot pair.
     * @param[in] idx Index of the robot pair to get fault state for. This index is the same as the
     * index of the constructor parameter [robot_pairs_sn].
     * @return Fault state of the first and second robot respectively in the robot pair. True: has
     * fault; false: no fault.
     * @throw std::invalid_argument if [idx] exceeds total number of robot pairs.
     */
    const std::pair<bool, bool> fault(unsigned int idx) const;

    /**
     * @brief [Non-blocking] Whether any of the connected robots is in fault state.
     * @return True: at least one robot has fault; false: none has fault.
     */
    bool any_fault(void) const;

    /**
     * @brief [Blocking] Try to clear minor or critical fault of the robot without a power cycle.
     * @param[in] timeout_sec Maximum time in seconds to wait for the fault to be successfully
     * cleared. Normally, a minor fault should take no more than 3 seconds to clear, and a critical
     * fault should take no more than 30 seconds to clear.
     * @return True: successfully cleared fault; false: failed to clear fault.
     * @return For each element in the pair vector, true: successfully cleared fault or no fault for
     * this robot, false: failed to clear fault for this robot. The pattern of the pair vector is
     * the same as the constructor parameter [robot_pairs_sn].
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the fault is successfully cleared or [timeout_sec] has
     * elapsed.
     * @warning Clearing a critical fault through this function without a power cycle requires a
     * dedicated device, which may not be installed in older robot models.
     */
    std::vector<std::pair<bool, bool>> ClearFault(unsigned int timeout_sec = 30);

    /**
     * @brief [Non-blocking] Current reading from all digital input ports (16 on the control box + 2
     * inside the wrist connector) of the specified robot pair.
     * @param[in] idx Index of the robot pair to read from. This index is the same as the index
     * of the constructor parameter [robot_pairs_sn].
     * @return A pair of boolean arrays whose index corresponds to that of the digital input ports
     * of the corresponding robot in the pair. True: port high; false: port low.
     * @throw std::invalid_argument if [idx] exceeds total number of robot pairs.
     */
    const std::pair<std::array<bool, kIOPorts>, std::array<bool, kIOPorts>> digital_inputs(
        unsigned int idx) const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace tdk
} // namespace flexiv
