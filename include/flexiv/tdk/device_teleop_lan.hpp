/**
 * @file device_teleop_lan.hpp
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 */
#pragma once

#include "data.hpp"
#include <string>
#include <memory>

#include <flexiv/rdk/robot.hpp>
namespace flexiv {
namespace tdk {

using namespace rdk;

/**
 * @brief Teleoperation control interface to use device to teleop one or more robots connected to
 * the same LAN.
 */
class DeviceTeleopLan
{
public:
    /**
     * @brief [Blocking] Create an instance of the control interface. More than one pair of
     * [device-robot] can be controlled at the same time, see parameter [robot_sn_vec].
     * @param[in] robot_sn_vec Serial number of all robots to run teleoperation on. Each
     * robot can be bind to an external device. For example, provide 2 robot serial numbers to start
     * a dual-arm teleoperation that involves 2 robots and 2 devices. The accepted formats are:
     * "Rizon 4s-123456" and "Rizon4s-123456".
     * @param[in] network_interface_whitelist Limit the network interface(s) that can be used to try
     * to establish connection with the specified robot. The whitelisted network interface is
     * defined by its associated IPv4 address. For example, {"10.42.0.1", "192.168.2.102"}. If left
     * empty, all available network interfaces will be tried when searching for the specified robot.
     * @throw std::invalid_argument if the format of any element in [robot_sn_vec] is invalid.
     * @throw std::runtime_error if error occurred during construction.
     * @throw std::logic_error if one of the connected robots does not have a valid TDK license; or
     * the version of this TDK library is incompatible with one of the connected robots; or model of
     * any connected robot is not supported.
     * @warning This constructor blocks until the initialization sequence is successfully finished
     * and connection with all robots is established.
     */
    DeviceTeleopLan(const std::vector<std::string>& robot_sn_vec,
        const std::vector<std::string>& network_interface_whitelist = {});
    virtual ~DeviceTeleopLan();

    /**
     * @brief [Blocking] Get all robots ready for teleoperation. The following actions will
     * happen in sequence: a) enable robot,b) clear fault if any, c) zero force/torque sensors.
     * @throw std::runtime_error if the initialization sequence failed.
     * @note This function blocks until the initialization sequence is finished.
     * @warning This process involves sensor zeroing, please make sure the robot is not in contact
     * with anything during the process.
     */
    void Init();

    /**
     * @brief [Blocking] Start the teleoperation control loop.
     * @param [in] cmds Motion control commands data for all robots. The size of the vector should
     * match with the robot_sn_vec.
     * @throw std::runtime_error if failed to start the teleoperation control loop.
     * @throw std::logic_error if initialization sequence hasn't been triggered yet using Init().
     * @throw std::logic_error if size of cmds failed to match with the [robot_sn_vec].
     * @note This function blocks until the control loop has started running. The user might need to
     * implement further blocking after this function returns.
     */
    void Start(const std::vector<std::shared_ptr<MotionControlCmds>>& cmds);

    /**
     * @brief [Blocking] Stop the teleoperation control loop and make all robots hold their pose.
     * @throw std::runtime_error if failed to stop the teleoperation control loop.
     * @note This function blocks until the control loop has stopped running and all robots in hold.
     */
    void Stop();

    /**
     * @brief [Non-blocking] Set maximum contact wrench for the  specified robot. The controller
     * will regulate its output to maintain contact wrench (force and moment) with the environment
     * under the set values.
     * @param[in] idx Index of the robot to read from. This index is the same as the index
     * of the constructor parameter [robot_sn_vec].
     * @param[in] max_wrench Maximum contact wrench (force and moment): \f$ F_max \in \mathbb{R}^{6
     * \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ maximum force and \f$
     * \mathbb{R}^{3 \times 1} \f$ maximum moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$. Unit:
     * \f$ [N]~[Nm] \f$.
     * @throw std::invalid_argument if [max_wrench] contains any negative value.
     * @throw std::logic_error if teleop is not initialized.
     */
    void SetRobotMaxContactWrench(unsigned int idx, const std::array<double, kCartDoF>& max_wrench);

    /**
     * @brief [Blocking] Set reference joint positions used in the robot's null-space posture
     * control module for the specified robot. Call this only after Start() is triggered.
     * @param[in] idx Index of the robot to set null-space posture for. This index is the same
     * as the index of the constructor parameter [robot_pairs_sn].
     * @param[in] ref_positions Reference joint positions for the null-space posture control of
     * robot: \f$ q_{ns} \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad] \f$.
     * @throw std::invalid_argument if [idx] exceeds total number of robots.
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
    void SetNullSpacePostures(unsigned int idx, const std::vector<double>& ref_positions);

    /**
     * @brief [Non-blocking] Robot states of the specified robot.
     * @param[in] idx Index of the robot to get states for. This index is the same as the
     * index of the constructor parameter [robot_pairs_sn].
     * @return RobotStates value copy.
     * @throw std::invalid_argument if [idx] exceeds total number of robots.
     */
    const RobotStates robot_states(unsigned int idx) const;

    /**
     * @brief [Non-blocking] Fault state of the specified robot.
     * @param[in] idx Index of the robot to get fault state for. This index is the same as the
     * index of the constructor parameter [robot_sn_vec].
     * @return Fault state of the robot respectively in the robot vector. True: has
     * fault; false: no fault.
     * @throw std::invalid_argument if [idx] exceeds total number of robots.
     */
    bool fault(unsigned int idx) const;

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
     * the same as the constructor parameter [robot_sn_vec].
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the fault is successfully cleared or [timeout_sec] has
     * elapsed.
     * @warning Clearing a critical fault through this function without a power cycle requires a
     * dedicated device, which may not be installed in older robot models.
     */
    std::vector<bool> ClearFault(unsigned int timeout_sec = 30);

    /**
     * @brief [Non-blocking] Current reading from all digital input ports (16 on the control box + 2
     * inside the wrist connector) of the specified robot.
     * @param[in] idx Index of the robot to read from. This index is the same as the index
     * of the constructor parameter [robot_sn_vec].
     * @return A pair of boolean arrays whose index corresponds to that of the digital input ports
     * of the corresponding robot in the pair. True: port high; false: port low.
     * @throw std::invalid_argument if [idx] exceeds total number of robots.
     */
    const std::array<bool, kIOPorts> digital_inputs(unsigned int idx) const;

    /**
     * @brief [Non-blocking] Pointer to the underlying rdk::Robot instances of the robot.
     * @param[in] idx Index of the robot pair to get states for. This index is the same as the
     * index of the constructor parameter [robot_sn_vec].
     * @return Pointer to rdk::Robot instances.
     */
    std::shared_ptr<Robot> instance(unsigned int idx) const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace tdk
} // namespace flexiv
