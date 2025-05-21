/**
 * @file transparent_cartesian_teleop_lan.hpp
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
 * @brief Teleoperation control interface to run Cartesian-space Rizon4s-Rizon4s teleoperation for
 * one or more pairs of robots connected to the same LAN.  It performs synchronized, force guided
 * real-time motions and provide the operator with high-fidelity haptic feedback.
 */
class TransparentCartesianTeleopLAN
{
public:
    /**
     * @brief [Blocking] Create an instance of the control interface. More than one pair of
     * teleoperated robots can be controlled at the same time, see parameter [robot_pairs_sn].
     * @param[in] robot_pairs_sn Serial number of all robot pairs to run teleoperation on. Each pair
     * in the vector represents a pair of bilaterally teleoperated robots. For example, provide 2
     * pairs of robot serial numbers to start a dual-arm teleoperation that involves 2 pairs of
     * robots. The accepted formats are: "Rizon 4s-123456" and "Rizon4s-123456". In each pair, the
     * first robot is referred to as the "local robot", which interacts with human hands. The second
     * robot is referred to as the "remote robot", which interacts with the workpiece.
     * @param[in] network_interface_whitelist Limit the network interface(s) that can be used to try
     * to establish connection with the specified robot. The whitelisted network interface is
     * defined by its associated IPv4 address. For example, {"10.42.0.1", "192.168.2.102"}. If left
     * empty, all available network interfaces will be tried when searching for the specified robot.
     * @throw std::invalid_argument if the format of any element in [robot_pairs_sn] is invalid; or
     * the size of [robot_pairs_sn] exceeds the allowed number.
     * @throw std::runtime_error if error occurred during construction.
     * @throw std::logic_error if one of the connected robots does not have a valid TDK license; or
     * the version of this TDK library is incompatible with one of the connected robots; or model of
     * any connected robot is not supported.
     * @warning This constructor blocks until the initialization sequence is successfully finished
     * and connection with all robots is established.
     * @warning A FT sensor is required to installed on the robot, please NOT use this class if FT
     * sensor is configured.
     */
    TransparentCartesianTeleopLAN(
        const std::vector<std::pair<std::string, std::string>>& robot_pairs_sn,
        const std::vector<std::string>& network_interface_whitelist = {});
    virtual ~TransparentCartesianTeleopLAN();

    /**
     * @brief [Blocking] Get all robots ready for teleoperation. The following actions will
     * happen in sequence: a) enable robot, b) zero force/torque sensors.
     * @param[in] limit_wrist_singular Whether to limit wrist singularity. If twisted in the wrist
     * singularity zone, it may cause the robot to report error.
     * @throw std::runtime_error if the initialization sequence failed.
     * @note This function blocks until the initialization sequence is finished.
     * @warning This process involves sensor zeroing, please make sure the robot is not in contact
     * with anything during the process.
     */
    void Init(bool limit_wrist_singular = true);

    /**
     * @brief [Blocking] Start the teleoperation control loop.
     * @throw std::runtime_error if failed to start the teleoperation control loop.
     * @throw std::logic_error if initialization sequence hasn't been triggered yet using Init().
     * @note This function blocks until the control loop has started running. The user might need to
     * implement further blocking after this function returns.
     * @note None of the teleoperation participants will move until both sides are started.
     */
    void Start();

    /**
     * @brief [Blocking] Stop the teleoperation control loop and make all robots hold their pose.
     * @throw std::runtime_error if failed to stop the teleoperation control loop.
     * @note This function blocks until the control loop has stopped running and all robots in hold.
     * @note If you do NOT want to stop the control loop but temporarily pause the teleop, you can
     * lock/unlock all the axes, which is non-blocking. See SetAxisLockCmd.
     */
    void Stop();

    /**
     * @brief [Non-blocking] Engage/disengage the local and remote robot.
     * TransparentCartesianTeleopLAN supports teleop local and remote robots in different
     * configurations. When disengaged, the operators can move the local robot to the center of the
     * workspace or re-orientated for better ergonomics. Meanwhile, the remote robot will remain
     * stationary. When engaged again, the remote robot will only mimics the local's relative motion
     * instead of simply mirroring the pose.
     * @param[in] idx Index of the robot pair to set flag for. This index is the same as the index
     * of the constructor parameter [robot_pairs_sn].
     * @param[in] engaged True to engage the teleop, false to disengage.
     * @note The teleop will keep disengaged by default.
     */
    void Engage(unsigned int idx, bool engaged);

    /**
     * @brief [Blocking] Set reference joint positions used in the robot's null-space posture
     * control module for the specified local robot. By "local robot" we mean the first robot in
     * [robot_pairs_sn], which interacts with human hands. Call this only after Start() is
     * triggered.
     * @param[in] idx Index of the robot pair to set null-space posture for. This index is the same
     * as the index of the constructor parameter [robot_pairs_sn].
     * @param[in] ref_positions Reference joint positions for the null-space posture control of both
     * robots in the pair: \f$ q_{ns} \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad] \f$.
     * @throw std::invalid_argument if [idx] exceeds total number of robot pairs.
     * @throw std::invalid_argument if [ref_joint_positions] contains any value outside joint limits
     * or size of input vector does not match robot DoF.
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
    void SetLocalNullSpacePosture(unsigned int idx, const std::vector<double>& ref_joint_positions);

    /**
     * @brief [Blocking] Set reference joint positions used in the robot's null-space posture
     * control module for the specified remote robot. By "remote robot" we mean the second robot in
     * [robot_pairs_sn], which interacts with workpiece. Call this only after Start() is
     * triggered.
     * @param[in] idx Index of the robot pair to set null-space posture for. This index is the same
     * as the index of the constructor parameter [robot_pairs_sn].
     * @param[in] ref_positions Reference joint positions for the null-space posture control of both
     * robots in the pair: \f$ q_{ns} \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad] \f$.
     * @throw std::invalid_argument if [idx] exceeds total number of robot pairs.
     * @throw std::invalid_argument if [ref_joint_positions] contains any value outside joint limits
     * or size of input vector does not match robot DoF.
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
    void SetRemoteNullSpacePosture(
        unsigned int idx, const std::vector<double>& ref_joint_positions);

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

    /**
     * @brief [Non-blocking] Set maximum contact wrench for the remote robot of specified robot
     * pair. The controller will regulate its output to maintain contact wrench (force and moment)
     * with the environment under the set values.
     * @param[in] idx Index of the robot pair to read from. This index is the same as the index
     * of the constructor parameter [robot_pairs_sn].
     * @param[in] max_wrench Maximum contact wrench (force and moment): \f$ F_max \in \mathbb{R}^{6
     * \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ maximum force and \f$
     * \mathbb{R}^{3 \times 1} \f$ maximum moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$. Unit:
     * \f$ [N]~[Nm] \f$.
     * @throw std::invalid_argument if [max_wrench] contains any negative value.
     * @throw std::logic_error if teleop is not initialized.
     */
    void SetRemoteMaxContactWrench(
        unsigned int idx, const std::array<double, kCartDoF>& max_wrench);

    /**
     * @brief [Non-blocking] Set the repulsive force in World or Tcp frame of the remote robot.
     * @param[in] idx Index of the robot pair to set for. This index is the same as the
     * index of the constructor parameter [robot_pairs_sn].
     * @param[in] repulsive_force The virtual repulsive force that will applied on the remote
     * robot in the specified robot pair [idx].: \f$ repulsiveF \in \mathbb{R}^{3 \times 1} \f$.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ repulsive force : \f$ [f_x, f_y, f_z]^T \f$.
     * Unit: \f$ [N] \f$.
     *  @param[in] in_world Flag to indicate that the repulsive force is in World frame or Tcp
     * frame. true in World frame, false in Tcp frame.
     * @note This virtual repulsive wrench will only work on those unlocked axis, and will be
     * ignored if the manipulability is not good enough.
     * @warning Overlarge or discontinuous force can cause the robot to report errors. Please use
     * this interface carefully. To use this API, users should possess robot programming
     * capabilities.
     * @see SetLocalAxisLockCmd
     * @see GetLocalAxisLockState
     */
    void SetRepulsiveForce(
        unsigned int idx, const std::array<double, 3>& repulsive_force, bool in_world = true);

    /**
     * @brief[Non-blocking] Set the wrench feedback scaling factor.
     * @param[in] idx Index of the robot pair to set for. This index is the same as the
     * index of the constructor parameter [robot_pairs_sn].
     * @param[in] factor This coefficient will scale the feedback wrench of the remote robot.
     * Scale factor greater than 1 means that the external force received by the remote robot is
     * amplified, otherwise it will be reduce. Setting scale to zero means no wrench feedback
     * and 1 means 100% transparency. Valid range: [0, kMaxWrenchFeedbackScale]
     * @throw std::invalid_argument if input scale is outside the valid range.
     * @warning Only when the user ensures that the interaction force between the remote robot
     * and workpiece is very small, such as when operating a very soft object, do they need to
     * set the factor to be greater than 1. Or to use [SetRemoteMaxContactWrench] to limit the
     * maximum contact wrench. If the object in contact with the remote robot has high stiffness,
     * please set the factor very carefully. The higher the scale, the greater the force feedback to
     * the local robot will be. Using a scaling factor of 1 is recommended.
     * @see SetRemoteMaxContactWrench
     * @see kMaxWrenchFeedbackScale
     */
    void SetWrenchFeedbackScalingFactor(unsigned int idx, double factor = 1.0);

    /**
     * @brief [Non-blocking] Set the local robot axis locking command.
     * @param[in] idx Index of the robot pair to set commands for. This index is the same as the
     * index of the constructor parameter [robot_pairs_sn].
     * @param[in] cmd User input command to lock the motion of the specified axis in the reference
     * coordinate.
     */
    void SetAxisLockCmd(unsigned int idx, const AxisLock& cmd);

    /**
     * @brief [Non-blocking] Get the local robot axis locking status
     * @param[in] idx Index of the robot pair to get state for. This index is the same as the
     * index of the constructor parameter [robot_pairs_sn].
     * @param[out] data Current axis locking state of local robot.
     */
    void GetAxisLockState(unsigned int idx, AxisLock& data);

    /**
     * @brief [Non-blocking] Get the local robot axis locking status
     * @param[in] idx Index of the robot pair to get states for. This index is the same as the
     * index of the constructor parameter [robot_pairs_sn].
     * @warning This fuction is less efficient than the other overloaded one as additional runtime
     * memory allocation and data copying are performed.
     * @return AxisLock
     */
    AxisLock GetAxisLockState(unsigned int idx);

    /**
     * @brief [Non-blocking] Pointers to the underlying rdk::Robot instances of the robot pair.
     * @param[in] idx Index of the robot pair to get states for. This index is the same as the
     * index of the constructor parameter [robot_pairs_sn].
     * @return Respective pointers to rdk::Robot instances.
     */
    std::pair<std::shared_ptr<Robot>, std::shared_ptr<Robot>> instances(unsigned int idx) const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace tdk
} // namespace flexiv
