/**
 * @file transparent_cartesian_teleop_wan.hpp
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
 * @brief Teleoperation control interface that represents leader or follower robots in transparent
 * teleoperation over WAN (TCP/IP). Teleoperation is established between leader and follower
 * robots when they are controlled by an instance of this interface, with one set as
 * TCP server and the other set as TCP client via the parameter [is_tcp_server] in NetworkCfg.
 * @warning This is highly transparent Cartesian teleoperation and therefore requires the robot to
 * be configured with a flange-end FT sensor before using this class.
 * @note In the documentation of this class, "leader robot" refers to the robot which operated by a
 * human operator during teleoperation; "follower robot" refers to the robot interacts with
 * workpieces.
 */
class TransparentCartesianTeleopWAN
{
public:
    /**
     * @brief [Blocking] Create an instance of the control interface. More than one pair of
     * teleoperated robots can be controlled at the same time, see parameter [robot_pairs_sn].
     * @param[in] robot_pairs_sn Serial number of all leader-follower pairs to run teleoperation on.
     * Each pair in the vector represents a pair of bilaterally teleoperated robots. For example,
     * provide 2 pairs of robot serial numbers to start a dual-arm teleoperation that involves 2
     * pairs of robots. The accepted formats are: "Rizon 4s-123456" and "Rizon4s-123456". In each
     * pair, the first robot is referred to as the "leader robot", which operated by human operator
     * during teleoperation. The second robot is referred to as the "follower robot", which
     * interacts with the workpiece.
     * @param [in] role The role in transparent teleoperation over WAN. There are two types of
     * participants in teleoperation , one is the "leader", which operated by a human during
     * teleoperation. The other is referred to as the "follower", which interacts with
     * the environment during teleoperation.
     * @param[in] network_cfg Network configuration including server/client role configuration, IPv4
     * address and listening port.
     * @throw std::invalid_argument if the format robot_sn or any IPv4 address or listening port is
     * invalid.
     * @throw std::runtime_error if error occurred during construction.
     * @throw std::logic_error if one of the connected robots does not have a valid TDK license; or
     * the version of this TDK library is incompatible with one of the connected robots; or model of
     * any connected robot is not supported; or there are multiple instantiated TDK objects.
     * @warning This constructor blocks until the connection with the robot is established and
     * initialization sequence is successfully finished. It does not wait for the WAN connection to
     * be established.
     */
    TransparentCartesianTeleopWAN(
        const std::vector<std::pair<std::string, std::string>>& robot_pairs_sn,
        flexiv::tdk::Role role, const NetworkCfg& network_cfg);
    virtual ~TransparentCartesianTeleopWAN();

    //========================================= ACCESSORS ==========================================
    /**
     * @brief [Non-blocking] Check the TCP connection status and retrieve the average message
     * latency.
     *
     * This function estimates the average TCP message latency (in milliseconds) using an internal
     * sliding-window filter to suppress occasional spikes. The result is returned through the
     * output parameter [latency_ms].
     *
     * The return value and latency interpretation are as follows:
     * - **Case 1:** `latency_ms` is a very large positive number → Connection not yet established.
     *   Returns **false**.
     * - **Case 2:** `latency_ms` is within [0, 350] milliseconds → Connection established.
     *   Returns **true**.
     * - **Case 3:** `latency_ms` is negative → The system clocks of the two computers are not
     *   properly synchronized.
     *   Returns **false**. In this case, ensure both computers run the `chrony` service to
     *   synchronize their system time.
     *
     * @param[in] idx Index of the robot pair. This corresponds to the index of the constructor
     * parameter [robot_pairs_sn].
     * @param[out] latency_ms Average TCP message latency in milliseconds.
     * @return True if a valid connection is established and the average latency is within the
     *         acceptable range [0, 350] ms. False otherwise.
     * @throw std::invalid_argument if [idx] is out of range.
     * @warning
     * - If [latency_ms] > 200 ms, the connection quality is poor and may cause delayed feedback or
     *   command responses.
     * - If [latency_ms] > 350 ms, teleoperation will be disconnected, and follower robots will hold
     *   their pose until incoming message latency is in valid range.
     */
    bool CheckTcpConnectionLatency(unsigned int idx, double& latency_ms);

    /**
     * @brief [Non-blocking] Robot states of the current role.
     * @param[in] idx Index of the robot to get states for current role. This index is the same as
     * the index of the constructor parameter [robot_pairs_sn].
     * @throw std::invalid_argument if [idx] is outside the valid range.
     * @return RobotStates value copy.
     */
    const RobotStates robot_states(unsigned int idx) const;

    /**
     * @brief [Non-blocking] Whether the current role is in fault state.
     * @param[in] idx Index of the robot to get fault state for. This index is the same as the
     * index of the constructor parameter [robot_pairs_sn].
     * @throw std::invalid_argument if [idx] is outside the valid range.
     * @return True: robot has fault; false: robot normal.
     */
    bool fault(unsigned int idx) const;

    /**
     * @brief [Non-blocking] Current reading from all digital input ports (16 on the control box + 2
     * inside the wrist connector) of the current role in specified robot pair.
     * @param[in] idx Index of the robot pair to read from. This index is the same as the index
     * of the constructor parameter [robot_pairs_sn].
     * @throw std::invalid_argument if [idx] is outside the valid range.
     * @return A boolean array whose index corresponds to that of the digital input ports.
     * True: port high; false: port low.
     */
    const std::array<bool, kIOPorts> digital_inputs(unsigned int idx) const;

    /**
     * @brief [Non-blocking] Whether teleop process has stopped. After teleop is started, the teleop
     * process may stop for certain reasons. If it stops, the user needs to check the reason, then
     * call Init() again and then call Start() again. Possible reasons include: a) the user actively
     * called Stop(); b) the robot became not operational for certain reasons; c) the control mode
     * did not match; d) the network connection between the user's computer and the control box was
     * unstable; e) other possible reasons
     * @param[in] idx Index of the robot pair to init. This index is the same as the index
     * of the constructor parameter [robot_pairs_sn].
     * @throw std::invalid_argument if [idx] is outside the valid range.
     * @return True: stopped; false: started.
     */
    bool stopped(unsigned int idx) const;

    //==================================== TELEOP LIFECYCLE ====================================
    /**
     * @brief [Blocking] Get current role ready for teleoperation. The following actions will
     * happen in sequence: a) enable robot if it's servo off, b) zero force/torque sensors, c) stop
     * the robot and init teleop control params.
     * @param[in] limit_wrist_singular Whether to limit wrist singularity. If twisted towards the
     * wrist singularity zone, it may cause the robot to report error.
     * @throw std::runtime_error if the initialization sequence failed.
     * @note This function blocks until the initialization sequence is finished.
     * @warning This process involves sensor zeroing, please make sure the robot is not in contact
     * with anything during the process.
     * @see Role
     */
    void Init(bool limit_wrist_singular = true);

    /**
     * @brief [Non-Blocking] Start the teleoperation control loop for current roles (leaders or
     * followers) specified with 'role' in constructor.
     * @throw std::logic_error if initialization sequence hasn't been triggered yet using Init().
     * @note Teleop will only work properly when the following conditions are met: a) The control
     * loops of leaders and followers start normally b) TCP connection successfully established and
     * message latency is in valid range. c) Engaged by the leader
     */
    void Start();

    /**
     * @brief [Blocking] Stop the teleoperation control loop and make all robots hold their pose.
     * @throw std::runtime_error if failed to stop the robots.
     * @note If users want to control a robot individually, first need to call Stop() to stop
     * the teleop process. Whenever users want to restart teleop, the restart process should be call
     * Init() first and then call Start().
     * @note This function blocks until all robots stopped in hold. If users do NOT want to stop the
     * teleop process but temporarily pause teleoperation, users can lock/unlock all the axes, which
     * is non-blocking. See SetAxisLockCmd.
     */
    void Stop();

    /**
     * @brief [Blocking] Get current role in specified pair ready for teleoperation. The
     * following actions will happen in sequence: a) enable robot if it's servo off, b) zero
     * force/torque sensors, c) stop the robot and init teleop control params.
     * @param[in] idx Index of the robot pair to init. This index is the same as the index
     * of the constructor parameter [robot_pairs_sn].
     * @param[in] limit_wrist_singular Whether to limit wrist singularity. If twisted towards the
     * wrist singularity zone, it may cause the robot to report error.
     * @throw std::runtime_error if the initialization sequence failed.
     * @note This function blocks until the initialization sequence is finished.
     * @warning This process involves sensor zeroing, please make sure the robot is not in contact
     * with anything during the process.
     * @see Role
     */
    void InitWithIdx(unsigned int idx, bool limit_wrist_singular = true);

    /**
     * @brief [Non-Blocking] Start the teleoperation control loop for the specified robot pair.
     * @param[in] idx Index of the robot pair to start. This index is the same as the index
     * of the constructor parameter [robot_pairs_sn].
     * @throw std::logic_error if initialization sequence hasn't been triggered yet using Init() or
     * InitWithIdx().
     * @note Teleop will only work properly when the following conditions are met: a) The control
     * loops of leaders and followers start normally b) TCP connection successfully established and
     * message latency is in valid range. c) Engaged by the leader
     */
    void StartWithIdx(unsigned int idx);

    /**
     * @brief [Blocking] Stop the teleoperation control loop and make robot hold its pose for
     * current role in the specified robot pair.
     * @param[in] idx Index of the robot pair to stop. This index is the same as the index
     * of the constructor parameter [robot_pairs_sn].
     * @throw std::runtime_error if failed to stop the robots.
     * @note If users want to control a robot individually, first need to call StopWithIdx() to stop
     * the teleop process. Whenever users want to restart teleop, the restart process should be call
     * InitWithIdx() first and then call StartWithIdx().
     * @note This function blocks until all robots stopped in hold. If users do NOT want to stop the
     * teleop process but temporarily pause teleop, users can lock/unlock all the axes, which is
     * non-blocking. See SetAxisLockCmd.
     */
    void StopWithIdx(unsigned int idx);

    //==================================== TELEOP CONTROL ====================================
    /**
     * @brief [Non-blocking] Engage/disengage the leader and follower robot in the specified robot
     * pair. TransparentCartesianTeleopWAN supports teleop leader and follower robots in different
     * configurations. When disengaged, the operators can move the leader robot to the center of the
     * workspace or re-orientated for better ergonomics. Meanwhile, the follower robot will remain
     * stationary. When engaged again, the follower robot will only mimics the leader's relative
     * motion instead of simply mirroring the joint or Cartesian pose.
     * @param[in] idx Index of the robot pair to set flag for. This index is the same as the index
     * of the constructor parameter [robot_pairs_sn].
     * @param[in] engaged True to engage the teleop, false to disengage.
     * @throw std::invalid_argument if [idx] is outside the valid range.
     * @throw std::logic_error if the teleoperation control loop is not started or the instance is
     * not initialized as leader robot.
     * @note The teleoperation will keep disengaged by default.
     */
    void Engage(unsigned int idx, bool engaged);

    /**
     * @brief [Blocking] Set reference joint positions used in the robot's null-space posture
     * control module for the current role in the robot pairs.
     * @param[in] idx Index of the robot pair to set null-space posture for current role. This index
     * is the same as the index of the constructor parameter [robot_pairs_sn].
     * @param[in] ref_positions Reference joint positions for the null-space posture control of
     * specified robot in the pair: \f$ q_{ns} \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad]
     * \f$.
     * @throw std::invalid_argument if [ref_joint_positions] contains any value outside joint limits
     * or size of input vector does not match robot DoF.
     * @throw std::invalid_argument if [idx] is outside the valid range.
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
    void SetNullSpacePosture(unsigned int idx, const std::vector<double>& ref_joint_positions);

    /**
     * @brief [Non-blocking] Set maximum contact wrench for the current role in the robot pairs. The
     * controller will regulate its output to maintain contact wrench (force and moment) with the
     * environment under the set values.
     * @param[in] idx Index of the robot pair to set null-space posture for current role. This index
     * is the same as the index of the constructor parameter [robot_pairs_sn].
     * @param[in] max_wrench Maximum contact wrench (force and moment): \f$ F_max \in \mathbb{R}^{6
     * \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ maximum force and \f$
     * \mathbb{R}^{3 \times 1} \f$ maximum moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$. Unit:
     * \f$ [N]~[Nm] \f$.
     * @throw std::invalid_argument if [max_wrench] contains any negative value.
     * @throw std::invalid_argument if [idx] is outside the valid range.
     * @throw std::logic_error if teleop is not initialized.
     */
    void SetMaxContactWrench(unsigned int idx, const std::array<double, kCartDoF>& max_wrench);

    //======================================= SYSTEM CONTROL =======================================
    /**
     * @brief [Blocking] Try to clear minor or critical fault for current role (leaders or
     * followers) without a power cycle.
     * @param[in] timeout_sec Maximum time in seconds to wait for the fault to be successfully
     * cleared. Normally, a minor fault should take no more than 3 seconds to clear, and a critical
     * fault should take no more than 30 seconds to clear.
     * @return True: successfully cleared fault; false: failed to clear fault.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the fault is successfully cleared or [timeout_sec] has
     * elapsed.
     * @warning Clearing a critical fault through this function without a power cycle requires a
     * dedicated device, which may not be installed in older robot models.
     */
    std::vector<bool> ClearFault(unsigned int timeout_sec = 30);

    /**
     * @brief [Non-blocking] Pointer to the underlying rdk::Robot instance of the current role.
     * @param[in] idx Index of the robot to get rdk instance for. This index is the same as the
     * index of the constructor parameter [robot_pairs_sn].
     * @throw std::invalid_argument if [idx] is outside the valid range.
     * @return Pointer to rdk::Robot instance.
     */
    std::shared_ptr<Robot> instance(unsigned int idx) const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace tdk
} // namespace flexiv
