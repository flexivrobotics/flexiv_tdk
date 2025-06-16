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
 * @brief Teleoperation control interface that represents leader or follower robots in teleoperation
 * over WAN (Internet). Teleoperation is established between two robots when each of them is
 * controlled by an instance of this interface, with one set as TCP server and the other set as TCP
 * client via the constructor parameter [is_tcp_server].
 * @warning This is highly transparent Cartesian teleoperation and therefore requires the
 * robot to be configured with a flange-end FT sensor before using this class.
 * @note In the documentation of this class, "leader robot" refers to the robot interacts with human
 * operator; "follower robot" refers to the robot interacts with workpieces.
 * @par TCP Server and Client Configuration
 * In a teleoperation-over-WAN setup, there are one robot + one edge device on each side of the
 * teleoperation. One edge device needs to function as a TCP server while the other device functions
 * as a TCP client. It does not matter which side is configured as TCP server or client. However,
 * while the edge device for TCP client doesn't need any additional configuration other than
 * connecting to the Internet, the TCP server needs to complete the following additional steps:
 *
 * 1. In the settings of the network router that the edge device for TCP server is connected to,
 * enable NAT (network address translation). This is usually enabled by default on modern routers.
 * 2. Note down the private (WAN) IPv4 address assigned to the edge device for TCP server.
 * 3. In the router settings, add TCP port forwarding rule for the IPv4 address noted in step 2. The
 * port number can be set to any unoccupied one. Use this port number as the [listening_port]
 * constructor parameter for BOTH sides of teleoperation.
 * 4. On the edge device for TCP server, open https://whatismyipaddress.com/ and note down its
 * public IPv4 address. Use this address as the [public_ipv4_address] constructor parameter for BOTH
 * sides of teleoperation.
 */
class TransparentCartesianTeleopWAN
{
public:
    /**
     * @brief [Blocking] Create an instance of the control interface. More than one pair of
     * teleoperated robots can be controlled at the same time, see parameter [robot_pairs_sn].
     * @param[in] robot_sn Serial number of the local robot. The accepted formats are:
     * "Rizon 4s-123456" and "Rizon4s-123456". There are two participants in teleoperation , one is
     * the "leader robot", which operated by a human during teleoperation. The other robot is
     * referred to as the "follower robot", which interacts with the remote environment during
     * teleoperation. The role can be configured during initial process using Init().
     * @param[in] network_cfg Network configuration including server/client, IPv4 address and
     * listening port.
     * @throw std::invalid_argument if the format robot_sn or any IPv4 address is invalid.
     * @throw std::runtime_error if error occurred during construction.
     * @throw std::logic_error if one of the connected robots does not have a valid TDK license; or
     * the version of this TDK library is incompatible with one of the connected robots; or model of
     * any connected robot is not supported.
     * @warning This constructor blocks until the initialization sequence is successfully finished
     * and connection with all robots is established.
     * @warning A FT sensor is required to installed on both robots, please NOT use this class if FT
     * sensor is not configured.
     */
    TransparentCartesianTeleopWAN(const std::string& robot_sn, const NetworkCfg& network_cfg);
    virtual ~TransparentCartesianTeleopWAN();

    /**
     * @brief [Blocking] Get all robots ready for teleoperation. The following actions will
     * happen in sequence: a) enable robot, b) zero force/torque sensors.
     * @param [in] role The role in transparent teleoperation over WAN.
     * @param[in] limit_wrist_singular Whether to limit wrist singularity. If twisted in the wrist
     * singularity zone, it may cause the robot to report error.
     * @throw std::runtime_error if the initialization sequence failed.
     * @note This function blocks until the initialization sequence is finished.
     * @warning This process involves sensor zeroing, please make sure the robot is not in contact
     * with anything during the process.
     * @see Role
     */
    void Init(flexiv::tdk::Role role, bool limit_wrist_singular = true);

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
     * @brief [Non-blocking] Engage/disengage the leader and follower robot.
     * TransparentCartesianTeleopWAN supports teleop leader and follower robots in different
     * configurations. When disengaged, the operators can move the leader robot to the center of the
     * workspace or re-orientated for better ergonomics. Meanwhile, the follower robot will remain
     * stationary. When engaged again, the follower robot will only mimics the leader's relative
     * motion instead of simply mirroring the pose.
     * @param[in] engaged True to engage the teleop, false to disengage.
     * @throw std::logic_error if the teleoperation control loop is not started or the instance is
     * not initialized as leader robot.
     * @note The teleoperation will keep disengaged by default.
     */
    void Engage(bool engaged);

    /**
     * @brief [Blocking] Set reference joint positions used in the robot's null-space posture
     * control module for the current role specified in Init().
     * @param[in] ref_positions Reference joint positions for the null-space posture control of both
     * robots in the pair: \f$ q_{ns} \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad] \f$.
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
    void SetNullSpacePosture(const std::vector<double>& ref_joint_positions);

    /**
     * @brief [Non-blocking] Set maximum contact wrench for the current role. The controller will
     * regulate its output to maintain contact wrench (force and moment) with the environment under
     * the set values.
     * @param[in] max_wrench Maximum contact wrench (force and moment): \f$ F_max \in \mathbb{R}^{6
     * \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ maximum force and \f$
     * \mathbb{R}^{3 \times 1} \f$ maximum moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$. Unit:
     * \f$ [N]~[Nm] \f$.
     * @throw std::invalid_argument if [max_wrench] contains any negative value.
     * @throw std::logic_error if teleop is not initialized.
     */
    void SetMaxContactWrench(const std::array<double, kCartDoF>& max_wrench);

    /**
     * @brief [Non-blocking] Robot states of the current role.
     * @return RobotStates value copy.
     */
    const RobotStates robot_states() const;

    /**
     * @brief [Non-blocking] Whether the current role is in fault state.
     * @return True: robot has fault; false: robot normal.
     */
    bool fault() const;

    /**
     * @brief [Blocking] Try to clear minor or critical fault of the current role without a power
     * cycle.
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
    bool ClearFault(unsigned int timeout_sec = 30);

    /**
     * @brief [Non-blocking] Current reading from all digital input ports (16 on the control box + 2
     * inside the wrist connector) of the current role.
     * @return A boolean array whose index corresponds to that of the digital input ports.
     * True: port high; false: port low.
     */
    const std::array<bool, kIOPorts> digital_inputs() const;

    /**
     * @brief [Non-blocking] Pointer to the underlying rdk::Robot instance of the current role.
     * @return Pointer to rdk::Robot instance.
     */
    std::shared_ptr<Robot> instance() const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace tdk
} // namespace flexiv
