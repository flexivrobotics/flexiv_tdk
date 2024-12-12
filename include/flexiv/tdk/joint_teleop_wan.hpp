/**
 * @file joint_teleop_wan.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */
#pragma once

#include <string>
#include <vector>
#include <memory>

namespace flexiv {
namespace tdk {

/**
 * @brief Teleoperation control interface that represents one participant in the joint-space
 * robot-robot teleoperation over WAN (Internet). Teleoperation is established between two robots
 * when each of them is controlled by an instance of this interface, with one set as TCP server and
 * the other set as TCP client via the constructor parameter [is_tcp_server].
 * @note In the documentation of this class, "local robot" refers to the robot represented by this
 * instance; "remote robot" refers to the robot represented by an instance on the other side of
 * teleoperation. This reference is a relative idea and is interchangeable.
 * @par TCP Server and Client Configuration
 * In a teleoperation-over-WAN setup, there are one robot + one edge device on each side of the
 * teleoperation. One edge device needs to function as a TCP server while the other device functions
 * as a TCP client. It does not matter which side is configured as TCP server or client. However,
 * while the edge device for TCP client doesn't need any additional configuration other than
 * connecting to the Internet, the TCP server needs to complete the following additional steps:
 *
 * 1. In the settings of the network router that the edge device for TCP server is connected to,
 * enable NAT (network address translation). This is usually enabled by default on modern routers.
 * 2. Note down the private (LAN) IPv4 address assigned to the edge device for TCP server.
 * 3. In the router settings, add TCP port forwarding rule for the IPv4 address noted in step 2. The
 * port number can be set to any unoccupied one. Use this port number as the [listening_port]
 * constructor parameter for BOTH sides of teleoperation.
 * 4. On the edge device for TCP server, open https://whatismyipaddress.com/ and note down its
 * public IPv4 address. Use this address as the [public_ipv4_address] constructor parameter for BOTH
 * sides of teleoperation.
 */
class JointTeleopWAN
{
public:
    /**
     * @brief [Blocking] Create an instance of the control interface. Connection with the local
     * robot will be established, and WAN communication services will be started.
     * @param[in] robot_sn Serial number of the local robot. The accepted formats are:
     * "Rizon 4s-123456" and "Rizon4s-123456".
     * @param[in] is_tcp_server True: the machine running this instance functions as the TCP server;
     * false: functions as the TCP client. If true, then the machine on the other side of
     * teleoperation needs to function as the TCP client. It does not matter which side functions as
     * the TCP server, however, the server side must configure its network router with NAT and TCP
     * port forwarding.
     * @param[in] public_ipv4_address Public IPv4 address of whichever machine that functions as the
     * TCP server. Can be obtained from https://whatismyipaddress.com/. Both sides of the
     * teleoperation need to set the same address.
     * @param[in] listening_port Number of the port configured with TCP port forwarding. Both sides
     * of the teleoperation need to set the same listening port.
     * @param[in] lan_interface_whitelist Limit the network interface(s) that can be used to try
     * to establish connection with the local robot. The whitelisted network interface is
     * defined by its associated IPv4 address. For example, {"10.42.0.1", "192.168.2.102"}. If left
     * empty, all available network interfaces will be tried when searching for the local robot.
     * @param[in] wan_interface_whitelist Limit the network interface(s) that can be used to try
     * to establish connection with the remote robot. The whitelisted network interface is
     * defined by its associated IPv4 address. For example, {"10.42.0.1", "192.168.2.102"}. If left
     * empty, all available network interfaces will be tried when searching for the remote robot.
     * @throw std::invalid_argument if the format of [robot_sn] is invalid.
     * @throw std::runtime_error if error occurred during construction.
     * @throw std::logic_error if the local robot does not have a valid TDK license; or this TDK
     * library version is incompatible with the local robot; or model of the local robot is not
     * supported.
     * @warning This constructor blocks until the initialization sequence is successfully finished
     * and connection with the local robot is established. It does not wait for the WAN connection
     * to be established.
     */
    JointTeleopWAN(const std::string& robot_sn, bool is_tcp_server,
        const std::string& public_ipv4_address, unsigned int listening_port,
        const std::vector<std::string>& lan_interface_whitelist = {},
        const std::vector<std::string>& wan_interface_whitelist = {});
    virtual ~JointTeleopWAN();

    /**
     * @brief [Blocking] Get the local robot ready for teleoperation. The following actions will
     * happen in sequence: a) enable robot, b) zero force/torque sensors.
     * @throw std::runtime_error if the initialization sequence failed.
     * @note This function blocks until the initialization sequence is finished.
     * @warning This process involves sensor zeroing, please make sure the robot is not in contact
     * with anything during the process.
     */
    void Init();

    /**
     * @brief [Blocking] Sync pose of the local robot and the remote robot.
     * @param[in] sync_with_remote True: local robot moves to the remote robot's joint positions;
     * false: local robot moves to [sync_positions].
     * @param[in] sync_positions Joint positions for the local robot to move to if
     * [sync_with_remote] is false: \f$ q_0 \in \mathbb{R}^{n \times 1} \f$. If left empty, the
     * local robot will stay at its current pose. Unit: \f$ [rad] \f$.
     * @throw std::invalid_argument if size of [sync_positions] does not match robot DoF.
     * @throw std::logic_error if initialization sequence hasn't been triggered yet using Init().
     * @note This function blocks until the sync is finished.
     */
    void SyncPose(bool sync_with_remote, const std::vector<double>& sync_positions = {});

    /**
     * @brief [Blocking] Start the local teleoperation control loop.
     * @throw std::runtime_error if failed to start the teleoperation control loop.
     * @throw std::logic_error if initialization sequence hasn't been triggered yet using Init().
     * @throw std::logic_error if robot pose has not been synced yet using SyncPose().
     * @note This function blocks until the control loop has started running. The user might need to
     * implement further blocking after this function returns.
     * @note None of the teleoperation participants will move until both sides are started.
     */
    void Start();

    /**
     * @brief [Blocking] Stop local teleoperation control loop and make the robot hold its pose.
     * @throw std::runtime_error if failed to stop the teleoperation control loop.
     * @note This function blocks until the control loop has stopped running and robot in hold.
     * @note When the local robot is stopped, the remote robot's control loop will still be running,
     * but the robot will hold its pose.
     */
    void Stop();

    /**
     * @brief [Non-blocking] Activate/deactivate teleoperation.
     * @param[in] activated True: allow local robot to move; false: hold local robot. When either
     * local or remote robot holds, the other robot will also hold.
     * @note The teleoperation is deactivated by default.
     * @warning When either side of teleoperation calls this function, the activation/deactivation
     * command is automatically synced to the other side. Therefore, it's not recommended to call
     * this function from both sides of teleoperation.
     */
    void Activate(bool activated);

    /**
     * @brief [Non-blocking] Set the size of soft limit zone on both sides of all joints for the
     * local robot. When a joint enters the soft limit zone, a repulsive force is generated to keep
     * the joint from reaching the actual joint limit.
     * @param[in] zone_degrees Size of the zone in degrees. Set to 0 to disable soft limit.
     * @note No soft limit by default.
     * @warning Only enable soft limit for one side of the teleoperation as enabling on both sides
     * will create conflicting repulsive force that can result in an interlock situation.
     */
    void SetSoftLimit(double zone_degrees);

    /**
     * @brief [Blocking] Set joint impedance properties for the local robot.
     * @param[in] K_q_ratio Joint stiffness ratio. Actual K_q = K_q_ratio * K_q_nom.
     * Valid range: [0.0, 1.0].
     * @param[in] Z_q Joint damping ratio. Valid range: [0.3, 0.8]. The nominal (safe) value is
     * provided as default.
     * @throw std::invalid_argument if [K_q_ratio] or [Z_q] contains any value outside the valid
     * range or size of any input vector does not match robot DoF.
     * @throw std::logic_error if teleoperation control loop is not started yet.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     * @note This function cannot be called before Start().
     * @warning Changing damping ratio [Z_q] to a non-nominal value may lead to performance and
     * stability issues, please use with caution.
     */
    void SetJointImpedance(const std::vector<double>& K_q_ratio,
        const std::vector<double>& Z_q = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7});

    /**
     * @brief [Non-blocking] Set joint inertia shaping for the specified robot pair.
     * @param[in] shaped_joint_inertia Flag to enable/disable inertia shaping and the corresponding
     * shaped inertia value for each joint in the specified robot pair, see below for more details.
     * Valid range: > 0. Unit: \f$ [kg·m^2] \f$.
     * @throw std::invalid_argument if [shaped_joint_inertia] contains any value outside the valid
     * range or its vector size does not match robot DoF.
     * @warning Robot stability is not guaranteed if inertia shaping is enabled and the values are
     * not fine tuned, please use with caution.
     * @par Inertia Shaping
     * Joint-space inertia shaping algorithm utilizes sensor data to boost physical input from the
     * operator, such that the joints behave as if their inertia becomes smaller/larger than the
     * actual value. A small shaped inertia makes the joint feel light, whereas a large shaped
     * inertia makes the joint feel heavy.
     */
    void SetInertiaShaping(const std::vector<std::pair<bool, double>>& shaped_joint_inertia);

    /**
     * @brief Joint-space degrees of freedom of the local robot.
     */
    size_t DoF() const;

    /**
     * @brief [Non-blocking] Whether the local robot is in fault state.
     * @return True: robot has fault; false: robot normal.
     */
    bool fault() const;

    /**
     * @brief [Blocking] Try to clear minor or critical fault of the robot without a power cycle.
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
     * inside the wrist connector) of the local robot.
     * @return A boolean array whose index corresponds to that of the digital input ports.
     * True: port high; false: port low.
     */
    const std::array<bool, kIOPorts> digital_inputs() const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace tdk
} // namespace flexiv
