/**
 * @file data.hpp
 * @brief Header file containing various constant expressions, data structs, and enums.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 */

#pragma once

#include <array>
#include <vector>
#include <cstddef>
#include <string>
#include <mutex>

namespace flexiv {
namespace tdk {
/** Cartesian-space degrees of freedom */
constexpr size_t kCartDoF = 6;

/** Size of pose array (3 position + 4 quaternion) */
constexpr size_t kPoseSize = 7;

/** Number of digital IO ports (16 on control box + 2 inside the wrist connector) */
constexpr size_t kIOPorts = 18;

/** Max wrench feedback scaling factor for high transparency teleop*/
constexpr double kMaxWrenchFeedbackScale = 3;

struct NetworkCfg
{
    /**
     * @param is_tcp_server True : the machine running this instance functions as the TCP
     * server; false: functions as the TCP client. If true, then the machine on the other side of
     * teleoperation needs to function as the TCP client. It does not matter which side functions as
     * the TCP server, however, the server side must configure its network router with NAT and TCP
     * port forwarding.
     */
    bool is_tcp_server;

    /**
     * @param public_ipv4_address Public IPv4 address of whichever machine that functions as the
     * TCP server. Can be obtained from https://whatismyipaddress.com/. Both sides of the
     * teleoperation need to set the same address.
     */
    std::string public_ipv4_address;

    /**
     * @param listening_port Number of the port configured with TCP port forwarding. Both sides
     * of the teleoperation need to set the same listening port.
     */
    unsigned int listening_port;

    /**
     * @param lan_interface_whitelist Limit the network interface(s) that can be used to try
     * to establish connection with the robot via ethernet cable. The whitelisted network interface
     * is defined by its associated IPv4 address. For example, {"10.42.0.1", "192.168.2.102"}. If
     * left empty, all available network interfaces will be tried when searching for connection.
     */
    std::vector<std::string> lan_interface_whitelist = {};

    /**
     * @param wan_interface_whitelist Limit the network interface(s) that can be used to try
     * to establish connection with another participant. The whitelisted network interface is
     * defined by its associated IPv4 address. For example, {"10.42.0.1", "192.168.2.102"}. If left
     * empty, all available network interfaces will be tried when searching for connection.
     */
    std::vector<std::string> wan_interface_whitelist = {};
};

/** Role for transparent cartesian teleop */
enum Role
{
    UNKNOWN = 0,         ///> Unknown role.
    LAN_TELEOP,          ///> Teleoperation in LAN.
    WAN_TELEOP_LEADER,   ///> The leader robot operated by a human during teleoperation over WAN.
    WAN_TELEOP_FOLLOWER, ///> The follower robot that interacts with the remote environment during
                         /// teleoperation over WAN.
};

/**
 * @brief Motion control command struct for general device-robot teleop.
 */
struct MotionControlCmds
{
private:
    /**
     * @param pose Target TCP pose in world frame: \f$ {^{O}T_{TCP}}_{d} \in \mathbb{R}^{7
     * \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4
     * \times 1} \f$ quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]:[] \f$.
     */
    std::array<double, kPoseSize> pose {};
    /**
     * @param velocity Target TCP velocity (linear and angular) in world frame: \f$
     * ^{0}\dot{x}_d \in \mathbb{R}^{6 \times 1} \f$. Providing properly calculated target
     * velocity can improve the robot's overall tracking performance at the cost of reduced
     * robustness. Leaving this input 0 can maximize robustness at the cost of reduced tracking
     * performance. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear and \f$ \mathbb{R}^{3
     * \times 1} \f$ angular velocity. Unit: \f$ [m/s]:[rad/s] \f$.
     */
    std::array<double, kCartDoF> velocity {};

    /**
     * @param acceleration Target TCP acceleration (linear and angular) in world frame: \f$
     * ^{0}\ddot{x}_d \in \mathbb{R}^{6 \times 1} \f$. Feeding forward target acceleration can
     * improve the robot's tracking performance for highly dynamic motions, but it's also okay
     * to leave this input 0. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear and \f$
     * \mathbb{R}^{3 \times 1} \f$ angular acceleration. Unit: \f$ [m/s^2]:[rad/s^2] \f$.
     */
    std::array<double, kCartDoF> acceleration {};

    /**
     * @brief mutex for thread safety.
     */
    mutable std::mutex mutex;

public:
    /**
     * @brief Tread-safe read function: copy data to output parameters.
     * @param[out] out_pose Target TCP pose in world frame.
     * @param[out] out_velocity Target TCP velocity (linear and angular) in world frame.
     * @param[out] out_acceleration Target TCP acceleration (linear and angular) in world frame.
     */
    void read(std::array<double, kPoseSize>& out_pose, std::array<double, kCartDoF>& out_velocity,
        std::array<double, kCartDoF>& out_acceleration) const
    {
        std::lock_guard<std::mutex> lock(mutex);
        out_pose = pose;
        out_velocity = velocity;
        out_acceleration = acceleration;
    }

    /**
     * @brief Thread-safe write function: copy the input parameters to member variables.
     * @param in_pose Target TCP pose in world frame.
     * @param in_velocity Target TCP velocity (linear and angular) in world frame.
     * @param in_acceleration Target TCP acceleration (linear and angular) in world frame.
     */
    void write(const std::array<double, kPoseSize>& in_pose,
        const std::array<double, kCartDoF>& in_velocity,
        const std::array<double, kCartDoF>& in_acceleration)
    {
        std::lock_guard<std::mutex> lock(mutex);
        pose = in_pose;
        velocity = in_velocity;
        acceleration = in_acceleration;
    }

    /**
     * @brief Thread-safe zero function: set all data to 0.
     */
    void zero()
    {
        std::lock_guard<std::mutex> lock(mutex);
        pose.fill(0);
        // q.w() = 1;
        pose[3] = 1;
        velocity.fill(0);
        acceleration.fill(0);
    }
};

/**
 * @brief Reference coordinate that the axis to be locked for high transparency teleop
 */
enum CoordType
{
    COORD_UNKNOWN = 0, ///> Unknown coordinate
    COORD_TCP,         ///> TCP coordinate of local robot
    COORD_WORLD        ///> WORLD coordinate of local robot
};

static const std::string CoordTypeStr[] = {"UNKNOWN", "TCP", "WORLD"};

/**
 * @brief Get the coordinate type of axis locking status for high transparency teleop
 * @param[in] str string name of the coordinate
 * @return CoordType
 */
static inline CoordType GetCoordType(const std::string& str)
{
    for (size_t i = 0; i < COORD_WORLD - COORD_UNKNOWN + 1; i++) {
        if (str == CoordTypeStr[i]) {
            return static_cast<CoordType>(i);
        }
    }
    return COORD_UNKNOWN;
}

/**
 * @brief Data for locking axis, including reference frame and axis to be locked for high
 * transparency teleop. Coordinate type options are: "COORD_TCP" for TCP frame and "COORD_WORLD"
 * for WORLD frame.
 */
struct AxisLock
{
    /**
     * @brief Reference coordinate that the axis to be locked
     */
    CoordType coord = CoordType::COORD_UNKNOWN;

    /**
     * @brief Translation axis lock, the corresponding axis order is \f$ [X, Y, Z] \f$. True
     * for locking, false for floating.
     */
    std::array<bool, 3> lock_trans_axis = {false, false, false};

    /**
     * @brief Orientation axis lock, the corresponding axis order is \f$ [Rx, Ry, Rz] \f$.
     * True for locking, false for floating.
     */
    std::array<bool, 3> lock_ori_axis = {false, false, false};
};

} // namespace tdk
} // namespace flexiv
