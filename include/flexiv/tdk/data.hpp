/**
 * @file data.hpp
 * @brief Header file containing various constant expressions, data structs, and enums.
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 */

#pragma once

#include <array>
#include <vector>
#include <cstddef>
#include <string>
#include <mutex>
#include <flexiv/rdk/data.hpp>

namespace flexiv {
namespace tdk {
/** Cartesian-space degrees of freedom */
constexpr size_t kCartDoF = 6;

/** Size of pose array (3 position + 4 quaternion) */
constexpr size_t kPoseSize = 7;

/** Number of digital IO ports. Same as flexiv::rdk::kIOPorts: 16 on the control box, then the M8
 * connector of each wrist (2 ports * maximum 2 wrists), then the pogo pin connector of each wrist
 * (2 ports * maximum 2 wrists). */
constexpr size_t kIOPorts = rdk::kIOPorts;

/** Max wrench feedback scaling factor for transparent teleop under LAN */
constexpr double kMaxWrenchFeedbackScale = 3;

/** Max robot pairs */
constexpr size_t kMaxRobotPairsNum = 2;

/**
 * @struct NetworkCfgStd
 * @brief TCP Server and Client Configuration TDK Standard Edition.
 * In TDK standard edition, there are one robot + one edge device on each side of the
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
 * @note All messages are transmitted in plaintext over TCP and message latency is highly dependent
 * on the quality of the user network connection.
 *
 */
struct NetworkCfgStd
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
     * @param wan_interface_whitelist Limit the network interface(s) that can be used to try
     * to establish connection with another participant. The whitelisted network interface is
     * defined by its OS-level name(s) of the network interface(s) that connect to the internet.
     * For example, "wlo1" for Wi-Fi and "enp3s0" for Ethernet on many Linux machines. Only the
     * whitelisted network interfaces will be used to accept incoming connections. If empty, all
     * available network interfaces will be tried when searching for connection.
     */
    std::vector<std::string> wan_interface_whitelist = {};
};

/**
 * @struct NetworkCfgPro
 * @brief Network configuration for TDK Professional Edition (credential file from TDK Server).
 *
 * Extract the credential package delivered with your TDK Server deployment and set
 * [client_config_file] to the path of `client.conf` inside that package.
 */
struct NetworkCfgPro
{
    /**
     * @param client_config_file Path to `client.conf` from the TDK Server credential package.
     */
    std::string client_config_file;
};

/**
 * @enum Role
 * @brief Roles of participants in transparent cartesian teleop
 */
enum Role
{
    UNKNOWN = 0,         ///> Unknown role.
    LAN_TELEOP,          ///> Teleoperation in LAN.
    WAN_TELEOP_LEADER,   ///> The leader robot operated by a human during teleoperation over WAN.
    WAN_TELEOP_FOLLOWER, ///> The follower robot that interacts with the remote environment during
                         /// teleoperation over WAN.
};

static const std::string RoleTypeStr[]
    = {"UNKNOWN", "LAN_TELEOP", "WAN_TELEOP_LEADER", "WAN_TELEOP_FOLLOWER"};

/**
 * @struct MotionControlCmds
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
 * @enum CoordType
 * @brief Reference coordinate that the axis to be locked used by transparent cartesian teleop
 */
enum CoordType
{
    COORD_UNKNOWN = 0, ///> Unknown coordinate
    COORD_TCP,         ///> TCP coordinate of local robot
    COORD_WORLD        ///> WORLD coordinate of local robot
};

static const std::string CoordTypeStr[] = {"UNKNOWN", "TCP", "WORLD"};

/**
 * @brief Get the coordinate type of axis locking status used by transparent cartesian teleop
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
 * @struct AxisLock
 * @brief Data for locking axis, including reference frame and axis to be locked for transparent
 * cartesian teleop. Coordinate type options are: "COORD_TCP" for TCP frame and "COORD_WORLD" for
 * WORLD frame.
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

/**
 * @enum ZeroFTSensor
 * @brief Sensor calibration options
 */
enum class ZeroFTSensor
{
    Enable,
    Disable
};

/**
 * @enum TeleopIssueCode
 * @brief Issue code and possible reasons for a teleoperation restriction or interruption.
 * @note Host applications should switch on this code for localization. The English
 * title/description/suggestion in TeleopIssue are defaults for logs and simple UIs.
 */
enum class TeleopIssueCode
{
    NONE = 0,              ///< No issue.
    JOINT_LIMIT,           ///< A joint is near its position limit.
    SINGULARITY,           ///< Arm is near a kinematic singularity.
    HIGH_JOINT_VELOCITY,   ///< Joint velocity is close to the safety limit.
    HIGH_LATENCY,          ///< Network latency exceeded the configured threshold; teleop is force-disengaged.
    NETWORK_DISCONNECTED,  ///< Connection to the peer is not established.
    CLOCK_MISMATCH,        ///< Clocks disagree (negative latency); teleop is force-disengaged.
    ROBOT_FAULT,           ///< Robot is in fault state.
    ROBOT_NOT_OPERATIONAL, ///< Robot is not operational (E-Stop, disabled, or link lost).
    CONTROL_MODE_MISMATCH, ///< Robot left real-time Cartesian motion-force mode.
};

static const std::string TeleopIssueCodeStr[] = {"NONE", "JOINT_LIMIT", "SINGULARITY",
    "HIGH_JOINT_VELOCITY", "HIGH_LATENCY", "NETWORK_DISCONNECTED", "CLOCK_MISMATCH", "ROBOT_FAULT",
    "ROBOT_NOT_OPERATIONAL", "CONTROL_MODE_MISMATCH"};

/**
 * @enum TeleopIssueLevel
 * @brief How urgently a teleop issue should be presented to the operator.
 */
enum class TeleopIssueLevel
{
    INFO = 0, ///< Advisory. Motion is still possible.
    WARNING,  ///< Motion is restricted.
    ERROR,    ///< Teleop was disengaged, stopped, or the robot needs recovery.
};

static const std::string TeleopIssueLevelStr[] = {"INFO", "WARNING", "ERROR"};

/**
 * @enum TeleopIssueSide
 * @brief Which participant the issue belongs to.
 */
enum class TeleopIssueSide
{
    NONE = 0,
    LEADER,   ///< Leader / operator-side robot.
    FOLLOWER, ///< Follower / remote-environment robot.
    NETWORK,  ///< Link between the two sides, not a specific robot.
};

static const std::string TeleopIssueSideStr[] = {"NONE", "LEADER", "FOLLOWER", "NETWORK"};

/**
 * @struct TeleopIssue
 * @brief One restriction or interruption that the operator should understand.
 *
 * Typical host usage: show [title] as a banner, [description] as the explanation, and
 * [suggestion] as the next action. Use [code] if the UI needs a localized string table.
 */
struct TeleopIssue
{
    /** Machine-readable reason. */
    TeleopIssueCode code = TeleopIssueCode::NONE;

    /** Presentation urgency. */
    TeleopIssueLevel level = TeleopIssueLevel::INFO;

    /** Which robot or the network this issue belongs to. */
    TeleopIssueSide side = TeleopIssueSide::NONE;

    /**
     * 1-based joint index (A1 = 1). 0 if the issue is not joint-specific.
     */
    int joint_index = 0;

    /**
     * True if this issue is happening now. False if it is the last remembered event
     * after the condition has already cleared.
     */
    bool active = false;

    /** True if the controller froze motion to stop the condition from getting worse. */
    bool motion_locked = false;

    /** True if teleoperation was force-disengaged because of this issue. */
    bool forced_disengage = false;

    /**
     * Measured quantity associated with the issue. Unit depends on [code]:
     * latency in ms, joint position in rad, joint velocity in rad/s.
     */
    double value = 0.0;

    /** Limit that [value] was compared against. Same unit as [value]. */
    double threshold = 0.0;

    /** Short operator-facing title, English. */
    std::string title;

    /** What happened, English, for a non-expert operator. */
    std::string description;

    /** What the operator should do next, English. */
    std::string suggestion;
};

/**
 * @struct TeleopStatus
 * @brief Snapshot of teleoperation health for one robot pair / joint group.
 *
 * [primary] is the issue the host should show first: the highest-level active
 * issue, or the last significant event if nothing is active. [issues] lists every
 * condition that is active right now.
 */
struct TeleopStatus
{
    bool initialized = false;
    bool started = false;
    /**
     * False if the operator has not engaged, or if the controller force-disengaged
     * (high latency, disconnect, or clock mismatch). Check [issues] / [primary]
     * for CLOCK_MISMATCH when [latency_ms] is negative.
     */
    bool engaged = false;
    bool stopped = true;
    bool fault = false;

    /** True if the robot currently holds pose to avoid a limit or singularity. */
    bool motion_restricted = false;

    /**
     * Estimated one-way message latency in milliseconds. WAN only; 0 on LAN.
     * Negative means clock mismatch. A very large value means not connected.
     */
    double latency_ms = 0.0;

    /** Configured latency threshold in milliseconds. WAN only. */
    double latency_threshold_ms = 0.0;

    /** Highest-priority issue to display. [primary.code] is NONE when nothing is wrong. */
    TeleopIssue primary;

    /** All currently active issues. Empty when the system is healthy. */
    std::vector<TeleopIssue> issues;
};

/**
 * @brief Default level for a teleop issue code.
 */
inline TeleopIssueLevel GetTeleopIssueLevel(TeleopIssueCode code)
{
    switch (code) {
        case TeleopIssueCode::HIGH_LATENCY:
        case TeleopIssueCode::NETWORK_DISCONNECTED:
        case TeleopIssueCode::CLOCK_MISMATCH:
        case TeleopIssueCode::ROBOT_FAULT:
        case TeleopIssueCode::ROBOT_NOT_OPERATIONAL:
        case TeleopIssueCode::CONTROL_MODE_MISMATCH:
            return TeleopIssueLevel::ERROR;
        case TeleopIssueCode::NONE:
            return TeleopIssueLevel::INFO;
        default:
            return TeleopIssueLevel::WARNING;
    }
}

} // namespace tdk
} // namespace flexiv
