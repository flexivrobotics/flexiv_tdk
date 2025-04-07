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
     * ^{0}\dot{x}_d \in \mathbb{R}^{6 \times 1} \f$. Providing properly calculated target velocity
     * can improve the robot's overall tracking performance at the cost of reduced robustness.
     * Leaving this input 0 can maximize robustness at the cost of reduced tracking performance.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear and \f$ \mathbb{R}^{3 \times 1} \f$
     * angular velocity. Unit: \f$ [m/s]:[rad/s] \f$.
     */
    std::array<double, kCartDoF> velocity {};

    /**
     * @param acceleration Target TCP acceleration (linear and angular) in world frame: \f$
     * ^{0}\ddot{x}_d \in \mathbb{R}^{6 \times 1} \f$. Feeding forward target acceleration can
     * improve the robot's tracking performance for highly dynamic motions, but it's also okay to
     * leave this input 0. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear and \f$
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
 * transparency teleop. Coordinate type options are: "COORD_TCP" for TCP frame and "COORD_WORLD" for
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
 * @struct RobotStates
 * @brief Data structure containing the joint- and Cartesian-space robot states.
 * @see Robot::states().
 */
struct RobotStates
{
    /**
     * Measured joint positions of the arm using link-side encoder: \f$ q \in \mathbb{R}^{n \times
     * 1} \f$. This is the direct measurement of joint positions, preferred for most cases. Unit:
     * \f$ [rad] \f$.
     */
    std::vector<double> q = {};

    /**
     * Measured joint positions of the arm using motor-side encoder: \f$ \theta \in \mathbb{R}^{n
     * \times 1} \f$. This is the indirect measurement of joint positions. \f$ \theta = q + \Delta
     * \f$, where \f$ \Delta \f$ is the joint's internal deflection between motor and link. Unit:
     * \f$ [rad] \f$.
     */
    std::vector<double> theta = {};

    /**
     * Measured joint velocities of the arm using link-side encoder: \f$ \dot{q} \in \mathbb{R}^{n
     * \times 1} \f$. This is the direct but more noisy measurement of joint velocities. Unit: \f$
     * [rad/s] \f$.
     */
    std::vector<double> dq = {};

    /**
     * Measured joint velocities of the arm using motor-side encoder: \f$ \dot{\theta} \in
     * \mathbb{R}^{n \times 1} \f$. This is the indirect but less noisy measurement of joint
     * velocities, preferred for most cases. Unit: \f$ [rad/s] \f$.
     */
    std::vector<double> dtheta = {};

    /**
     * Measured joint torques of the arm: \f$ \tau \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [Nm]
     * \f$.
     */
    std::vector<double> tau = {};

    /**
     * Desired joint torques of the arm: \f$ \tau_{d} \in \mathbb{R}^{n \times 1} \f$. Compensation
     * of nonlinear dynamics (gravity, centrifugal, and Coriolis) is excluded. Unit: \f$ [Nm] \f$.
     */
    std::vector<double> tau_des = {};

    /**
     * Numerical derivative of measured joint torques of the arm: \f$ \dot{\tau} \in \mathbb{R}^{n
     * \times 1} \f$. Unit: \f$ [Nm/s] \f$.
     */
    std::vector<double> tau_dot = {};

    /**
     * Estimated external joint torques of the arm: \f$ \hat \tau_{ext} \in \mathbb{R}^{n \times 1}
     * \f$. Produced by any external contact (with robot body or end-effector) that does not belong
     * to the known robot model. Unit: \f$ [Nm] \f$.
     */
    std::vector<double> tau_ext = {};

    /**
     * Measured joint positions of the external axes (if any): \f$ q_e \in \mathbb{R}^{n_e \times 1}
     * \f$. Unit: \f$ [rad] \f$.
     */
    std::vector<double> q_e = {};

    /**
     * Measured joint velocities of the external axes (if any): \f$ \dot{q}_e \in \mathbb{R}^{n_e
     * \times 1} \f$. Unit: \f$ [rad/s] \f$.
     */
    std::vector<double> dq_e = {};

    /**
     * Measured joint torques of the external axes (if any): \f$ \tau_e \in \mathbb{R}^{n_e \times
     * 1} \f$. Unit: \f$ [Nm] \f$.
     */
    std::vector<double> tau_e = {};

    /**
     * Measured TCP pose expressed in world frame: \f$ ^{O}T_{TCP} \in \mathbb{R}^{7 \times 1} \f$.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$
     * quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]:[] \f$.
     */
    std::array<double, kPoseSize> tcp_pose = {};

    /**
     * Measured TCP velocity expressed in world frame: \f$ ^{O}\dot{X} \in \mathbb{R}^{6 \times 1}
     * \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear velocity and \f$ \mathbb{R}^{3 \times
     * 1} \f$ angular velocity: \f$ [v_x, v_y, v_z, \omega_x, \omega_y, \omega_z]^T \f$.
     * Unit: \f$ [m/s]:[rad/s] \f$.
     */
    std::array<double, kCartDoF> tcp_vel = {};

    /**
     * Measured flange pose expressed in world frame: \f$ ^{O}T_{flange} \in \mathbb{R}^{7 \times 1}
     * \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$
     * quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]:[] \f$.
     */
    std::array<double, kPoseSize> flange_pose = {};

    /**
     * Force-torque (FT) sensor raw reading in flange frame: \f$ ^{flange}F_{raw} \in \mathbb{R}^{6
     * \times 1} \f$. The value is 0 if no FT sensor is installed. Consists of \f$ \mathbb{R}^{3
     * \times 1} \f$ force and \f$ \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y,
     * m_z]^T \f$. Unit: \f$ [N]:[Nm] \f$.
     */
    std::array<double, kCartDoF> ft_sensor_raw = {};

    /**
     * Estimated external wrench applied on TCP and expressed in TCP frame: \f$ ^{TCP}F_{ext} \in
     * \mathbb{R}^{6 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$
     * \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$.
     * Unit: \f$ [N]:[Nm] \f$.
     */
    std::array<double, kCartDoF> ext_wrench_in_tcp = {};

    /**
     * Estimated external wrench applied on TCP and expressed in world frame: \f$ ^{0}F_{ext} \in
     * \mathbb{R}^{6 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$
     * \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$.
     * Unit: \f$ [N]:[Nm] \f$.
     */
    std::array<double, kCartDoF> ext_wrench_in_world = {};

    /**
     * Unfiltered version of ext_wrench_in_tcp. The data is more noisy but has no filter latency.
     */
    std::array<double, kCartDoF> ext_wrench_in_tcp_raw = {};

    /**
     * Unfiltered version of ext_wrench_in_world The data is more noisy but has no filter latency.
     */
    std::array<double, kCartDoF> ext_wrench_in_world_raw = {};
};

} // namespace tdk
} // namespace flexiv
