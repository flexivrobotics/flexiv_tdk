/**
 * @file robot_robot_teleop.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */
#pragma once

#include <string>
#include <memory>
#include <flexiv/tdk/data.hpp>
#include <flexiv/rdk/data.hpp>
namespace flexiv {
namespace tdk {

/**
 * @brief Main interface for Rizon series robot-robot teleop in Cartesian space, containing several
 * function categories and background services. Robot2RobotTeleop consists of a collection of
 * local-remote adaptive robot arms. It performs synchronized, force guided real-time motions and
 * provides the operator with high-fidelity perceptual feedback.
 */
class Robot2RobotTeleop
{
public:
    Robot2RobotTeleop() = delete;
    Robot2RobotTeleop(const Robot2RobotTeleop&) = delete;
    Robot2RobotTeleop(Robot2RobotTeleop&&) = delete;
    Robot2RobotTeleop& operator=(const Robot2RobotTeleop&) = delete;
    /**
     * @brief [Blocking] Create a flexiv::tdk::Robot2RobotTeleop instance as the main interface.
     * Teleop services will initialize and connection with the local and remote robot will be
     * established.
     * @param[in] local_sn Serial number of the local robot, e.g. Rizon4s-062001.
     * @param[in] remote_sn Serial number of the remote robot, e.g. Rizon4s-062002.
     * @param [in] path_to_license_jsn Path to the license config json file. See README.md to apply
     * for license.
     * @throw std::runtime_error if the initialization sequence failed.
     * @throw std::logic_error if the connected robot does not have a valid license; or this teleop
     * library version is incompatible with the connected robot; or model of the connected robot is
     * not supported.
     * @warning This constructor blocks until the initialization sequence is successfully finished
     * and connection with the robot is established.
     */
    Robot2RobotTeleop(const std::string& local_sn, const std::string& remote_sn,
        const std::string& path_to_license_jsn);
    virtual ~Robot2RobotTeleop();

    /**
     * @brief [Blocking] Initialize local and remote robots states and commands.
     * @throw std::logic_error if robots are not connected.
     * @throw std::runtime_error if failed to execute the request.
     * @note This function blocks until the request is successfully executed.
     * @warning This will zeroing force/torque sensors, make sure nothing is in contact with the
     * local and remote robot.
     */
    void Init(void);

    /**
     * @brief [Blocking] Enable the local and remote robots. If all E-stop are released and there's
     * no fault, both local and remote will release brakes, and becomes operational a few seconds
     * later.
     * @throw std::logic_error if the robot is not connected.
     * @throw std::runtime_error if failed to execute the request.
     * @note This function blocks until the request is successfully executed.
     */
    void Enable(void);

    /**
     * @brief [Non-blocking] Engage/disengage the local and remote robot. TDK supports teleop local
     * and remote robots in different configurations. When disengaged, the operators can move the
     * local robot to the center of the workspace or re-orientated for better ergonomics. Meanwhile,
     * the remote robot will remain stationary. When engaged again, the remote robot will only
     * mimics the local's relative motion instead of simply mirroring the pose.
     * @param[in] flag True to engage the teleop, false to disengage.
     * @note The teleop will keep disengaged by default.
     */
    void Engage(bool flag);

    /**
     * @brief [Non-blocking] Stop teleop by lock all the axes in World frame. Both local and remote
     * will stop moving.
     */
    void Stop(void);

    /**
     * @brief [Non-blocking] Check if robots in fault state.
     * @return True: Teleop has fault, false: everything normal.
     */
    bool fault(void);

    /**
     * @brief [Non-blocking] Whether the local and remote robots are normally operational, which
     * requires the following conditions to be met: enabled, brakes fully released, in auto-remote
     * mode, no fault, and not in reduced state.
     * @return True: operational, false: not operational.
     * @warning The robot won't execute any command until it becomes normally operational.
     */
    bool operational(void);

    /**
     * @brief [Blocking] Clear fault of the local and remote robots.
     * @return True: successfully cleared fault, false: cannot clear fault.
     * @throw std::runtime_error if failed to deliver the request.
     * @note This function blocks until fault on local and remote is successfully cleared or
     * maximum number of attempts is reached.
     */
    bool ClearFault(void);

    /**
     * @brief [Non-blocking] Periodically step teleop and the called frequency should be 1000Hz.
     * The remote will always imitate the movements of the local and feedback the external wrench to
     * the local.
     * @throw std::runtime_error if robot becomes inoperable during teleoperation.
     * @warning This is the main function doing the real-time teleop task computing, the called
     * frequency should be strictly 1000Hz.
     */
    void Step(void);

    /**
     * @brief [Non-blocking] Set preferred joint positions for the null-space posture of local
     * robot.
     * @param[in] preferred_joint_pos Preferred joint positions for the null-space posture control:
     * \f$ q_{ns} \in \mathbb{R}^{n \times 1} \f$. Valid range: [flexiv::rdk::RobotInfo::q_min,
     * flexiv::rdk::RobotInfo::q_max]. Unit: \f$ [rad] \f$.
     * @throw std::invalid_argument if [preferred_joint_pos] contains any value outside the valid
     * range.
     * @throw std::logic_error if teleop was not successfully initialized.
     * @note This setting will persist across the applicable control modes until changed again.
     * @par Null-space posture control
     * Similar to human arm, a robotic arm with redundant joint-space degree(s) of freedom (DOF > 6)
     * can change its overall posture without affecting the ongoing primary task. This is achieved
     * through a technique called "null-space control". After the preferred joint positions for a
     * desired robot posture is set using this function, the robot's null-space control module will
     * try to pull the arm as close to this posture as possible without affecting the primary
     * Cartesian motion-force control task.
     */
    void SetLocalNullSpacePosture(const std::vector<double>& preferred_joint_pos);

    /**
     * @brief [Non-blocking] Set preferred joint positions for the null-space posture of remote
     * robot.
     * @param[in] preferred_joint_pos Preferred joint positions for the null-space posture control:
     * \f$ q_{ns} \in \mathbb{R}^{n \times 1} \f$. Valid range: [flexiv::rdk::RobotInfo::q_min,
     * flexiv::rdk::RobotInfo::q_max]. Unit: \f$ [rad] \f$.
     * @throw std::invalid_argument if [preferred_joint_pos] contains any value outside the valid
     * range.
     * @throw std::logic_error if teleop was not successfully initialized.
     * @note This setting will persist across the applicable control modes until changed again.
     * @par Null-space posture control
     * Similar to human arm, a robotic arm with redundant joint-space degree(s) of freedom (DOF > 6)
     * can change its overall posture without affecting the ongoing primary task. This is achieved
     * through a technique called "null-space control". After the preferred joint positions for a
     * desired robot posture is set using this function, the robot's null-space control module will
     * try to pull the arm as close to this posture as possible without affecting the primary
     * Cartesian motion-force control task.
     */
    void SetRemoteNullSpacePosture(const std::vector<double>& preferred_joint_pos);

    /**
     * @brief [Non-blocking] Set maximum contact wrench for the teleop robot. The controller will
     * regulate its output to maintain contact wrench (force and moment) with the environment under
     * the set values.
     * @param[in] max_wrench Maximum contact wrench (force and moment): \f$ F_max \in \mathbb{R}^{6
     * \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ maximum force and \f$
     * \mathbb{R}^{3 \times 1} \f$ maximum moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$. Unit:
     * \f$ [N]~[Nm] \f$.
     * @throw std::invalid_argument if [max_wrench] contains any negative value.
     * @throw std::logic_error if teleop is not initialized.
     */
    void SetMaxContactWrench(const std::array<double, flexiv::rdk::kCartDoF>& max_wrench);

    /**
     * @brief [Non-blocking] Set the repulsive force in World/Tcp frame for the remote robot.
     * @param[in] repulsive_force The virtual repulsive force that will applied on the remote
     * robot.: \f$ repulsiveF \in \mathbb{R}^{3 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times
     * 1} \f$ repulsive force : \f$ [f_x, f_y, f_z]^T \f$. Unit: \f$ [N] \f$.
     *  @param[in] in_world Flag to indicate that the repulsive force is in World frame or Tcp
     * frame. true in World frame, false in Tcp frame.
     * @note This virtual repulsive wrench will only work on those unlocked axis.
     * @see SetLocalAxisLockCmd
     * @see GetLocalAxisLockState
     */
    void SetRepulsiveForce(const std::array<double, 3>& repulsive_force, bool in_world = true);

    /**
     * @brief[Non-blocking] Set the wrench feedback scaling factor.
     * @param[in] factor This coefficient will scale the feedback wrench of the remote robot. Scale
     * factor greater than 1 means that the external force received by the remote robot is
     * amplified, otherwise it will be reduced. Setting scale to zero means no wrench feedback and 1
     * means 100% transparency. Valid range: [0, kMaxWrenchFeedbackScale]
     * @throw std::invalid_argument if input scale is outside the valid range.
     * @warning Only when the user ensures that the interaction force between the remote robot and
     * workpiece is very small, such as when operating a very soft object, do they need to set the
     * factor to be greater than 1. If the object in contact with the remote robot has high
     * stiffness, please set the factor very carefully. The higher the scale, the greater the force
     * feedback to the local robot will be. Using a scaling factor of 1 is recommended.
     */
    void SetWrenchFeedbackScalingFactor(double factor = 1.0);

    /**
     * @brief [Non-blocking] Set the local robot axis locking command.
     * @param[in] cmd User input command to lock the motion of the specified axis in the reference
     * coordinate.
     */
    void SetLocalAxisLockCmd(const AxisLock& cmd);

    /**
     * @brief [Non-blocking] Get the local robot axis locking status
     * @param[out] data Current axis locking state of local robot.
     */
    void GetLocalAxisLockState(AxisLock& data);

    /**
     * @brief [Non-blocking] Get the local robot axis locking status
     * @warning This fuction is less efficient than the other overloaded one as additional runtime
     * memory allocation and data copying are performed.
     * @return AxisLock
     */
    AxisLock GetLocalAxisLockState(void);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace tdk
} // namespace flexiv
