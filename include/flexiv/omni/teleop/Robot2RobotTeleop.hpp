/**
 * @file Robot2RobotTeleop.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */
#pragma once

#include <string>
#include <memory>
#include <flexiv/omni/teleop/TeleopDefs.hpp>
namespace flexiv {
namespace omni {
namespace teleop {

/**
 * @brief Main interface for robot-robot teleop, containing several function categories
 * and background services.
 */
class Robot2RobotTeleop
{
public:
  Robot2RobotTeleop() = delete;
  Robot2RobotTeleop(const Robot2RobotTeleop&) = delete;
  Robot2RobotTeleop(Robot2RobotTeleop&&) = delete;
  Robot2RobotTeleop& operator=(const Robot2RobotTeleop&) = delete;
  /**
   * @brief [Blocking] Create a flexiv::omni::teleop instance as the main teleoperation interface.
   * Robot2RobotTeleop services will initialize and connection with the robots will be established.
   * @param[in] localSN Serial number of the local robot
   * @param[in] remoteSN Serial number of the remote robot
   * @param [in] licensePath Path to the license config json file.
   * @throw std::runtime_error if the initialization sequence failed.
   * @throw std::logic_error if the connected robot does not have a valid license; or this
   * Teleop library version is incompatible with the connected robot; or model of the connected
   * robot is not supported.
   * @warning This constructor blocks until the initialization sequence is successfully finished
   * and connection with the robot is established.
   */
  Robot2RobotTeleop(
      const std::string& localSN, const std::string& remoteSN, const std::string& licensePath);
  virtual ~Robot2RobotTeleop();

  /**
   * @brief [Blocking] Initialize teleoperation robots states
   * @throw std::logic_error if robots are not connected.
   * @throw std::runtime_error if failed to execute the request.
   * @note This function blocks until the request is successfully executed.
   */
  void init(void);

  /**
   * @brief [Blocking] Enable the teleoperation, if all E-stop are released and there's no
   * fault, both local and remote robots will release brakes, and becomes operational a few seconds
   * later.
   * @throw std::logic_error if the robot is not connected.
   * @throw std::runtime_error if failed to execute the request.
   * @note This function blocks until the request is successfully executed.
   */
  void enable(void);

  /**
   * @brief [Blocking] Stop teleoperation.
   * @throw std::runtime_error if failed to stop teleoperation.
   * @note This function blocks until teleoperation comes to a complete stop.
   */
  void stop(void);

  /**
   * @brief [Non-blocking] Check if teleoperaion in fault state.
   * @return True: teleoperation has fault, false: everything normal.
   */
  bool isFault(void);

  /**
   * @brief [Non-blocking] Whether the teleop robots are normally operational, which requires the
   * following conditions to be met: enabled, brakes fully released, in auto-remote mode, no fault,
   * and not in reduced state.
   * @return True: operational, false: not operational.
   * @warning The robot won't execute any command until it becomes normally operational.
   */
  bool isOperational(void);

  /**
   * @brief [Blocking] Clear minor fault of the teleop robots.
   * @return True: successfully cleared fault, false: cannot clear fault.
   * @throw std::runtime_error if failed to deliver the request.
   * @note This function blocks until fault on local and remote is successfully cleared or
   * maximum number of attempts is reached.
   */
  bool clearFault(void);

  /**
   * @brief [Non-blocking] Run teleoperation. The remote will always imitate the movements of the
   * local and feedback the external wrench to the local.
   * @note The remote pose will not exactly the same as that of the local. User can keep the remote
   * still by releasing the pedal, while the local can drag freely. When the local reaches an
   * appropriate pose, press down the pedal and the remote will follow the local again.
   * @note Please connect the pedal to the DI0 and 24V channel of the local control box. And
   * ensure that the circuit only connects when the pedal is pressed down.
   * @throw std::runtime_error if failed to execute the request.
   */
  void run(void);

  /**
   * @brief [Non-blocking] Set preferred joint positions for the null-space posture control of local
   * robot.
   * @param[in] preferredPositions Preferred joint positions for the null-space posture control:
   * \f$ q_{ns} \in \mathbb{R}^{n \times 1} \f$. Valid range: [TeleopRobotInfo::qMin,
   * TeleopRobotInfo::qMax]. Unit: \f$ [rad] \f$.
   * @throw std::invalid_argument if [preferredPositions] contains any value outside the valid
   * range.
   * @throw std::logic_error if robot is not in the correct control mode.
   * @note This setting will persist across the applicable control modes until changed again.
   * @par Null-space posture control
   * Similar to human arm, a robotic arm with redundant joint-space degree(s) of freedom (DOF > 6)
   * can change its overall posture without affecting the ongoing primary task. This is achieved
   * through a technique called "null-space control". After the preferred joint positions for a
   * desired robot posture is set using this function, the robot's null-space control module will
   * try to pull the arm as close to this posture as possible without affecting the primary
   * Cartesian motion-force control task.
   */
  void setLocalNullSpacePosture(const std::array<double, k_jointDOF>& preferredPositions);

  /**
   * @brief [Non-blocking] Set preferred joint positions for the null-space posture control of
   * remote robot.
   * @param[in] preferredPositions Preferred joint positions for the null-space posture control:
   * \f$ q_{ns} \in \mathbb{R}^{n \times 1} \f$. Valid range: [TeleopRobotInfo::qMin,
   * TeleopRobotInfo::qMax]. Unit: \f$ [rad] \f$.
   * @throw std::invalid_argument if [preferredPositions] contains any value outside the valid
   * range.
   * @throw std::logic_error if robot is not in the correct control mode.
   * @note This setting will persist across the applicable control modes until changed again.
   * @par Null-space posture control
   * Similar to human arm, a robotic arm with redundant joint-space degree(s) of freedom (DOF > 6)
   * can change its overall posture without affecting the ongoing primary task. This is achieved
   * through a technique called "null-space control". After the preferred joint positions for a
   * desired robot posture is set using this function, the robot's null-space control module will
   * try to pull the arm as close to this posture as possible without affecting the primary
   * Cartesian motion-force control task.
   */
  void setRemoteNullSpacePosture(const std::array<double, k_jointDOF>& preferredPositions);

  /**
   * @brief [Non-blocking] Set maximum wrench for the remote robot. The controller will regulate its
   * output to maintain contact wrench (force and moment) with the environment under the set values.
   * @param[in] max_wrench Maximum contact wrench (force and moment): \f$ F_max \in \mathbb{R}^{6
   * \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ maximum force and \f$
   * \mathbb{R}^{3 \times 1} \f$ maximum moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$. Unit:
   * \f$ [N]~[Nm] \f$.
   * @throw std::invalid_argument if [max_wrench] contains any negative value.
   */
  void setRemoteMaxWrench(const std::array<double, k_cartDOF>& max_wrench);

  /**
   * @brief [Non-blocking] Access general information of the local robot.
   * @return TeleopRobotInfo instance.
   */
  TeleopRobotInfo getLocalInfo(void) const;

  /**
   * @brief [Non-blocking] Access general information of the remote robot.
   * @return TeleopRobotInfo instance.
   */
  TeleopRobotInfo getRemoteInfo(void) const;

  /**
   * @brief [Non-blocking] Set the local robot axis locking command.
   *
   * @param[in] cmd User input command to lock the motion of the specified axis in the reference
   * coordinate.
   */
  void setLocalAxisLockCmd(const AxisLockDefs& cmd);

  /**
   * @brief [Non-blocking] Get the local robot axis locking status
   *
   * @param[out] data Current axis locking state of local robot.
   */
  void getLocalAxisLockState(AxisLockDefs& data);

  /**
   * @brief [Non-blocking] Get the local robot axis locking status
   * @warning This fuction is less efficient than the other overloaded one as additional runtime
   * memory allocation and data copying are performed.
   * @return AxisLockDefs
   */
  AxisLockDefs getLocalAxisLockState(void);

private:
  class Impl;
  std::unique_ptr<Impl> p_impl_;
};

} // namespace teleop
} // namespace omni
} // namespace flexiv
