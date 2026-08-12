/**
 * @file gripper_remote_control.hpp
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 */
#pragma once

#include "data.hpp"
#include "transparent_cartesian_teleop_wan.hpp"
#include <string>
#include <memory>
#include <utility>
#include <vector>

#include <flexiv/rdk/gripper.hpp>

namespace flexiv {
namespace tdk {

using namespace rdk;

/**
 * @brief Remote gripper control interface for the TransparentCartesianTeleopWAN teleoperation
 * scenario. When the follower robot has a gripper installed and configured, the operator
 * interacting with the leader robot can control the follower's gripper through this interface.
 * @note This interface reuses the rdk::Robot connection owned by a
 * TransparentCartesianTeleopWAN instance on the same machine (leader side or follower side), no
 * extra RDK client is created. Gripper commands and states are fully decoupled from the robot
 * teleop: the arm teleoperation does NOT need to be initialized or started for the gripper remote
 * control to work.
 * @note Motion commands (Move/Grasp/Stop) are published from leader to follower; lifecycle commands
 * (Enable/Disable/Init) are issued from leader to follower via RPC with explicit acknowledgment;
 * gripper states and params are published from follower to leader.
 * @warning Per RDK design, a robot's grippers must be managed by exactly one rdk::Gripper
 * instance: every instance keeps its own enabled-gripper map (starting empty on construction) and
 * pushes the complete map to the robot server on each Enable/Disable, overwriting the previous
 * one. This interface owns the only rdk::Gripper instance of the follower robot; do NOT create
 * other rdk::Gripper instances or enable/disable grippers through other tools (e.g. Elements) on
 * the follower robot while this interface is in use. A persistent desync is detected and healed
 * automatically by re-pushing the enabled-gripper map.
 */
class GripperRemoteControl
{
public:
    /**
     * @brief [Blocking] Create an instance of the gripper remote control interface using TDK
     * Professional Edition networking via a TDK Server credential file.
     * @param[in] teleop TransparentCartesianTeleopWAN instance.
     * @param[in] network_cfg_pro Network configuration containing the path to `client.conf`.
     * @param[in] verbose Whether to print verbose logs.
     * @throw std::invalid_argument if the network configuration is invalid.
     * @throw std::runtime_error if error occurred during construction.
     * @warning The referenced [teleop] instance must be constructed before this interface and
     * must outlive it. This constructor does not wait for the WAN connection with the remote side
     * to be established.
     */
    GripperRemoteControl(const TransparentCartesianTeleopWAN& teleop,
        const NetworkCfgPro& network_cfg_pro, bool verbose = true);

    virtual ~GripperRemoteControl();

    //========================================= ACCESSORS =========================================
    /**
     * @brief [Non-blocking] Whether the robot of the current role in the specified robot pair is
     * in fault state.
     * @param[in] idx Index of the robot pair. This index is the same as the index of the
     * [robot_pairs_sn] parameter of the TransparentCartesianTeleopWAN constructor.
     * @throw std::invalid_argument if [idx] is outside the valid range.
     * @return True: robot has fault; false: robot normal.
     */
    bool fault(unsigned int idx) const;

    /**
     * @brief [Blocking on leader side] Latest states of the gripper used by the specified joint
     * group. On the follower side the states are read directly from the local gripper
     * (non-blocking); on the leader side the latest states received over WAN are returned, and an
     * exception is thrown if no states have been received yet or the received states are stale.
     * @param[in] idx Index of the robot pair.
     * @param[in] group Joint group whose gripper states to get.
     * @throw std::invalid_argument if [idx] is outside the valid range.
     * @throw std::logic_error (follower) if no gripper is enabled for the specified joint group.
     * @throw std::runtime_error (leader) if no states received yet or the states are stale.
     * @return Latest gripper states: width [m], force [N] (positive: opening force, negative:
     * closing force), is_moving.
     */
    GripperStates states(unsigned int idx, JointGroup group);

    /**
     * @brief [Blocking on leader side] Latest params of the gripper used by the specified joint
     * group. Same data source as states().
     * @param[in] idx Index of the robot pair.
     * @param[in] group Joint group whose gripper params to get.
     * @throw std::invalid_argument if [idx] is outside the valid range.
     * @throw std::logic_error (follower) if no gripper is enabled for the specified joint group.
     * @throw std::runtime_error (leader) if no states received yet or the states are stale.
     * @return Latest gripper params: min/max width [m], min/max velocity [m/s], min/max force
     * [N]. Note: on the leader side the [name] field is empty.
     */
    GripperParams params(unsigned int idx, JointGroup group);

    //=================================== LEADER-ONLY LIFECYCLE ===================================
    /**
     * @brief [Blocking] Remotely enable the named gripper on the follower robot, issued via RPC
     * with explicit acknowledgment. The follower will servo on its robot first if needed.
     * @param[in] idx Index of the robot pair.
     * @param[in] group Joint group that uses this gripper. Only existing single-arm joint groups
     * like ARM_1 and ARM_2 are accepted.
     * @param[in] gripper_name Name of the gripper to enable, as configured in Elements -> Device
     * on the follower robot.
     * @throw std::invalid_argument if [idx] is outside the valid range or [gripper_name] is
     * empty.
     * @throw std::logic_error if this instance is not initialized as leader robot.
     * @throw std::runtime_error if the RPC timed out or the follower failed to enable the
     * gripper.
     * @note This function blocks until the follower acknowledges the command or the RPC times
     * out.
     */
    void Enable(unsigned int idx, JointGroup group, const std::string& gripper_name);

    /**
     * @brief [Blocking] Remotely disable the gripper currently used by the specified joint group
     * on the follower robot, issued via RPC with explicit acknowledgment.
     * @param[in] idx Index of the robot pair.
     * @param[in] group Joint group whose gripper to be disabled.
     * @throw std::invalid_argument if [idx] is outside the valid range.
     * @throw std::logic_error if this instance is not initialized as leader robot.
     * @throw std::runtime_error if the RPC timed out or the follower failed to disable the
     * gripper.
     * @note This function blocks until the follower acknowledges the command or the RPC times
     * out.
     */
    void Disable(unsigned int idx, JointGroup group);

    /**
     * @brief [Blocking] Remotely trigger the initialization of the enabled gripper used by the
     * specified joint group on the follower robot, issued via RPC with explicit acknowledgment.
     * This step is not needed for grippers that automatically initialize upon power-on.
     * @param[in] idx Index of the robot pair.
     * @param[in] group Joint group whose gripper is to be initialized.
     * @throw std::invalid_argument if [idx] is outside the valid range.
     * @throw std::logic_error if this instance is not initialized as leader robot.
     * @throw std::runtime_error if the RPC timed out or the follower failed to trigger the
     * initialization.
     * @note This function blocks until the follower acknowledges the command or the RPC times
     * out. The follower does not wait for the initialization sequence to finish.
     */
    void Init(unsigned int idx, JointGroup group);

    //================================== FOLLOWER-ONLY LIFECYCLE ==================================
    /**
     * @brief [Blocking] Locally enable the named gripper on the follower robot, so that it
     * becomes remotely controllable without waiting for the leader's Enable RPC. Typically
     * called once at follower startup. Idempotent: if a gripper is already enabled for the
     * specified joint group, this call is a no-op.
     * @param[in] idx Index of the robot pair.
     * @param[in] group Joint group that uses this gripper. Only existing single-arm joint groups
     * like ARM_1 and ARM_2 are accepted.
     * @param[in] gripper_name Name of the gripper to enable, as configured in Elements -> Device
     * on the follower robot.
     * @throw std::invalid_argument if [idx] is outside the valid range or [gripper_name] is
     * empty.
     * @throw std::logic_error if this instance is not initialized as follower robot.
     * @throw std::runtime_error if the follower robot cannot be brought to operational state or
     * failed to enable the gripper.
     * @note The follower robot will be servoed on first if not yet operational.
     */
    void EnableLocal(unsigned int idx, JointGroup group, const std::string& gripper_name);

    //==================================== LEADER-ONLY CONTROL ====================================
    /**
     * @brief [Non-blocking] Remotely command the follower's gripper used by the specified joint
     * group to move the fingers with position control. The command is published to the follower
     * on a reliable topic.
     * @param[in] idx Index of the robot pair.
     * @param[in] group Joint group whose gripper to send this command to.
     * @param[in] width Target opening width. Valid range: [GripperParams::min_width,
     * GripperParams::max_width]. Unit: \f$ [m] \f$.
     * @param[in] velocity Closing/opening velocity, cannot be 0. Valid range:
     * [GripperParams::min_vel, GripperParams::max_vel]. Unit: \f$ [m/s] \f$.
     * @param[in] force_limit Maximum contact force during movement. Valid range:
     * [GripperParams::min_force, GripperParams::max_force]. Unit: \f$ [N] \f$.
     * @throw std::invalid_argument if [idx] is outside the valid range.
     * @throw std::logic_error if this instance is not initialized as leader robot.
     * @throw std::runtime_error if failed to publish the command to the WAN topic.
     * @note The command ranges are validated on the follower side against the actual gripper
     * params. Use params() to query the valid ranges beforehand.
     */
    void Move(
        unsigned int idx, JointGroup group, double width, double velocity, double force_limit);

    /**
     * @brief [Non-blocking] Remotely command the follower's gripper used by the specified joint
     * group to grasp with direct force control. The command is published to the follower on a
     * reliable topic. Requires the enabled gripper to support direct force control.
     * @param[in] idx Index of the robot pair.
     * @param[in] group Joint group whose gripper to send this command to.
     * @param[in] force Target gripping force. Positive: closing force, negative: opening force.
     * Valid range: [GripperParams::min_force, GripperParams::max_force]. Unit: \f$ [N] \f$.
     * @throw std::invalid_argument if [idx] is outside the valid range.
     * @throw std::logic_error if this instance is not initialized as leader robot.
     * @throw std::runtime_error if failed to publish the command to the WAN topic.
     * @warning Note the sign convention of [force] is opposite to that of GripperStates::force.
     */
    void Grasp(unsigned int idx, JointGroup group, double force);

    /**
     * @brief [Non-blocking] Remotely stop and hold the follower's gripper used by the specified
     * joint group. The command is published to the follower on a reliable topic.
     * @param[in] idx Index of the robot pair.
     * @param[in] group Joint group whose gripper to send this command to.
     * @throw std::invalid_argument if [idx] is outside the valid range.
     * @throw std::logic_error if this instance is not initialized as leader robot.
     * @throw std::runtime_error if failed to publish the command to the WAN topic.
     */
    void Stop(unsigned int idx, JointGroup group);

    //======================================= SYSTEM CONTROL ======================================
    /**
     * @brief [Non-blocking] Pointer to the underlying rdk::Robot instance of the current role,
     * shared with the referenced TransparentCartesianTeleopWAN instance.
     * @param[in] idx Index of the robot pair.
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
