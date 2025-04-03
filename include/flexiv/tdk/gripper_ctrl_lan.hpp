/**
 * @file gripper_control_lan.hpp
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 */
#pragma once
#include "data.hpp"
#include <memory>

namespace flexiv {
namespace tdk {

/**
 * @class GripperCtrlLAN
 * @brief Gripper controls in a local area network.
 */
class GripperCtrlLAN
{
public:
    /**
     * @brief [Non-blocking] Create an instance and initialize gripper control interface.
     * @param[in] robot_sn Serial number of the connected robot in LAN.
     * @param[in] network_interface_whitelist List of allowed network interfaces.
     * @throw std::runtime_error if the initialization sequence failed.
     */
    GripperCtrlLAN(const std::string& robot_sn,
        const std::vector<std::string>& network_interface_whitelist = {});
    virtual ~GripperCtrlLAN();

    /**
     * @brief [Blocking] Enable the specified gripper as a robot device.
     * @param[in] name Name of the gripper to enable, must be an existing one.
     * @throw std::invalid_argument if the specified gripper does not exist.
     * @throw std::logic_error if a gripper is already enabled.
     * @throw std::runtime_error if failed to deliver the request to the connected robot or failed
     * to sync gripper parameters.
     * @note This function blocks until the request is successfully delivered.
     * @note There can only be one enabled gripper at a time, call Disable() on the currently
     * enabled gripper before enabling another gripper.
     * @warning There's no enforced check on whether the enabled device is a gripper or not. Using
     * this function to enable a non-gripper device will likely lead to undefined behaviors.
     */
    void Enable(const std::string& name);

    /**
     * @brief [Blocking] Disable the currently enabled gripper.
     * @throw std::logic_error if no gripper is enabled.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     */
    void Disable();

    /**
     * @brief [Blocking] Manually trigger the initialization of the enabled gripper. This step is
     * not needed for grippers that automatically initialize upon power-on.
     * @throw std::logic_error if no gripper is enabled.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     * @warning This function does not wait for the initialization sequence to finish, the user may
     * need to implement wait after calling this function before commanding the gripper.
     */
    void Init();

    /**
     * @brief [Blocking] Grasp with direct force control. This function requires the enabled gripper
     * to support direct force control.
     * @param[in] force Target gripping force. Positive: closing force, negative: opening force.
     * Valid range: [GripperParams::min_force, GripperParams::max_force]. Unit: \f$ [N] \f$.
     * @throw std::invalid_argument if [force] is outside the valid range.
     * @throw std::logic_error if no gripper is enabled.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     * @warning Target inputs outside the valid range (see params()) will be saturated.
     */
    void Grasp(double force);

    /**
     * @brief [Blocking] Move the gripper fingers with position control.
     * @param[in] width Target opening width. Valid range: [GripperParams::min_width,
     * GripperParams::max_width]. Unit: \f$ [m] \f$.
     * @param[in] velocity Closing/opening velocity, cannot be 0. Valid range:
     * [GripperParams::min_vel, GripperParams::max_vel]. Unit: \f$ [m/s] \f$.
     * @param[in] force_limit Maximum contact force during movement. Valid range:
     * [GripperParams::min_force, GripperParams::max_force]. Unit: \f$ [N] \f$.
     * @throw std::invalid_argument if any input parameter is outside its valid range.
     * @throw std::logic_error if no gripper is enabled.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     * @warning Target inputs outside the valid range (see params()) will be saturated.
     */
    void Move(double width, double velocity, double force_limit);

    /**
     * @brief [Blocking] Stop the gripper and hold its current finger width.
     * @throw std::logic_error if no gripper is enabled.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     */
    void Stop();

    /**
     * @brief [Non-blocking] Parameters of the currently enabled gripper.
     * @return GripperParams value copy.
     */
    GripperParams params() const;

    /**
     * @brief [Non-blocking] Current states data of the enabled gripper.
     * @return GripperStates value copy.
     */
    GripperStates states() const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace tdk
} // namespace flexiv
