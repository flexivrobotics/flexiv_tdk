/**
 * @file joint_teleop_lan.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */
#pragma once

#include <string>
#include <vector>
#include <memory>

namespace flexiv {
namespace tdk {

/**
 * @brief Teleoperation control interface to run joint-space robot-robot teleoperation for one or
 * more pairs of robots connected to the same LAN.
 */
class JointTeleopLAN
{
public:
    /**
     * @brief [Blocking] Create an instance of the control interface. More than one pair of
     * teleoperated robots can be controlled at the same time, see parameter [robot_pairs_sn].
     * @param[in] robot_pairs_sn Serial number of all robot pairs to run teleoperation on. Each pair
     * in the vector represents a pair of bilaterally teleoperated robots. For example, provide 2
     * pairs of robot serial numbers to start a dual-arm teleoperation that involves 2 pairs of
     * robots. The accepted formats are: "Rizon 4s-123456" and "Rizon4s-123456".
     * @throw std::invalid_argument if the format of any element in [robot_pairs_sn] is invalid.
     * @throw std::runtime_error if error occurred during construction.
     * @throw std::logic_error if one of the connected robots does not have a valid TDK license; or
     * the version of this TDK library is incompatible with one of the connected robots; or model of
     * any connected robot is not supported.
     * @warning This constructor blocks until the initialization sequence is successfully finished
     * and connection with all robots is established.
     */
    JointTeleopLAN(const std::vector<std::pair<std::string, std::string>>& robot_pairs_sn);
    virtual ~JointTeleopLAN();

    /**
     * @brief [Blocking] Get all robots ready for teleoperation. The following actions will
     * happen in sequence: a) enable robot, b) zero force/torque sensors.
     * @throw std::runtime_error if the initialization sequence failed.
     * @note This function blocks until the initialization sequence is finished.
     * @warning This process involves sensor zeroing, please make sure the robot is not in contact
     * with anything during the process.
     */
    void Init();

    /**
     * @brief [Blocking] Sync pose of the specified robot pair.
     * @param[in] idx Index of the robot pair to sync pose. This index is the same as the index of
     * the constructor parameter [robot_pairs_sn].
     * @param[in] sync_positions Joint positions for both robots in pair [idx] to move to: \f$ q_0
     * \in \mathbb{R}^{n \times 1} \f$. If left empty, the first robot in the pair will stay at its
     * current pose, and the second robot in the pair will move to the first robot's joint
     * positions. Unit: \f$ [rad] \f$.
     * @throw std::invalid_argument if [idx] exceeds total number of robot pairs.
     * @throw std::invalid_argument if size of [sync_positions] does not match robot DoF.
     * @throw std::logic_error if initialization sequence hasn't been triggered yet using Init().
     * @note This function blocks until the sync is finished.
     */
    void SyncPose(unsigned int idx, const std::vector<double>& sync_positions = {});

    /**
     * @brief [Blocking] Start the teleoperation control loop.
     * @throw std::runtime_error if failed to start the teleoperation control loop.
     * @throw std::logic_error if initialization sequence hasn't been triggered yet using Init().
     * @throw std::logic_error if pose of any robot pair has not been synced yet using SyncPose().
     * @note This function blocks until the control loop has started running. The user might need to
     * implement further blocking after this function returns.
     * @note None of the teleoperation participants will move until both sides are started.
     */
    void Start();

    /**
     * @brief [Blocking] Stop the teleoperation control loop and make all robots hold their pose.
     * @throw std::runtime_error if failed to stop the teleoperation control loop.
     * @note This function blocks until the control loop has stopped running and all robots in hold.
     */
    void Stop();

    /**
     * @brief [Non-blocking] Activate/deactivate teleoperation for the specified robot pair.
     * @param[in] idx Index of the robot pair to set flag for. This index is the same as the index
     * of the constructor parameter [robot_pairs_sn].
     * @param[in] activated True: allow this robot pair to move; false: hold this robot pair.
     * @throw std::invalid_argument if [idx] exceeds total number of robot pairs.
     * @note The teleoperation is deactivated by default.
     */
    void Activate(unsigned int idx, bool activated);

    /**
     * @brief [Non-blocking] Set the size of soft limit zone on both sides of all joints for the
     * specified robot pair. When a joint enters the soft limit zone, a repulsive force is generated
     * to keep the joint from reaching the actual joint limit.
     * @param[in] idx Index of the robot pair to set soft limit for. This index is the same as the
     * index of the constructor parameter [robot_pairs_sn].
     * @param[in] zone_degrees Size of the zone in degrees. Set to 0 to disable soft limit.
     * @note No soft limit by default.
     */
    void SetSoftLimit(unsigned int idx, double zone_degrees);

    /**
     * @brief [Non-blocking] Individual fault state of each connected robots.
     * @return For each element in the pair vector, true: this robot has fault, false: this robot
     * has no fault. The pattern of the pair vector is the same as the constructor parameter
     * [robot_pairs_sn].
     */
    std::vector<std::pair<bool, bool>> fault() const;

    /**
     * @brief [Non-blocking] Whether any of the connected robots is in fault state.
     * @return True: at least one robot has fault; false: none has fault.
     */
    bool any_fault(void) const;

    /**
     * @brief [Blocking] Try to clear minor or critical fault of the robot without a power cycle.
     * @param[in] timeout_sec Maximum time in seconds to wait for the fault to be successfully
     * cleared. Normally, a minor fault should take no more than 3 seconds to clear, and a critical
     * fault should take no more than 30 seconds to clear.
     * @return True: successfully cleared fault; false: failed to clear fault.
     * @return For each element in the pair vector, true: successfully cleared fault or no fault for
     * this robot, false: failed to clear fault for this robot. The pattern of the pair vector is
     * the same as the constructor parameter [robot_pairs_sn].
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the fault is successfully cleared or [timeout_sec] has
     * elapsed.
     * @warning Clearing a critical fault through this function without a power cycle requires a
     * dedicated device, which may not be installed in older robot models.
     */
    std::vector<std::pair<bool, bool>> ClearFault(unsigned int timeout_sec = 30);

    /**
     * @brief [Non-blocking] Current reading from all digital input ports on the control boxes of
     * the specified robot pair.
     * @param[in] idx Index of the robot pair to read from. This index is the same as the index
     * of the constructor parameter [robot_pairs_sn].
     * @return A pair of boolean vectors whose index corresponds to that of the digital input ports
     * of the corresponding robot in the pair. True: port high; false: port low.
     */
    const std::pair<std::vector<bool>, std::vector<bool>> digital_inputs(unsigned int idx) const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace tdk
} // namespace flexiv
