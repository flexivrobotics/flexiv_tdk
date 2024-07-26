/**
 * @file data.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 *
 */
#pragma once
#include <vector>
#include <array>
#include <string>

namespace flexiv {
namespace tdk {

/**
 * @brief Reference coordinate that the axis to be locked
 */
enum CoordType
{
  COORD_UNKNOWN = 0, ///> Unknown coordinate
  COORD_TCP,         ///> TCP coordinate of local robot
  COORD_WORLD        ///> WORLD coordinate of local robot
};

static const std::string CoordTypeStr[] = {"UNKNOWN", "TCP", "BASE"};

/**
 * @brief Get the coordinate type of axis locking status
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
 * @brief Data for locking axis, including reference frame and axis to be locked.
 * Coordinate type options are: "COORD_TCP" for TCP frame and "COORD_WORLD" for WORLD frame.
 */
struct AxisLockStatus
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
