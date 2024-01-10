/**
 * @file TeleopDefs.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 *
 */
#pragma once
#include <array>
#include <string>

namespace flexiv {
namespace omni {
namespace teleop {

/** Robot Cartesian-space degrees of freedom \f$ m \f$ */
constexpr size_t k_cartDOF = 6;

/**
 * @brief Reference coordinate that the axis to be locked
 */
enum CoordType
{
    CD_UNKNOWN = 0, ///> Unknown coordinate
    CD_TCP,         ///> TCP coordinate of local robot
    CD_WORLD        ///> WORLD coordinate of local robot

};

static const std::string CoordTypeStr[] = {"UNKNOWN", "TCP", "BASE"};

/**
 * @brief Get the Ref Coord Type object
 *
 * @param[in] str string name of the coordinate
 * @return CoordType
 */
static inline CoordType getCoordType(const std::string& str)
{
    for (size_t i = 0; i < CD_WORLD - CD_UNKNOWN + 1; i++) {
        if (str == CoordTypeStr[i]) {
            return static_cast<CoordType>(i);
        }
    }
    return CD_UNKNOWN;
}

/**
 * @brief Data for locking axis, including reference frame and axis to be locked.
 * Coordinate type options are: "CD_TCP" for TCP frame and "CD_WORLD" for WORLD frame.
 */
struct AxisLockDefs
{
    /**
     * @brief Reference coordinate that the axis to be locked
     */
    CoordType coord = CoordType::CD_UNKNOWN;

    /**
     * @brief Translation axis locking list, the corresponding axis order is \f$ [X, Y, Z] \f$. True
     * for locking, false for floating.
     */
    std::array<bool, 3> m_transAxisLockList = {false, false, false};

    /**
     * @brief Orientation axis locking list, the corresponding axis order is \f$ [Rx, Ry, Rz] \f$.
     * True for locking, false for floating.
     */
    std::array<bool, 3> m_oriAxisLockList = {false, false, false};
};

} // namespace teleop
} // namespace omni
} // namespace flexiv
