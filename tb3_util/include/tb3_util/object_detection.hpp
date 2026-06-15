#ifndef _TB3_UTIL__OBJECT_DETECTION_HPP_
#define _TB3_UTIL__OBJECT_DETECTION_HPP_

#include <cstddef>
#include <vector>

#include "turtlebot3/lds03_packets.hpp"

namespace tb3_util
{

typedef std::vector<size_t> Object;
typedef std::vector<Object> Objects;

/** ****************************************************************************
 * @brief Detects objects in the given LDS03Packet.
 *
 * @param packet The LDS03Packet
 * @return The detected objects.
 * ****************************************************************************/
Objects detectObjects(const turtlebot3::LDS03Packet &packet);

/** ****************************************************************************
 * @brief Checks whether a suitable goal object is among the given objects.
 *
 * @param packet The LDS03 Packet
 * @param objects The detected objects
 * @return The goal object, if one was found. Otherwise an empty Object
 * ****************************************************************************/
Object getGoal(const turtlebot3::LDS03Packet &packet, const Objects &objects);

} /* namespace tb3_util */

#endif /* _TB3_UTIL__OBJECT_DETECTION_HPP_ */