#ifndef _TB3_UTIL__OBJECT_DETECTION_HPP_
#define _TB3_UTIL__OBJECT_DETECTION_HPP_

#include <cstddef>
#include <vector>

#include "turtlebot3/lds03_packets.hpp"

namespace tb3_util
{

/** ****************************************************************************
 * @brief Detects objects in the given LDS03Packet.
 *
 * @param packet The LDS03Packet
 * @return A vector containing a vector of the indices of the detected objects.
 * ****************************************************************************/
std::vector<std::vector<size_t>> detectObjects(
  const turtlebot3::LDS03Packet &packet
);

} /* namespace tb3_util */

#endif /* _TB3_UTIL__OBJECT_DETECTION_HPP_ */