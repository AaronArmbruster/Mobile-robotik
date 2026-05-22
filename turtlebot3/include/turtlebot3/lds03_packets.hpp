#ifndef _TURTLEBOT3__LDS03_PACKETS_HPP_
#define _TURTLEBOT3__LDS03_PACKETS_HPP_

#include <cstdint>
#include <vector>

namespace turtlebot3
{

// -----------------------------------------------------------------------------
struct __attribute__((packed)) LDS03FlagsRaw
{
  bool ring_start         : 1;
  uint8_t scan_frequency  : 7;
};

// -----------------------------------------------------------------------------
struct __attribute__((packed)) LDS03HeaderRaw
{
  LDS03FlagsRaw fags;
  uint8_t size;
  uint16_t angle_start;
  uint16_t angle_end;
  uint16_t crc;
};

// -----------------------------------------------------------------------------
struct __attribute__((packed)) LDS03DataRaw
{
  uint32_t            : 2;
  uint32_t intensity  : 8;
  uint32_t distance   : 14;
};

// -----------------------------------------------------------------------------
struct LDS03Header
{
  double angle_start;      // in radians
  double angle_end;        // in radians
  double angle_increment;  // in radians
};

// -----------------------------------------------------------------------------
struct LDS03Data
{
  double distance;   // in meters
  double intensity;  // in percent
};

// -----------------------------------------------------------------------------
struct LDS03Packet
{
  LDS03Header header;
  std::vector<LDS03Data> data;
};

} /* namespace turtlebot3 */

#endif /* _TURTLEBOT3__LDS03_PACKETS_HPP_ */