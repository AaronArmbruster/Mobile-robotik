#include "turtlebot3/lds03_wrapper.hpp"

#include <cmath>
#include <cstdint>

#include <fcntl.h>
#include <unistd.h>

namespace turtlebot3
{

// -----------------------------------------------------------------------------
LDS03Wrapper::LDS03Wrapper(const LiDARCallback &callback)
  : callback_(callback),
    file_descriptor_(-1),
    tty_({ 0 }),
    read_lidar_(false)
{ }

// -----------------------------------------------------------------------------
LDS03Wrapper::~LDS03Wrapper()
{
  read_lidar_ = false;

  if (read_lidar_thread_.joinable())
    read_lidar_thread_.join();

  // Deactivate LiDAR
  uint8_t data[] = { 0xAA, 0x55, 0xF5, 0x0A };
  write(file_descriptor_, data, sizeof(data));

  tcsetattr(file_descriptor_, TCSANOW, &tty_);

  close(file_descriptor_);
}

// -----------------------------------------------------------------------------
bool LDS03Wrapper::init(const char *port_name)
{
  if ((file_descriptor_ = open(port_name, O_RDWR | O_NOCTTY)) == -1)
    return false;

  if (tcgetattr(file_descriptor_, &tty_) == -1)
    return false;

  termios lidar_tty = tty_;
  cfmakeraw(&lidar_tty);

  if (cfsetspeed(&lidar_tty, B230400) == -1)
    return false;

  if (tcsetattr(file_descriptor_, TCSANOW, &lidar_tty) == -1)
    return false;

  // Activate LiDAR
  uint8_t data[] = { 0xAA, 0x55, 0xF0, 0x0F };
  ssize_t bytes_written = write(file_descriptor_, data, sizeof(data));

  if (bytes_written != sizeof(data))
    return false;

  read_lidar_ = true;
  read_lidar_thread_ = std::thread(
    std::bind(&LDS03Wrapper::readLiDARThread, this)
  );

  return true;
}

// -----------------------------------------------------------------------------
void LDS03Wrapper::readLiDARThread()
{
  while(read_lidar_)
  {
    uint8_t timeout = 0U;

    while (++timeout)
    {
      uint8_t header[2] = { 0 };
      readBytes(header, 1);

      if (header[0] != 0xAA)
        continue;

      readBytes(header + 1, 1);

      if (header[1] != 0x55)
        continue;

      break;
    }

    if (timeout)
    {
      LDS03HeaderRaw header_raw = { 0 };
      readBytes(reinterpret_cast<uint8_t *>(&header_raw), sizeof(header_raw));

      if (header_raw.fags.ring_start)
        continue;

      LDS03DataRaw data_raw[header_raw.size] = { 0 };
      readBytes(
        reinterpret_cast<uint8_t *>(data_raw),
        sizeof(LDS03DataRaw) * header_raw.size
      );

      LDS03Packet packet = { 0 };
      LDS03Header &header = packet.header;
      header.angle_start =  M_PI_4 * header_raw.angle_start / 5760.0;
      header.angle_end = M_PI_4 * header_raw.angle_end / 5760.0;
      header.angle_increment
        = ((header.angle_end > header.angle_start ? 0.0 : 2.0 * M_PI)
          + header.angle_end - header.angle_start) / (header_raw.size - 1);

      packet.data.reserve(header_raw.size);

      for (LDS03DataRaw d : data_raw)
      {
        LDS03Data data;
        data.distance = d.distance / 1000.0;
        data.intensity = d.intensity / 255.0;
        packet.data.push_back(data);
      }

      callback_(packet);
    }
  }
}

// -----------------------------------------------------------------------------
bool LDS03Wrapper::readBytes(uint8_t *buffer, size_t size)
{
  size_t total_bytes_read = 0;

  while (total_bytes_read < size)
  {
    ssize_t bytes_read = read(
      file_descriptor_,
      buffer + total_bytes_read,
      size - total_bytes_read
    );

    if (bytes_read == -1)
      return false;

    total_bytes_read += bytes_read;
  }

  return true;
}

} /* namespace turtlebot3 */