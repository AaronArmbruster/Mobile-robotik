#ifndef _TURTLEBOT3__LDS03_WRAPPER_HPP_
#define _TURTLEBOT3__LDS03_WRAPPER_HPP_

#include <atomic>
#include <functional>
#include <thread>

#include <termios.h>

#include "turtlebot3/lds03_packets.hpp"

namespace turtlebot3
{

class LDS03Wrapper
{
  // Directives ----------------------------------------------------------------
  private:
    typedef std::function<void (const LDS03Packet &)> LiDARCallback;

  // Variables -----------------------------------------------------------------
  private:
    LiDARCallback callback_;

    int file_descriptor_;
    termios tty_;

    std::atomic_bool read_lidar_;
    std::thread read_lidar_thread_;

  // Methods -------------------------------------------------------------------
  public:
    LDS03Wrapper(const LiDARCallback &callback);
    ~LDS03Wrapper();

    /** ************************************************************************
     * @brief Initializes the LiDAR.
     *
     * @param port_name The port to which the LiDAR is connected.
     * @return true if the initialization was successful.
     * @return false otherwise.
     * ************************************************************************/
    bool init(const char *port_name = "/dev/ttyUSB0");

  private:
    void readLiDARThread();
    bool readBytes(uint8_t *buffer, size_t size);
};

} /* namespace turtlebot3 */

#endif /* _TURTLEBOT3__LDS03_WRAPPER_HPP_ */