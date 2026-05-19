#ifndef _TURTLEBOT3__INTERACTIVE_TERMINAL_HPP_
#define _TURTLEBOT3__INTERACTIVE_TERMINAL_HPP_

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace turtlebot3
{

class InteractiveTerminal
{
  // Directives ----------------------------------------------------------------
  // Variables -----------------------------------------------------------------
  private:
    termios tty_;
    int status_flags_;

  // Methods -------------------------------------------------------------------
  public:
    inline InteractiveTerminal()
      : tty_({ 0 }),
        status_flags_(0)
    {
      tcgetattr(STDIN_FILENO, &tty_);

      termios interactive_tty = tty_;
      interactive_tty.c_lflag &= ~(ICANON | ECHO);
      interactive_tty.c_cc[VMIN] = 1;
      interactive_tty.c_cc[VTIME] = 0;

      tcsetattr(STDIN_FILENO, TCSANOW, &interactive_tty);

      status_flags_ = fcntl(STDIN_FILENO, F_GETFL);
      fcntl(STDIN_FILENO, F_SETFL, status_flags_ | O_NONBLOCK);
    }

    inline ~InteractiveTerminal()
    {
      tcsetattr(STDIN_FILENO, TCSANOW, &tty_);
      fcntl(STDIN_FILENO, F_SETFL, status_flags_);
    }

    /** ************************************************************************
     * @brief Reads the pressed character.
     *
     * @param key The pressed character
     * @return true if a key was pressed.
     * @return false otherwise.
     * ************************************************************************/
    inline bool readKey(char *const key)
    {
      return read(STDIN_FILENO, key, 1) == 1;
    }
};

} /* namespace turtlebot3 */

#endif /* _TURTLEBOT3__INTERACTIVE_TERMINAL_HPP_ */