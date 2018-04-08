#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
namespace venom {

/*
 * wait_key: command line wait key function.
 *
 * Returns the return code of select(). On success, return the number of ready
 * descriptors. If timeout, return 0. On error return -1.
 *
 * Reference: http://www.cplusplus.com/forum/unices/11910/
 */
int wait_key(time_t sec, time_t msec, char &c) { 

  // Initialization
  struct termios oldSettings, newSettings;
  tcgetattr( fileno( stdin ), &oldSettings );
  newSettings = oldSettings;
  newSettings.c_lflag &= (~ICANON & ~ECHO);
  tcsetattr( fileno( stdin ), TCSANOW, &newSettings );    

  // Actual waitkey
  fd_set set;
  struct timeval tv;
  tv.tv_sec = sec;
  tv.tv_usec = msec;

  FD_ZERO( &set );
  FD_SET( fileno( stdin ), &set );

  int res = select( fileno( stdin )+1, &set, NULL, NULL, &tv );
  if (res > 0)
    read( fileno( stdin ), &c, 1 );

  tcsetattr( fileno( stdin ), TCSANOW, &oldSettings );
  return res;
}

} // namespace venom
