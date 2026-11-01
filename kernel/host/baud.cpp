// PosixSerial::set_baud on its own: <asm/termbits.h> (termios2, BOTHER)
// cannot share a translation unit with glibc's <termios.h>.
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include "posix.h"

namespace kilo {

bool PosixSerial::set_baud(uint32_t baud) {
  struct termios2 t;
  if (ioctl(fd_, TCGETS2, &t) != 0) return false;
  t.c_cflag &= ~(tcflag_t)CBAUD;
  t.c_cflag |= BOTHER;
  t.c_ispeed = baud;
  t.c_ospeed = baud;
  return ioctl(fd_, TCSETS2, &t) == 0;
}

}  // namespace kilo
