#include "link/consts.h"

namespace kilo {

uint8_t query_reply(uint8_t type) {
  switch (type) {
    case KILO_MSG_HELLO_REQ: return KILO_MSG_HELLO;
    case KILO_MSG_PING: return KILO_MSG_PONG;
    case KILO_MSG_STATS_REQ: return KILO_MSG_STATS;
    case KILO_MSG_PCA_READ: return KILO_MSG_PCA_COUNTS;
    case KILO_MSG_CAL_GET: return KILO_MSG_CAL_VALUES;
    case KILO_MSG_PARAM_GET: return KILO_MSG_PARAM_VALUE;
    case KILO_MSG_EEPROM_READ: return KILO_MSG_EEPROM_DATA;
    default: return 0;
  }
}

}  // namespace kilo
