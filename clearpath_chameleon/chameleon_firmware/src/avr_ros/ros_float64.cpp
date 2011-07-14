
#include "avr_ros/ros_float64.h"

namespace ros {

float float64::convert() {
  union {
    float real;
    uint8_t base[4];
  } u;
  uint16_t expd = ((raw[7] & 127) << 4) + ((raw[6] & 240) >> 4);
  uint16_t expf = expd ? (expd - 1024) + 128 : 0;
  u.base[3] = (raw[7] & 128) + (expf >> 1);
  u.base[2] = ((expf & 1) << 7) + ((raw[6] & 15) << 3) + ((raw[5] & 0xe0) >> 5);
  u.base[1] = ((raw[5] & 0x1f) << 3) + ((raw[4] & 0xe0) >> 5);
  u.base[0] = ((raw[4] & 0x1f) << 3) + ((raw[3] & 0xe0) >> 5);
  return u.real; 
}

}
