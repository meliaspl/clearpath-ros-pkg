#ifndef ROS_FLOAT64_H_
#define ROS_FLOAT64_H_
#include <stdint.h>

namespace ros {
  class float64 {
  public:
    float convert();
    uint8_t raw[8];
  };
}

#endif /* STRING_H_ */
