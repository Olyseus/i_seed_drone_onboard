#ifndef MISSION_BUILDER_H_
#define MISSION_BUILDER_H_

#include <vector>

#include "lat_lon.h"

class mission_builder {
 public:
  static std::vector<lat_lon> make(const std::vector<lat_lon>& points,
                                   const lat_lon& home);
};

#endif  // MISSION_BUILDER_H_
