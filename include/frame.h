#ifndef FRAME_H
#define FRAME_H

#include <cstdint>

enum FrameType : uint8_t {
  FRAME_TELEMETRY = 0xAA,
  FRAME_CONTROL   = 0xBB
};

#endif
