#ifndef FRAME_H
#define FRAME_H

#include <cstdint>

// The first byte tells the receiver how to read the rest of the frame.
enum FrameType : uint8_t {
  FRAME_TELEMETRY = 0xAA,
  FRAME_CONTROL   = 0xBB
};

#endif
