// include/apply_config.h — signature shared by all firmwares, implementation is per-protocol
#ifndef APPLY_CONFIG_H
#define APPLY_CONFIG_H

#include "control_packet.h"

// Returns true if applied successfully (possibly after a radio reinit).
bool applyConfig(const ControlPacket &cmd);

#endif