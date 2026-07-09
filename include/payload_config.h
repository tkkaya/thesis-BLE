#ifndef PAYLOAD_CONFIG_H
#define PAYLOAD_CONFIG_H

// 0 = dummy payload (validate link first)
// 1 = real MPU6050 data
#define USE_IMU_PAYLOAD 1

// NODE_ID is now set automatically per environment in platformio.ini
// (-DNODE_ID=1 for TX, -DNODE_ID=2 for RX)
// This fallback only applies if building outside PlatformIO
#ifndef NODE_ID
#define NODE_ID 1
#endif

#endif