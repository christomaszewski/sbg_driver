# Composite device-level status from SBG_ECOM_LOG_STATUS.
#
# Raw bitmasks are exposed verbatim (general_status, com_status, com_status_2,
# aiding_status); consult the SBG firmware reference manual to decode bits.

std_msgs/Header header

uint32 time_stamp_us            # microseconds since sensor power-up
uint16 general_status           # device general health bitmask
uint32 com_status               # communication-port status bitmask
uint16 com_status_2             # extended communication status (added in SDK v4.0)
uint32 aiding_status            # aiding-equipment availability bitmask
uint32 uptime_seconds           # 0 if unavailable
uint8  cpu_usage_percent        # 0xFF if unavailable
