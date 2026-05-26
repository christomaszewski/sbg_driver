# GPIO sync-in event from SBG_ECOM_LOG_EVENT_{A,B,C,D,E}.
# Each event log may contain up to 5 timestamped pulses observed in one cycle:
# the primary at time_stamp_us, plus up to 4 secondary offsets.

std_msgs/Header header

uint32 time_stamp_us              # primary event timestamp (us since power-up)
uint16 status                     # event flag bitmask
uint16[4] time_offsets_us         # secondary events 2..5 (us, relative to primary)
