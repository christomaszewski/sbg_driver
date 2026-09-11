# GPIO sync-in event from SBG_ECOM_LOG_EVENT_{A,B,C,D,E}.
# Each event log may contain up to 5 timestamped pulses observed in one cycle:
# the primary at time_stamp_us, plus up to 4 secondary offsets.

std_msgs/Header header

uint8 CHANNEL_A=1
uint8 CHANNEL_B=2
uint8 CHANNEL_C=3
uint8 CHANNEL_D=4
uint8 CHANNEL_E=5
uint8 channel                    # physical GPIO input (CHANNEL_A through CHANNEL_E)

uint32 time_stamp_us              # primary event timestamp (us since power-up)
uint16 status                     # event flag bitmask
uint16[4] time_offsets_us         # secondary events 2..5 (us, relative to primary)
