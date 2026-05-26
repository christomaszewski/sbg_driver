# Raw GNSS observable blob (RTCM-style or proprietary, varies by receiver).
# Intended for post-processing PPP / RTK pipelines that want the raw stream.

std_msgs/Header header

uint8[] raw_data
