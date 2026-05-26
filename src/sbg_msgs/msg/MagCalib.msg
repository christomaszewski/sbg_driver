# Snapshot from SBG_ECOM_LOG_MAG_CALIB - raw 16-byte opaque calibration data
# emitted during the mag-cal workflow. Used by /sbg/start_mag_calibration and
# /sbg/save_mag_calibration services (phase 3d).

std_msgs/Header header

uint32 time_stamp_us
uint8[16] mag_data        # opaque calibration payload
