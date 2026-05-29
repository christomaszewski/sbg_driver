# INS solution mode + aiding fusion bits, decoded from SbgEComLogEkfNav.status.
#
# `solution_mode` is the EKF filter state (0=uninitialized, 1=vertical-gyro,
# 2=AHRS, 3=NAV_VELOCITY, 4=NAV_POSITION). Higher values mean better state.
# `raw_status` is the original packed uint32 from the SDK for advanced users.

std_msgs/Header header

uint8 SOLUTION_MODE_UNINITIALIZED = 0
uint8 SOLUTION_MODE_VERTICAL_GYRO = 1
uint8 SOLUTION_MODE_AHRS          = 2
uint8 SOLUTION_MODE_NAV_VELOCITY  = 3
uint8 SOLUTION_MODE_NAV_POSITION  = 4

uint8 solution_mode

bool attitude_valid       # SBG_ECOM_SOL_ATTITUDE_VALID
bool heading_valid        # SBG_ECOM_SOL_HEADING_VALID
bool velocity_valid       # SBG_ECOM_SOL_VELOCITY_VALID
bool position_valid       # SBG_ECOM_SOL_POSITION_VALID

bool vert_ref_used        # vertical reference fused
bool mag_ref_used         # magnetometer fused
bool gps1_vel_used
bool gps1_pos_used
bool gps1_hdt_used        # GNSS-1 dual-antenna heading
bool gps2_vel_used
bool gps2_pos_used
bool gps2_hdt_used
bool vel_constraints_used # vehicle velocity constraints
bool odometer_used
bool dvl_bottom_track_used
bool dvl_water_track_used
bool generic_vel1_used
bool usbl_used
bool airspeed_used        # true airspeed fused
bool zupt_used            # zero-velocity update applied
bool align_valid          # sensor alignment + residual errors converged
bool vertical_aiding_used # depth or barometric altimeter fused
bool zaru_used            # zero angular-rate update applied
bool generic_pos1_used    # generic position 1 fused

uint32 raw_status         # original packed status field
