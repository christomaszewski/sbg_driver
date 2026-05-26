# Marine-specific motion solution from SBG_ECOM_LOG_SHIP_MOTION.
# Surge/sway/heave are body-frame motions of a ship.

std_msgs/Header header

uint16 status                                   # validity bitmask
float32 main_heave_period_s                     # dominant heave period
geometry_msgs/Vector3 motion_m                  # surge, sway, heave  (m)
geometry_msgs/Vector3 acceleration_m_per_s2     # surge, sway, heave  (m/s²)
geometry_msgs/Vector3 velocity_m_per_s          # surge, sway, heave  (m/s)
