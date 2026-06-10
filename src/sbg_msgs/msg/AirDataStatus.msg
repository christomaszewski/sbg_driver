# Air-data sensor status bits decoded from SbgEComLogAirData.status.
# sensor_msgs/FluidPressure + Temperature companions are planned (phase 3e).

std_msgs/Header header

uint16 status_bits        # raw bitmask
bool pressure_valid
bool altitude_valid
bool airspeed_valid       # pitot-tube true airspeed
bool temperature_valid
