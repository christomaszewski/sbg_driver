# Air-data sensor status bits decoded from SbgEComLogAirData.status.
# Published alongside sensor_msgs/FluidPressure and sensor_msgs/Temperature.

std_msgs/Header header

uint16 status_bits        # raw bitmask
bool pressure_valid
bool altitude_valid
bool airspeed_valid       # pitot-tube true airspeed
bool temperature_valid
