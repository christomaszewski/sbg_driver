# Sbg Driver Parameters

Default Config
```yaml
sbg_driver:
  ros__parameters:
    configure_device:
      aiding:
        air_data_port: disabled
        apply: false
        dvl_port: disabled
        gps1_port: internal
        rtcm_port: disabled
      enable: false
      gnss:
        apply: false
        lever_arm_primary: '{0.0, 0.0, 0.0}'
        lever_arm_secondary: '{0.0, 0.0, 0.0}'
        secondary_mode: single
      imu_alignment:
        apply: false
        axis_x: forward
        axis_y: right
        lever_arm: '{0.0, 0.0, 0.0}'
        mis_pitch_deg: 0.0
        mis_roll_deg: 0.0
        mis_yaw_deg: 0.0
      mag_model:
        apply: false
        value: internal_normal
      motion_profile:
        apply: false
        value: general_purpose
      output:
        apply: false
        ekf_nav: unchanged
        ekf_quat: unchanged
        ekf_vel_body: unchanged
        gps_pos: unchanged
        gps_vel: unchanged
        imu: unchanged
        mag: unchanged
        port: main
        status: unchanged
        utc: unchanged
    convention:
      use_enu: false
    frames:
      base: base_link
      gps: gps_link
      imu: imu_link
      odom: odom
      time_reference: ''
    imu:
      accel_noise_stddev: -1.0
      gyro_noise_stddev: -1.0
      mag_scale: 1.0
      sensor_model: custom
    outputs:
      publish_ekf_nav_sat_fix: false
      publish_nmea_gga: false
    tf:
      broadcast_odom_to_base: true
    time:
      source: receive_time
    topics:
      ekf_nav_sat_fix: ekf/fix
      imu_data: imu/data
      imu_temperature: imu/temperature
      mag: imu/mag
      nav_sat_fix: gps/fix
      nmea: nmea
      odom: odom
      sbg_air_data_status: sbg/air_data_status
      sbg_ekf_status: sbg/ekf_status
      sbg_event: sbg/event
      sbg_gps_raw: sbg/gps_raw
      sbg_mag_calib: sbg/mag_calib
      sbg_ship_motion: sbg/ship_motion
      sbg_status: sbg/status
      time_reference: time_reference
    transport:
      file:
        path: ''
        real_time_pace: true
      serial:
        baud: 921600.0
        port: /dev/ttyUSB0
      type: file
      udp:
        in_port: 1234.0
        out_port: 5678.0
        remote_ip: 192.168.1.20

```

## transport.type

Transport: 'serial', 'udp', or 'file' (replay)


* Type: `string`

* Default Value: "file"
* Read only: True

*Constraints:*
 - one of the specified values: ['serial', 'udp', 'file']

*Additional Constraints:*



## transport.file.path

Path to .bin replay file (used when transport.type=file)


* Type: `string`

* Default Value: ""
* Read only: True

## transport.file.real_time_pace

Pace replay to original recording rate


* Type: `bool`

* Default Value: true
* Read only: True

## transport.serial.port

Serial device path (Linux) or COM name (Windows)


* Type: `string`

* Default Value: "/dev/ttyUSB0"
* Read only: True

## transport.serial.baud

Serial baud rate


* Type: `int`

* Default Value: 921600
* Read only: True

*Constraints:*
 - one of the specified values: [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]

*Additional Constraints:*



## transport.udp.remote_ip

Remote device IP address


* Type: `string`

* Default Value: "192.168.1.20"
* Read only: True

## transport.udp.in_port

Local UDP listen port


* Type: `int`

* Default Value: 1234
* Read only: True

*Constraints:*
 - parameter must be within bounds [1, 65535]

*Additional Constraints:*



## transport.udp.out_port

Remote UDP port


* Type: `int`

* Default Value: 5678
* Read only: True

*Constraints:*
 - parameter must be within bounds [1, 65535]

*Additional Constraints:*



## frames.imu

TF frame ID for IMU + magnetometer + barometer outputs


* Type: `string`

* Default Value: "imu_link"

## frames.gps

TF frame ID for GNSS outputs (NavSatFix, gps velocity, etc.)


* Type: `string`

* Default Value: "gps_link"

## frames.time_reference

frame_id for sensor_msgs/TimeReference (empty = global time)


* Type: `string`

* Default Value: ""

## frames.base

TF frame ID of the vehicle base


* Type: `string`

* Default Value: "base_link"

## frames.odom

TF frame ID of the odom frame


* Type: `string`

* Default Value: "odom"

## imu.sensor_model

Sensor tier used to look up approximate accel/gyro noise defaults for the /imu/data covariance. 'custom' uses the explicit *_noise_stddev params below (or leaves covariance unknown if those are <0). Per-model defaults are rough datasheet-derived starting points - refine for your unit.


* Type: `string`

* Default Value: "custom"

*Constraints:*
 - one of the specified values: ['custom', 'ellipse', 'pulse', 'ekinox', 'apogee', 'quanta']

*Additional Constraints:*



## imu.accel_noise_stddev

Per-axis accelerometer 1σ noise in m/s². Overrides the sensor_model default when >= 0. Covariance diagonal = stddev². <0 => unknown.


* Type: `double`

* Default Value: -1.0

## imu.gyro_noise_stddev

Per-axis gyroscope 1σ noise in rad/s. Overrides the sensor_model default when >= 0. Covariance diagonal = stddev². <0 => unknown.


* Type: `double`

* Default Value: -1.0

## imu.mag_scale

Scale applied to the SBG magnetometer values before publishing. SBG outputs arbitrary units (~1.0 = local Earth field), NOT the Tesla that sensor_msgs/MagneticField specifies; the default 1.0 publishes the raw a.u. values (matches the upstream SBG driver). Set to the approximate local field magnitude in Tesla (e.g. 5.0e-5) to publish physical units.


* Type: `double`

* Default Value: 1.0

*Constraints:*
 - greater than 0.0

*Additional Constraints:*



## time.source

header.stamp source for published messages. 'receive_time' stamps with the host clock at dispatch (default; keeps all robot sensors in one timebase with zero configuration). 'device_utc' stamps each log from its device timeStamp mapped through the INS's UTC clock (via the UTC_TIME log), removing serial/scheduler jitter and giving replayed captures their original recording times - use it when the host and other sensors discipline their clocks to the INS's NTP/PTP server, so device UTC IS the system timebase. Requires the UTC_TIME log to be enabled; falls back to receive_time until the first GNSS-synced UTC log arrives (and for logs that carry no timestamp, e.g. raw GNSS/RTCM blobs). sensor_msgs/TimeReference keeps a receive-time header in both modes.


* Type: `string`

* Default Value: "receive_time"

*Constraints:*
 - one of the specified values: ['receive_time', 'device_utc']

*Additional Constraints:*



## convention.use_enu

Convert sensor-native NED outputs to ENU per REP-103. Most ROS nav stacks (nav2, robot_localization) assume ENU and will produce wrong results with NED data. Default false (sensor-native); flip to true for typical ROS pipelines.


* Type: `bool`

* Default Value: false

## topics.imu_data

sensor_msgs/Imu output topic


* Type: `string`

* Default Value: "imu/data"

## topics.imu_temperature

sensor_msgs/Temperature from IMU board temp


* Type: `string`

* Default Value: "imu/temperature"

## topics.mag

sensor_msgs/MagneticField output topic


* Type: `string`

* Default Value: "imu/mag"

## topics.nav_sat_fix

sensor_msgs/NavSatFix output topic


* Type: `string`

* Default Value: "gps/fix"

## topics.time_reference

sensor_msgs/TimeReference output topic (sensor UTC clock)


* Type: `string`

* Default Value: "time_reference"

## topics.odom

nav_msgs/Odometry output topic (composed from EKF Nav+Quat+VelBody)


* Type: `string`

* Default Value: "odom"

## topics.ekf_nav_sat_fix

sensor_msgs/NavSatFix output topic for the FUSED INS geodetic position from EkfNav. Only created when outputs.publish_ekf_nav_sat_fix is true. Distinct from `nav_sat_fix`, which carries the raw GNSS receiver fix.


* Type: `string`

* Default Value: "ekf/fix"

## topics.nmea

nmea_msgs/Sentence ($GPGGA) output topic. Only created when outputs.publish_nmea_gga is true. Pair with a third-party NTRIP client (e.g. remap to its nmea input) to upload position to a VRS caster.


* Type: `string`

* Default Value: "nmea"

## topics.sbg_status

sbg_msgs/Status output topic - device general/com/aiding bitmasks


* Type: `string`

* Default Value: "sbg/status"

## topics.sbg_ekf_status

sbg_msgs/EkfStatus output topic - INS solution mode + aiding bits


* Type: `string`

* Default Value: "sbg/ekf_status"

## topics.sbg_air_data_status

sbg_msgs/AirDataStatus - validity bits from the air-data sensor


* Type: `string`

* Default Value: "sbg/air_data_status"

## topics.sbg_event

sbg_msgs/Event - GPIO sync-in events (A through E)


* Type: `string`

* Default Value: "sbg/event"

## topics.sbg_gps_raw

sbg_msgs/GpsRaw - raw GNSS observable blob for PPP/RTK post-processing


* Type: `string`

* Default Value: "sbg/gps_raw"

## topics.sbg_mag_calib

sbg_msgs/MagCalib - mag-calibration snapshot used by /sbg/*_mag_calibration


* Type: `string`

* Default Value: "sbg/mag_calib"

## topics.sbg_ship_motion

sbg_msgs/ShipMotion - marine surge/sway/heave motion


* Type: `string`

* Default Value: "sbg/ship_motion"

## outputs.publish_ekf_nav_sat_fix

Publish a second sensor_msgs/NavSatFix (topic topics.ekf_nav_sat_fix, default ekf/fix) carrying the FUSED INS geodetic position from EkfNav. Off by default: /gps/fix already carries the raw GNSS fix and /odom the fused solution in a local Cartesian frame. Enable when a consumer needs the fused GLOBAL lat/lon - smoother than raw GNSS and available through brief GNSS dropouts via dead-reckoning. Note NavSatStatus is coarse for the fused solution (position-valid → STATUS_FIX, else STATUS_NO_FIX).


* Type: `bool`

* Default Value: false

## outputs.publish_nmea_gga

Publish NMEA GGA (nmea_msgs/Sentence on topics.nmea, default nmea) at ~1 Hz from the GNSS position. Off by default. Enable to pair with a third-party NTRIP client that needs the rover position (GGA) to request corrections from a VRS / network-RTK caster. Only emitted for a computed fix; UTC time-of-day is receive-time (sufficient for VRS).


* Type: `bool`

* Default Value: false

## tf.broadcast_odom_to_base

Broadcast TF: frames.odom -> frames.base derived from each Odometry emission. Disable if a downstream node (e.g. robot_localization) provides the transform.


* Type: `bool`

* Default Value: true

## configure_device.enable

Master switch. When true, apply the configure_device.* settings to the device during on_activate (after open, before streaming). Each section below has its own `apply` flag, so you set only what you intend. Settings are written to device RAM and take effect immediately for this session; they are NOT persisted to non-volatile memory here. To persist, call the /sbg/save_settings service afterwards (that reboots the device).


* Type: `bool`

* Default Value: false

## configure_device.motion_profile.apply

Apply motion_profile.value when configure_device.enable is true.


* Type: `bool`

* Default Value: false

## configure_device.motion_profile.value

EKF motion-dynamics model for the platform.


* Type: `string`

* Default Value: "general_purpose"

*Constraints:*
 - one of the specified values: ['general_purpose', 'automotive', 'marine', 'airplane', 'helicopter', 'pedestrian', 'uav_rotary_wing', 'heavy_machinery', 'static', 'truck', 'railway', 'off_road_vehicle', 'underwater']

*Additional Constraints:*



## configure_device.aiding.apply

Apply the aiding-input port assignments below.


* Type: `bool`

* Default Value: false

## configure_device.aiding.gps1_port

Port the primary GNSS module is connected on.


* Type: `string`

* Default Value: "internal"

*Constraints:*
 - one of the specified values: ['port_a', 'port_b', 'port_c', 'port_d', 'port_e', 'internal', 'disabled']

*Additional Constraints:*



## configure_device.aiding.rtcm_port

Port RTCM corrections arrive on (serial-fed DGPS/RTK).


* Type: `string`

* Default Value: "disabled"

*Constraints:*
 - one of the specified values: ['port_a', 'port_b', 'port_c', 'port_d', 'port_e', 'internal', 'disabled']

*Additional Constraints:*



## configure_device.aiding.dvl_port

Port a DVL is connected on.


* Type: `string`

* Default Value: "disabled"

*Constraints:*
 - one of the specified values: ['port_a', 'port_b', 'port_c', 'port_d', 'port_e', 'internal', 'disabled']

*Additional Constraints:*



## configure_device.aiding.air_data_port

Port air-data aiding is connected on.


* Type: `string`

* Default Value: "disabled"

*Constraints:*
 - one of the specified values: ['port_a', 'port_b', 'port_c', 'port_d', 'port_e', 'internal', 'disabled']

*Additional Constraints:*



## configure_device.imu_alignment.apply

Apply the IMU axis alignment + lever arm below.


* Type: `bool`

* Default Value: false

## configure_device.imu_alignment.axis_x

Vehicle direction the IMU X axis points along.


* Type: `string`

* Default Value: "forward"

*Constraints:*
 - one of the specified values: ['forward', 'backward', 'left', 'right', 'up', 'down']

*Additional Constraints:*



## configure_device.imu_alignment.axis_y

Vehicle direction the IMU Y axis points along.


* Type: `string`

* Default Value: "right"

*Constraints:*
 - one of the specified values: ['forward', 'backward', 'left', 'right', 'up', 'down']

*Additional Constraints:*



## configure_device.imu_alignment.mis_roll_deg

Fine roll misalignment, degrees (converted to rad on apply).


* Type: `double`

* Default Value: 0.0

## configure_device.imu_alignment.mis_pitch_deg

Fine pitch misalignment, degrees.


* Type: `double`

* Default Value: 0.0

## configure_device.imu_alignment.mis_yaw_deg

Fine yaw misalignment, degrees.


* Type: `double`

* Default Value: 0.0

## configure_device.imu_alignment.lever_arm

IMU-to-vehicle-reference lever arm [x, y, z], metres (IMU axes).


* Type: `double_array`

* Default Value: {0.0, 0.0, 0.0}

*Constraints:*
 - length must be equal to 3

*Additional Constraints:*



## configure_device.gnss.apply

Apply the GNSS antenna lever arms + dual-antenna mode.


* Type: `bool`

* Default Value: false

## configure_device.gnss.lever_arm_primary

Primary GNSS antenna lever arm [x, y, z], metres (IMU axes).


* Type: `double_array`

* Default Value: {0.0, 0.0, 0.0}

*Constraints:*
 - length must be equal to 3

*Additional Constraints:*



## configure_device.gnss.lever_arm_secondary

Secondary (dual) GNSS antenna lever arm [x, y, z], metres.


* Type: `double_array`

* Default Value: {0.0, 0.0, 0.0}

*Constraints:*
 - length must be equal to 3

*Additional Constraints:*



## configure_device.gnss.secondary_mode

Dual-antenna mode. 'single' means no secondary antenna.


* Type: `string`

* Default Value: "single"

*Constraints:*
 - one of the specified values: ['single', 'dual_auto', 'dual_rough', 'dual_precise']

*Additional Constraints:*



## configure_device.mag_model.apply

Apply mag_model.value.


* Type: `bool`

* Default Value: false

## configure_device.mag_model.value

Magnetometer model.


* Type: `string`

* Default Value: "internal_normal"

*Constraints:*
 - one of the specified values: ['internal_normal', 'external_ecom']

*Additional Constraints:*



## configure_device.output.apply

Apply the per-log device-side output rates below.


* Type: `bool`

* Default Value: false

## configure_device.output.port

Device output port these rates apply to (usually the main link).


* Type: `string`

* Default Value: "main"

*Constraints:*
 - one of the specified values: ['main', 'port_c', 'port_e']

*Additional Constraints:*



## configure_device.output.imu

IMU_DATA output rate. 'unchanged' leaves the device's current setting; 'disabled' turns it off; div_N divides the 200 Hz loop (div_8=25 Hz).


* Type: `string`

* Default Value: "unchanged"

*Constraints:*
 - one of the specified values: ['unchanged', 'disabled', 'main_loop', 'div_2', 'div_4', 'div_5', 'div_8', 'div_10', 'div_20', 'div_40', 'div_100', 'div_200', 'new_data', 'pps']

*Additional Constraints:*



## configure_device.output.ekf_quat

EKF_QUAT output rate (orientation for /imu/data + /odom).


* Type: `string`

* Default Value: "unchanged"

*Constraints:*
 - one of the specified values: ['unchanged', 'disabled', 'main_loop', 'div_2', 'div_4', 'div_5', 'div_8', 'div_10', 'div_20', 'div_40', 'div_100', 'div_200', 'new_data', 'pps']

*Additional Constraints:*



## configure_device.output.ekf_nav

EKF_NAV output rate (fused position for /odom, /ekf/fix).


* Type: `string`

* Default Value: "unchanged"

*Constraints:*
 - one of the specified values: ['unchanged', 'disabled', 'main_loop', 'div_2', 'div_4', 'div_5', 'div_8', 'div_10', 'div_20', 'div_40', 'div_100', 'div_200', 'new_data', 'pps']

*Additional Constraints:*



## configure_device.output.ekf_vel_body

EKF_VEL_BODY output rate (/odom twist).


* Type: `string`

* Default Value: "unchanged"

*Constraints:*
 - one of the specified values: ['unchanged', 'disabled', 'main_loop', 'div_2', 'div_4', 'div_5', 'div_8', 'div_10', 'div_20', 'div_40', 'div_100', 'div_200', 'new_data', 'pps']

*Additional Constraints:*



## configure_device.output.mag

MAG output rate (/imu/mag).


* Type: `string`

* Default Value: "unchanged"

*Constraints:*
 - one of the specified values: ['unchanged', 'disabled', 'main_loop', 'div_2', 'div_4', 'div_5', 'div_8', 'div_10', 'div_20', 'div_40', 'div_100', 'div_200', 'new_data', 'pps']

*Additional Constraints:*



## configure_device.output.gps_pos

GPS1_POS output rate (/gps/fix, /nmea).


* Type: `string`

* Default Value: "unchanged"

*Constraints:*
 - one of the specified values: ['unchanged', 'disabled', 'main_loop', 'div_2', 'div_4', 'div_5', 'div_8', 'div_10', 'div_20', 'div_40', 'div_100', 'div_200', 'new_data', 'pps']

*Additional Constraints:*



## configure_device.output.gps_vel

GPS1_VEL output rate (GNSS velocity aiding).


* Type: `string`

* Default Value: "unchanged"

*Constraints:*
 - one of the specified values: ['unchanged', 'disabled', 'main_loop', 'div_2', 'div_4', 'div_5', 'div_8', 'div_10', 'div_20', 'div_40', 'div_100', 'div_200', 'new_data', 'pps']

*Additional Constraints:*



## configure_device.output.utc

UTC_TIME output rate (/time_reference).


* Type: `string`

* Default Value: "unchanged"

*Constraints:*
 - one of the specified values: ['unchanged', 'disabled', 'main_loop', 'div_2', 'div_4', 'div_5', 'div_8', 'div_10', 'div_20', 'div_40', 'div_100', 'div_200', 'new_data', 'pps']

*Additional Constraints:*



## configure_device.output.status

STATUS output rate (/sbg/status, /diagnostics).


* Type: `string`

* Default Value: "unchanged"

*Constraints:*
 - one of the specified values: ['unchanged', 'disabled', 'main_loop', 'div_2', 'div_4', 'div_5', 'div_8', 'div_10', 'div_20', 'div_40', 'div_100', 'div_200', 'new_data', 'pps']

*Additional Constraints:*



