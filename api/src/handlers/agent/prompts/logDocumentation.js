export const logDataMessageGroups = {
  inertialMotion: {
    key: 'inertialMotion',
    name: 'Inertial & Motion Sensors',
    description: 'IMU/AHRS/Accelerometers/Gyros',
    bin: [
      'ACC',
      'GYR',
      'HEAT',
      'AHR2',
      'EAHR',
      'ILB1',
      'ILB2',
      'ILB3',
      'ILB4',
      'ILB5',
      'ILB6',
      'ILB7',
      'ILB8',
      'IMU',
      'VNIM',
      'VNAT',
      'IREG',
      'ISBD',
      'ISBH',
    ],
    tlog: ['AHRS', 'AHRS2', 'AHRS3'],
  },
  gpsNavigation: {
    key: 'gpsNavigation',
    name: 'GPS, Navigation & Positioning',
    description: 'GPS, vehicle, vision, vector navigation, AIS messages',
    bin: [
      'GPS',
      'GRAW',
      'GRXH',
      'GRXS',
      'GPYW',
      'GPA',
      'POS',
      'QPOS',
      'VNAT',
      'VISP',
      'VISV',
      'VISO',
      'AIS1',
      'AIS4',
      'AIS5',
      'AISR',
      'ORGN',
      'RSO2',
      'RSO3',
      'RSLL',
    ],
    tlog: ['GLOBAL_POSITION_INT', 'GPS_RAW_INT'],
  },
  controlAutopilot: {
    key: 'controlAutopilot',
    name: 'Control, Tuning & Autopilot',
    description: 'Attitude, control surfaces, PID gains, tuning, autopilot state',
    bin: [
      'ANG',
      'AOA',
      'ATT',
      'CTUN',
      'ATSC',
      'PID',
      'PIQ',
      'PRTN',
      'PS*',
      'RATE',
      'NTUN',
      'QTUN',
      'ARM',
      'AUXF',
      'PL',
      'QBRK',
      'QPOS',
      'FWDT',
      'TRMP',
      'TRMS',
      'TRQD',
      'TRSE',
      'TRST',
      'TILT',
      'TSIT',
      'TSYN',
    ],
    tlog: ['ATTITUDE', 'PARAM_VALUE'],
  },
  actuatorsMotors: {
    key: 'actuatorsMotors',
    name: 'Actuators & Motors',
    description: 'ESCs, motors, RC channels, servos, filter tuning, airspeed',
    bin: [
      'ARSP',
      'ESC',
      'ESCX',
      'EDT2',
      'RCO',
      'RCI2',
      'RCIN',
      'MOTB',
      'FCN',
      'FTN',
      'FTNS',
      'FENC',
      'CSRV',
    ],
    tlog: [],
  },
  airEnvironment: {
    key: 'airEnvironment',
    name: 'Air & Environmental Sensors',
    description:
      'Barometer, rangefinder, wind, temperature, hygrometer, variometer, airspeed, optical flow, proximity',
    bin: [
      'BARO',
      'BARD',
      'RFND',
      'WIND',
      'TEMP',
      'TCLR',
      'HYGR',
      'VAR',
      'AOA',
      'ARSP',
      'OF',
      'OFCA',
      'OAVG',
      'PRX',
      'PRXR',
    ],
    tlog: [],
  },
  simulationSitl: {
    key: 'simulationSitl',
    name: 'Simulation & SITL',
    description:
      'Simulation data, blimp/glider simulations, servo angles, mass & COG, flight dynamics, EKF smoothing',
    bin: [
      'ASM1',
      'ASM2',
      'SIM',
      'SIM2',
      'SITL',
      'SLV1',
      'SLV2',
      'SMGC',
      'SL2',
      'SLD',
      'SMVZ',
      'SAN1',
      'SAN2',
      'SBA1',
      'SFA1',
      'SFAN',
      'SFN',
      'SFT',
      'SFV1',
      'SCTL',
      'SMOO',
      'SOAR',
      'SORC',
      'SRT',
      'SSAN',
    ],
    tlog: [],
  },
  communicationTelemetry: {
    key: 'communicationTelemetry',
    name: 'Communication & Telemetry',
    description:
      'MAVLink, telemetry radios, CAN/CANFD, UART, uBlox, MCU diagnostics, message formats, Total Energy Control System',
    bin: [
      'MAV',
      'MAVC',
      'RAD',
      'UART',
      'UBX1',
      'UBX2',
      'UBXT',
      'IOMC',
      'CAND',
      'CANF',
      'CANS',
      'CAFD',
      'MSG',
      'MULT',
      'FMT',
      'FMTU',
      'NVF',
      'NVS',
      'TEC2',
      'TEC3',
      'TEC4',
      'TECS',
    ],
    tlog: ['HEARTBEAT', 'STATUSTEXT', 'SYSTEM_TIME'],
  },
  loggingReplay: {
    key: 'loggingReplay',
    name: 'Logging, Replay & Blackbox',
    description: 'Replay, logging, blackbox, EKF innovations, frame headers, visual odometry',
    bin: [
      'BBX1',
      'BBX2',
      'DMS',
      'DSF',
      'FILE',
      'EDT2',
      'RFR*',
      'RBR*',
      'RGPH',
      'RGPI',
      'RGPJ',
      'REPH',
      'REV*',
      'RVOH',
      'XKF*',
      'NKF*',
      'XKFD',
      'XKFM',
      'XKFS',
      'XKT',
      'XKV*',
      'RS*',
      'SLV*',
      'SOAR',
    ],
  },
  powerBattery: {
    key: 'powerBattery',
    name: 'Power & Battery',
    description: 'Battery, power, fuel cell, voltage & temperature monitoring, generator telemetry',
    bin: ['BAT', 'BCL', 'BCL2', 'POWR', 'IEFC', 'IE24', 'MCU', 'RICH', 'WDOG', 'TRQD', 'TRMP'],
    tlog: [],
  },
  camerasSensors: {
    key: 'camerasSensors',
    name: 'Cameras & Sensors',
    description: 'Camera, optical flow, vision, stabilisation, Swift Health / Time',
    bin: ['CAM', 'TRIG', 'VISP', 'VISO', 'SCVE', 'SBPH'],
    tlog: [],
  },
  safetyMisc: {
    key: 'safetyMisc',
    name: 'Safety, Object Avoidance & Misc',
    description: 'Object avoidance, safety events, EKF telemetry & innovations, system status',
    bin: [
      'OABR',
      'OADJ',
      'OAVG',
      'SA',
      'SAF1',
      'RS',
      'FENC',
      'NKQ',
      'NKY',
      'XKT',
      'XKQ',
      'STAT',
      'SIDD',
      'SIDS',
      'STAK',
    ],
    tlog: [],
  },
};

export const messageDataDocs_tlog = [
  {
    id: 'system_time',
    title: 'SYSTEM_TIME',
    description:
      'Synchronizes flight log time with UTC. Useful for aligning with GPS or camera timestamps.',
    tables: [
      [
        {
          name: 'time_unix_usec',
          type: 'uint64',
          unit: 'μs',
          description: 'UTC time from onboard clock',
        },
        {
          name: 'time_boot_ms',
          type: 'uint32',
          unit: 'ms',
          description: 'Time since boot (system uptime)',
        },
      ],
    ],
  },
  {
    id: 'global_position_int',
    title: 'GLOBAL_POSITION_INT',
    description:
      'Provides estimated global position (from EKF or GPS fusion). Common use: 3D trajectory plotting or flight replay.',
    tables: [
      [
        { name: 'lat', type: 'int32', unit: '1e-7 degrees', description: 'Latitude' },
        { name: 'lon', type: 'int32', unit: '1e-7 degrees', description: 'Longitude' },
        { name: 'alt', type: 'int32', unit: 'mm', description: 'Altitude above mean sea level' },
        {
          name: 'relative_alt',
          type: 'int32',
          unit: 'mm',
          description: 'Altitude above home position',
        },
        { name: 'vx', type: 'int16', unit: 'cm/s', description: 'Ground speed in X (North)' },
        { name: 'vy', type: 'int16', unit: 'cm/s', description: 'Ground speed in Y (East)' },
        { name: 'vz', type: 'int16', unit: 'cm/s', description: 'Ground speed in Z (Down)' },
        { name: 'hdg', type: 'uint16', unit: 'cdeg', description: 'Heading, 0.01° units' },
        { name: 'time_boot_ms', type: 'uint32', unit: 'ms', description: 'Time since boot' },
      ],
    ],
  },
  {
    id: 'gps_raw_int',
    title: 'GPS_RAW_INT',
    description:
      'Raw GPS readings directly from the GNSS module. Useful for quality checks, GPS drift, and RTK analysis.',
    tables: [
      [
        { name: 'time_usec', type: 'uint64', unit: 'µs', description: 'Timestamp of the GPS fix' },
        {
          name: 'fix_type',
          type: 'uint8',
          unit: '—',
          description: '0=No fix, 2=2D fix, 3=3D fix, 4=DGPS, 5=RTK',
        },
        { name: 'lat', type: 'int32', unit: '1e-7 deg', description: 'Latitude' },
        { name: 'lon', type: 'int32', unit: '1e-7 deg', description: 'Longitude' },
        { name: 'alt', type: 'int32', unit: 'mm', description: 'Altitude (MSL)' },
        { name: 'eph', type: 'uint16', unit: 'cm', description: 'Horizontal position uncertainty' },
        { name: 'epv', type: 'uint16', unit: 'cm', description: 'Vertical position uncertainty' },
        { name: 'vel', type: 'uint16', unit: 'cm/s', description: 'Ground speed' },
        { name: 'cog', type: 'uint16', unit: 'cdeg', description: 'Course over ground (0.01°)' },
        {
          name: 'satellites_visible',
          type: 'uint8',
          unit: 'count',
          description: 'Number of visible satellites',
        },
        {
          name: 'alt_ellipsoid',
          type: 'int32',
          unit: 'mm',
          description: 'Altitude above WGS84 ellipsoid',
        },
        { name: 'h_acc', type: 'uint32', unit: 'mm', description: 'Estimated horizontal accuracy' },
        { name: 'v_acc', type: 'uint32', unit: 'mm', description: 'Estimated vertical accuracy' },
        {
          name: 'vel_acc',
          type: 'uint32',
          unit: 'mm/s',
          description: 'Estimated velocity accuracy',
        },
        { name: 'hdg_acc', type: 'uint32', unit: 'mm', description: 'Estimated heading accuracy' },
        { name: 'time_boot_ms', type: 'uint32', unit: 'ms', description: 'Time since boot' },
      ],
    ],
  },
  {
    id: 'heartbeat',
    title: 'HEARTBEAT',
    description:
      'Provides high-level system state and mode changes. Use case: Mode timelines or detecting arm/disarm events.',
    tables: [
      [
        {
          name: 'type',
          type: 'uint8',
          unit: '—',
          description: 'Vehicle type (e.g., quadcopter = 2)',
        },
        {
          name: 'autopilot',
          type: 'uint8',
          unit: '—',
          description: 'Autopilot type (e.g., ArduPilot = 3)',
        },
        {
          name: 'base_mode',
          type: 'uint8',
          unit: 'bitmask',
          description: 'Flags: safety, manual, guided, armed, etc.',
        },
        { name: 'custom_mode', type: 'uint32', unit: '—', description: 'Flight mode identifier' },
        {
          name: 'system_status',
          type: 'uint8',
          unit: '—',
          description: 'System health (e.g., active, standby, critical)',
        },
        {
          name: 'mavlink_version',
          type: 'uint8',
          unit: '—',
          description: 'MAVLink protocol version',
        },
        {
          name: 'asText',
          type: 'string',
          unit: '—',
          description: 'Human-readable mode (e.g., QLOITER, GUIDED, QLAND)',
        },
        { name: 'craft', type: 'string', unit: '—', description: 'Vehicle name/alias' },
        { name: 'time_boot_ms', type: 'uint32', unit: 'ms', description: 'Timestamp' },
      ],
    ],
  },
  {
    id: 'attitude',
    title: 'ATTITUDE',
    description:
      'Orientation and angular motion data. Commonly used for attitude plots, stabilization analysis, or 3D orientation visualization.',
    tables: [
      [
        { name: 'time_boot_ms', type: 'uint32', unit: 'ms', description: 'Time since boot' },
        {
          name: 'roll',
          type: 'float',
          unit: 'radians',
          description: 'Roll angle (rotation around X)',
        },
        {
          name: 'pitch',
          type: 'float',
          unit: 'radians',
          description: 'Pitch angle (rotation around Y)',
        },
        {
          name: 'yaw',
          type: 'float',
          unit: 'radians',
          description: 'Yaw angle (rotation around Z)',
        },
        { name: 'rollspeed', type: 'float', unit: 'rad/s', description: 'Angular rate around X' },
        { name: 'pitchspeed', type: 'float', unit: 'rad/s', description: 'Angular rate around Y' },
        { name: 'yawspeed', type: 'float', unit: 'rad/s', description: 'Angular rate around Z' },
      ],
      [
        {
          name: 'roll',
          type: 'float',
          unit: 'radians',
          description: 'Orientation from specific IMU/estimator',
        },
        {
          name: 'pitch',
          type: 'float',
          unit: 'radians',
          description: 'Orientation from specific IMU/estimator',
        },
        {
          name: 'yaw',
          type: 'float',
          unit: 'radians',
          description: 'Orientation from specific IMU/estimator',
        },
        {
          name: 'error_rp',
          type: 'float',
          unit: 'radians',
          description: 'Attitude estimation errors',
        },
        {
          name: 'error_yaw',
          type: 'float',
          unit: 'radians',
          description: 'Attitude estimation errors',
        },
        { name: 'time_boot_ms', type: 'uint32', unit: 'ms', description: 'Timestamp' },
      ],
    ],
  },
  {
    id: 'param_value',
    title: 'PARAM_VALUE',
    description:
      'Configuration snapshot — helps recreate flight control setup or detect mid-flight parameter changes.',
    tables: [
      [
        {
          name: 'param_id',
          type: 'string[16]',
          unit: '—',
          description: "Parameter name (e.g., 'ATC_ANG_RLL_P')",
        },
        { name: 'param_value', type: 'float', unit: 'varies', description: 'Parameter value' },
        {
          name: 'param_type',
          type: 'uint8',
          unit: '—',
          description: 'Type enum (float, int, etc.)',
        },
        {
          name: 'param_index',
          type: 'uint16',
          unit: '—',
          description: 'Index in total parameter list',
        },
        { name: 'time_boot_ms', type: 'uint32', unit: 'ms', description: 'Timestamp' },
      ],
    ],
  },
  {
    id: 'statustext',
    title: 'STATUSTEXT',
    description: 'Textual system messages (arming warnings, failsafe alerts, GPS lost, etc.).',
    tables: [
      [
        {
          name: 'severity',
          type: 'uint8',
          unit: '—',
          description: 'Log severity (0–7: Emergency → Debug)',
        },
        { name: 'text', type: 'string[50]', unit: '—', description: 'Status or warning message' },
        { name: 'time_boot_ms', type: 'uint32', unit: 'ms', description: 'Timestamp' },
      ],
    ],
  },
  {
    id: 'trajectories',
    title: 'trajectories',
    description: 'Precomputed geospatial path for efficient rendering (e.g., 3D flight viewer).',
    tables: [
      [
        {
          name: 'GLOBAL_POSITION_INT',
          type: 'object',
          unit: '—',
          description: 'Derived path from GPS/position data',
        },
        { name: 'startAltitude', type: 'float', unit: 'meters', description: 'Starting altitude' },
        {
          name: 'trajectory',
          type: 'float[][]',
          unit: '[lon, lat, alt, time]',
          description: 'Flight path',
        },
        {
          name: 'timeTrajectory',
          type: 'object',
          unit: 'map',
          description: 'Timestamp → [lon, lat, alt, time] lookup',
        },
      ],
    ],
  },
];

export const messageDataDocs_bin = [
  {
    id: 'acc',
    title: 'ACC',
    description: 'IMU accelerometer data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'accelerometer sensor instance number',
        },
        {
          name: 'SampleUS',
          unit: 'μs',
          description: 'time since system startup this sample was taken',
        },
        {
          name: 'AccX',
          unit: 'm/s/s',
          description: 'acceleration along X axis',
        },
        {
          name: 'AccY',
          unit: 'm/s/s',
          description: 'acceleration along Y axis',
        },
        {
          name: 'AccZ',
          unit: 'm/s/s',
          description: 'acceleration along Z axis',
        },
      ],
    ],
  },
  {
    id: 'adsb',
    title: 'ADSB',
    description: 'Automatic Dependent Serveillance - Broadcast detected vehicle information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'ICAO_address',
          unit: 'Transponder address',
          description: '',
        },
        {
          name: 'Lat',
          unit: '1e-7 deglatitude',
          description: 'Vehicle latitude',
        },
        {
          name: 'Lng',
          unit: '1e-7 deglongitude',
          description: 'Vehicle longitude',
        },
        {
          name: 'Alt',
          unit: 'mm',
          description: 'Vehicle altitude',
        },
        {
          name: 'Heading',
          unit: 'cdegheading',
          description: 'Vehicle heading',
        },
        {
          name: 'Hor_vel',
          unit: 'mm/s',
          description: 'Vehicle horizontal velocity',
        },
        {
          name: 'Ver_vel',
          unit: 'mm/s',
          description: 'Vehicle vertical velocity',
        },
        {
          name: 'Squark',
          unit: 'Transponder squawk code',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'aetr',
    title: 'AETR',
    description: 'Normalised pre-mixer control surface outputs',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Ail',
          unit: 'Pre-mixer value for aileron output (between -4500 and 4500)',
          description: '',
        },
        {
          name: 'Elev',
          unit: 'Pre-mixer value for elevator output (between -4500 and 4500)',
          description: '',
        },
        {
          name: 'Thr',
          unit: 'Pre-mixer value for throttle output (between -100 and 100)',
          description: '',
        },
        {
          name: 'Rudd',
          unit: 'Pre-mixer value for rudder output (between -4500 and 4500)',
          description: '',
        },
        {
          name: 'Flap',
          unit: 'Pre-mixer value for flaps output (between 0 and 100)',
          description: '',
        },
        {
          name: 'Steer',
          unit: 'Pre-mixer value for steering output (between -4500 and 4500)',
          description: '',
        },
        {
          name: 'SS',
          unit: 'Surface movement / airspeed scaling value',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'ahr2',
    title: 'AHR2',
    description: 'Backup AHRS data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Roll',
          unit: 'deg',
          description: 'Estimated roll',
        },
        {
          name: 'Pitch',
          unit: 'deg',
          description: 'Estimated pitch',
        },
        {
          name: 'Yaw',
          unit: 'degheading',
          description: 'Estimated yaw',
        },
        {
          name: 'Alt',
          unit: 'm',
          description: 'Estimated altitude',
        },
        {
          name: 'Lat',
          unit: 'deglatitude',
          description: 'Estimated latitude',
        },
        {
          name: 'Lng',
          unit: 'deglongitude',
          description: 'Estimated longitude',
        },
        {
          name: 'Q1',
          unit: 'Estimated attitude quaternion component 1',
          description: '',
        },
        {
          name: 'Q2',
          unit: 'Estimated attitude quaternion component 2',
          description: '',
        },
        {
          name: 'Q3',
          unit: 'Estimated attitude quaternion component 3',
          description: '',
        },
        {
          name: 'Q4',
          unit: 'Estimated attitude quaternion component 4',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'ais1',
    title: 'AIS1',
    description:
      'Contents of ‘position report’ AIS message, see: https://gpsd.gitlab.io/gpsd/AIVDM.html#_types_1_2_and_3_position_report_class_a',
    tables: [
      [
        {
          name: 'US',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'typ',
          unit: 'Message Type',
          description: '',
        },
        {
          name: 'rep',
          unit: 'Repeat Indicator',
          description: '',
        },
        {
          name: 'mmsi',
          unit: 'MMSI',
          description: '',
        },
        {
          name: 'nav',
          unit: 'Navigation Status',
          description: '',
        },
        {
          name: 'rot',
          unit: 'Rate of Turn (ROT)',
          description: '',
        },
        {
          name: 'sog',
          unit: 'Speed Over Ground (SOG)',
          description: '',
        },
        {
          name: 'pos',
          unit: 'Position Accuracy',
          description: '',
        },
        {
          name: 'lon',
          unit: 'Longitude',
          description: '',
        },
        {
          name: 'lat',
          unit: 'Latitude',
          description: '',
        },
        {
          name: 'cog',
          unit: 'Course Over Ground (COG)',
          description: '',
        },
        {
          name: 'hed',
          unit: 'True Heading (HDG)',
          description: '',
        },
        {
          name: 'sec',
          unit: 'Time Stamp',
          description: '',
        },
        {
          name: 'man',
          unit: 'Maneuver Indicator',
          description: '',
        },
        {
          name: 'raim',
          unit: 'RAIM flag',
          description: '',
        },
        {
          name: 'rad',
          unit: 'Radio status',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'ais4',
    title: 'AIS4',
    description:
      'Contents of ‘Base Station Report’ AIS message, see: https://gpsd.gitlab.io/gpsd/AIVDM.html#_type_4_base_station_report',
    tables: [
      [
        {
          name: 'US',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'rep',
          unit: 'Repeat Indicator',
          description: '',
        },
        {
          name: 'mmsi',
          unit: 'MMSI',
          description: '',
        },
        {
          name: 'year',
          unit: 'Year (UTC)',
          description: '',
        },
        {
          name: 'mth',
          unit: 'Month (UTC)',
          description: '',
        },
        {
          name: 'day',
          unit: 'Day (UTC)',
          description: '',
        },
        {
          name: 'h',
          unit: 'Hour (UTC)',
          description: '',
        },
        {
          name: 'm',
          unit: 'Minute (UTC)',
          description: '',
        },
        {
          name: 's',
          unit: 'Second (UTC)',
          description: '',
        },
        {
          name: 'fix',
          unit: 'Fix quality',
          description: '',
        },
        {
          name: 'lon',
          unit: 'Longitude',
          description: '',
        },
        {
          name: 'lat',
          unit: 'Latitude',
          description: '',
        },
        {
          name: 'epfd',
          unit: 'Type of EPFD',
          description: '',
        },
        {
          name: 'raim',
          unit: 'RAIM flag',
          description: '',
        },
        {
          name: 'rad',
          unit: 'Radio status',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'ais5',
    title: 'AIS5',
    description:
      'Contents of ‘static and voyage related data’ AIS message, see: https://gpsd.gitlab.io/gpsd/AIVDM.html#_type_5_static_and_voyage_related_data',
    tables: [
      [
        {
          name: 'US',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'rep',
          unit: 'Repeat Indicator',
          description: '',
        },
        {
          name: 'mmsi',
          unit: 'MMSI',
          description: '',
        },
        {
          name: 'ver',
          unit: 'AIS Version',
          description: '',
        },
        {
          name: 'imo',
          unit: 'IMO Number',
          description: '',
        },
        {
          name: 'cal',
          unit: 'char[16]',
          description: 'Call Sign',
        },
        {
          name: 'nam',
          unit: 'char[64]',
          description: 'Vessel Name',
        },
        {
          name: 'typ',
          unit: 'Ship Type',
          description: '',
        },
        {
          name: 'bow',
          unit: 'm',
          description: 'Dimension to Bow',
        },
        {
          name: 'stn',
          unit: 'm',
          description: 'Dimension to Stern',
        },
        {
          name: 'prt',
          unit: 'm',
          description: 'Dimension to Port',
        },
        {
          name: 'str',
          unit: 'm',
          description: 'Dimension to Starboard',
        },
        {
          name: 'fix',
          unit: 'Position Fix Type',
          description: '',
        },
        {
          name: 'dght',
          unit: 'dm',
          description: 'Draught',
        },
        {
          name: 'dst',
          unit: 'char[64]',
          description: 'Destination',
        },
        {
          name: 'dte',
          unit: 'DTE',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'aisr',
    title: 'AISR',
    description:
      'Raw AIS AVDIM messages contents, see: https://gpsd.gitlab.io/gpsd/AIVDM.html#_aivdmaivdo_sentence_layer',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'num',
          unit: 'count of fragments in the currently accumulating message',
          description: '',
        },
        {
          name: 'total',
          unit: 'fragment number of this sentence',
          description: '',
        },
        {
          name: 'ID',
          unit: 'sequential message ID for multi-sentence messages',
          description: '',
        },
        {
          name: 'payload',
          unit: 'char[64]',
          description: 'data payload',
        },
      ],
    ],
  },
  {
    id: 'ang',
    title: 'ANG',
    description: 'Attitude control attitude',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Timestamp of the current Attitude loop',
        },
        {
          name: 'DesRoll',
          unit: 'deg',
          description: 'vehicle desired roll',
        },
        {
          name: 'Roll',
          unit: 'deg',
          description: 'achieved vehicle roll',
        },
        {
          name: 'DesPitch',
          unit: 'deg',
          description: 'vehicle desired pitch',
        },
        {
          name: 'Pitch',
          unit: 'deg',
          description: 'achieved vehicle pitch',
        },
        {
          name: 'DesYaw',
          unit: 'degheading',
          description: 'vehicle desired yaw',
        },
        {
          name: 'Yaw',
          unit: 'degheading',
          description: 'achieved vehicle yaw',
        },
        {
          name: 'Dt',
          unit: 's',
          description: 'attitude delta time',
        },
      ],
    ],
  },
  {
    id: 'aoa',
    title: 'AOA',
    description: 'Angle of attack and Side Slip Angle values',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'AOA',
          unit: 'deg',
          description: 'Angle of Attack calculated from airspeed, wind vector,velocity vector',
        },
        {
          name: 'SSA',
          unit: 'deg',
          description: 'Side Slip Angle calculated from airspeed, wind vector,velocity vector',
        },
      ],
    ],
  },
  {
    id: 'arm',
    title: 'ARM',
    description: 'Arming status changes',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'ArmState',
          unit: 'true if vehicle is now armed',
          description: '',
        },
        {
          name: 'ArmChecks',
          unit: 'bitmask',
          description: 'arming bitmask at time of arming Bitmask values:',
        },
        {
          name: 'ALL',
          unit: '1',
          description: '',
        },
        {
          name: 'BARO',
          unit: '2',
          description: '',
        },
        {
          name: 'COMPASS',
          unit: '4',
          description: '',
        },
        {
          name: 'GPS',
          unit: '8',
          description: '',
        },
        {
          name: 'INS',
          unit: '16',
          description: '',
        },
        {
          name: 'PARAMETERS',
          unit: '32',
          description: '',
        },
        {
          name: 'RC',
          unit: '64',
          description: '',
        },
        {
          name: 'VOLTAGE',
          unit: '128',
          description: '',
        },
        {
          name: 'BATTERY',
          unit: '256',
          description: '',
        },
        {
          name: 'AIRSPEED',
          unit: '512',
          description: '',
        },
        {
          name: 'LOGGING',
          unit: '1024',
          description: '',
        },
        {
          name: 'SWITCH',
          unit: '2048',
          description: '',
        },
        {
          name: 'GPS_CONFIG',
          unit: '4096',
          description: '',
        },
        {
          name: 'SYSTEM',
          unit: '8192',
          description: '',
        },
        {
          name: 'MISSION',
          unit: '16384',
          description: '',
        },
        {
          name: 'RANGEFINDER',
          unit: '32768',
          description: '',
        },
        {
          name: 'CAMERA',
          unit: '65536',
          description: '',
        },
        {
          name: 'AUX_AUTH',
          unit: '131072',
          description: '',
        },
        {
          name: 'VISION',
          unit: '262144',
          description: '',
        },
        {
          name: 'FFT',
          unit: '524288',
          description: '',
        },
        {
          name: 'OSD',
          unit: '1048576',
          description: '',
        },
        {
          name: 'Forced',
          unit: 'true if arm/disarm was forced',
          description: '',
        },
        {
          name: 'Method',
          unit: 'enum',
          description: 'method used for arming Values:',
        },
        {
          name: 'RUDDER',
          unit: '0',
          description: '',
        },
        {
          name: 'MAVLINK',
          unit: '1',
          description: '',
        },
        {
          name: 'AUXSWITCH',
          unit: '2',
          description: '',
        },
        {
          name: 'MOTORTEST',
          unit: '3',
          description: '',
        },
        {
          name: 'SCRIPTING',
          unit: '4',
          description: '',
        },
        {
          name: 'TERMINATION',
          unit: '5',
          description: 'only disarm uses this…',
        },
        {
          name: 'CPUFAILSAFE',
          unit: '6',
          description: 'only disarm uses this…',
        },
        {
          name: 'BATTERYFAILSAFE',
          unit: '7',
          description: 'only disarm uses this…',
        },
        {
          name: 'SOLOPAUSEWHENLANDED',
          unit: '8',
          description: 'only disarm uses this…',
        },
        {
          name: 'AFS',
          unit: '9',
          description: 'only disarm uses this…',
        },
        {
          name: 'ADSBCOLLISIONACTION',
          unit: '10',
          description: 'only disarm uses this…',
        },
        {
          name: 'PARACHUTE_RELEASE',
          unit: '11',
          description: 'only disarm uses this…',
        },
        {
          name: 'CRASH',
          unit: '12',
          description: 'only disarm uses this…',
        },
        {
          name: 'LANDED',
          unit: '13',
          description: 'only disarm uses this…',
        },
        {
          name: 'MISSIONEXIT',
          unit: '14',
          description: 'only disarm uses this…',
        },
        {
          name: 'FENCEBREACH',
          unit: '15',
          description: 'only disarm uses this…',
        },
        {
          name: 'RADIOFAILSAFE',
          unit: '16',
          description: 'only disarm uses this…',
        },
        {
          name: 'DISARMDELAY',
          unit: '17',
          description: 'only disarm uses this…',
        },
        {
          name: 'GCSFAILSAFE',
          unit: '18',
          description: 'only disarm uses this…',
        },
        {
          name: 'TERRRAINFAILSAFE',
          unit: '19',
          description: 'only disarm uses this…',
        },
        {
          name: 'FAILSAFE_ACTION_TERMINATE',
          unit: '20',
          description: 'only disarm uses this…',
        },
        {
          name: 'TERRAINFAILSAFE',
          unit: '21',
          description: 'only disarm uses this…',
        },
        {
          name: 'MOTORDETECTDONE',
          unit: '22',
          description: 'only disarm uses this…',
        },
        {
          name: 'BADFLOWOFCONTROL',
          unit: '23',
          description: 'only disarm uses this…',
        },
        {
          name: 'EKFFAILSAFE',
          unit: '24',
          description: 'only disarm uses this…',
        },
        {
          name: 'GCS_FAILSAFE_SURFACEFAILED',
          unit: '25',
          description: 'only disarm uses this…',
        },
        {
          name: 'GCS_FAILSAFE_HOLDFAILED',
          unit: '26',
          description: 'only disarm uses this…',
        },
        {
          name: 'TAKEOFFTIMEOUT',
          unit: '27',
          description: 'only disarm uses this…',
        },
        {
          name: 'AUTOLANDED',
          unit: '28',
          description: 'only disarm uses this…',
        },
        {
          name: 'PILOT_INPUT_FAILSAFE',
          unit: '29',
          description: 'only disarm uses this…',
        },
        {
          name: 'TOYMODELANDTHROTTLE',
          unit: '30',
          description: 'only disarm uses this…',
        },
        {
          name: 'TOYMODELANDFORCE',
          unit: '31',
          description: 'only disarm uses this…',
        },
        {
          name: 'LANDING',
          unit: '32',
          description: 'only disarm uses this…',
        },
        {
          name: 'DEADRECKON_FAILSAFE',
          unit: '33',
          description: 'only disarm uses this…',
        },
        {
          name: 'BLACKBOX',
          unit: '34',
          description: '',
        },
        {
          name: 'DDS',
          unit: '35',
          description: '',
        },
        {
          name: 'AUTO_ARM_ONCE',
          unit: '36',
          description: '',
        },
        {
          name: 'TURTLE_MODE',
          unit: '37',
          description: '',
        },
        {
          name: 'TOYMODE',
          unit: '38',
          description: '',
        },
        {
          name: 'UNKNOWN',
          unit: '100',
          description: '',
        },
      ],
      [
        {
          name: 'ALL',
          unit: '1',
          description: '',
        },
        {
          name: 'BARO',
          unit: '2',
          description: '',
        },
        {
          name: 'COMPASS',
          unit: '4',
          description: '',
        },
        {
          name: 'GPS',
          unit: '8',
          description: '',
        },
        {
          name: 'INS',
          unit: '16',
          description: '',
        },
        {
          name: 'PARAMETERS',
          unit: '32',
          description: '',
        },
        {
          name: 'RC',
          unit: '64',
          description: '',
        },
        {
          name: 'VOLTAGE',
          unit: '128',
          description: '',
        },
        {
          name: 'BATTERY',
          unit: '256',
          description: '',
        },
        {
          name: 'AIRSPEED',
          unit: '512',
          description: '',
        },
        {
          name: 'LOGGING',
          unit: '1024',
          description: '',
        },
        {
          name: 'SWITCH',
          unit: '2048',
          description: '',
        },
        {
          name: 'GPS_CONFIG',
          unit: '4096',
          description: '',
        },
        {
          name: 'SYSTEM',
          unit: '8192',
          description: '',
        },
        {
          name: 'MISSION',
          unit: '16384',
          description: '',
        },
        {
          name: 'RANGEFINDER',
          unit: '32768',
          description: '',
        },
        {
          name: 'CAMERA',
          unit: '65536',
          description: '',
        },
        {
          name: 'AUX_AUTH',
          unit: '131072',
          description: '',
        },
        {
          name: 'VISION',
          unit: '262144',
          description: '',
        },
        {
          name: 'FFT',
          unit: '524288',
          description: '',
        },
        {
          name: 'OSD',
          unit: '1048576',
          description: '',
        },
      ],
      [
        {
          name: 'RUDDER',
          unit: '0',
          description: '',
        },
        {
          name: 'MAVLINK',
          unit: '1',
          description: '',
        },
        {
          name: 'AUXSWITCH',
          unit: '2',
          description: '',
        },
        {
          name: 'MOTORTEST',
          unit: '3',
          description: '',
        },
        {
          name: 'SCRIPTING',
          unit: '4',
          description: '',
        },
        {
          name: 'TERMINATION',
          unit: '5',
          description: 'only disarm uses this…',
        },
        {
          name: 'CPUFAILSAFE',
          unit: '6',
          description: 'only disarm uses this…',
        },
        {
          name: 'BATTERYFAILSAFE',
          unit: '7',
          description: 'only disarm uses this…',
        },
        {
          name: 'SOLOPAUSEWHENLANDED',
          unit: '8',
          description: 'only disarm uses this…',
        },
        {
          name: 'AFS',
          unit: '9',
          description: 'only disarm uses this…',
        },
        {
          name: 'ADSBCOLLISIONACTION',
          unit: '10',
          description: 'only disarm uses this…',
        },
        {
          name: 'PARACHUTE_RELEASE',
          unit: '11',
          description: 'only disarm uses this…',
        },
        {
          name: 'CRASH',
          unit: '12',
          description: 'only disarm uses this…',
        },
        {
          name: 'LANDED',
          unit: '13',
          description: 'only disarm uses this…',
        },
        {
          name: 'MISSIONEXIT',
          unit: '14',
          description: 'only disarm uses this…',
        },
        {
          name: 'FENCEBREACH',
          unit: '15',
          description: 'only disarm uses this…',
        },
        {
          name: 'RADIOFAILSAFE',
          unit: '16',
          description: 'only disarm uses this…',
        },
        {
          name: 'DISARMDELAY',
          unit: '17',
          description: 'only disarm uses this…',
        },
        {
          name: 'GCSFAILSAFE',
          unit: '18',
          description: 'only disarm uses this…',
        },
        {
          name: 'TERRRAINFAILSAFE',
          unit: '19',
          description: 'only disarm uses this…',
        },
        {
          name: 'FAILSAFE_ACTION_TERMINATE',
          unit: '20',
          description: 'only disarm uses this…',
        },
        {
          name: 'TERRAINFAILSAFE',
          unit: '21',
          description: 'only disarm uses this…',
        },
        {
          name: 'MOTORDETECTDONE',
          unit: '22',
          description: 'only disarm uses this…',
        },
        {
          name: 'BADFLOWOFCONTROL',
          unit: '23',
          description: 'only disarm uses this…',
        },
        {
          name: 'EKFFAILSAFE',
          unit: '24',
          description: 'only disarm uses this…',
        },
        {
          name: 'GCS_FAILSAFE_SURFACEFAILED',
          unit: '25',
          description: 'only disarm uses this…',
        },
        {
          name: 'GCS_FAILSAFE_HOLDFAILED',
          unit: '26',
          description: 'only disarm uses this…',
        },
        {
          name: 'TAKEOFFTIMEOUT',
          unit: '27',
          description: 'only disarm uses this…',
        },
        {
          name: 'AUTOLANDED',
          unit: '28',
          description: 'only disarm uses this…',
        },
        {
          name: 'PILOT_INPUT_FAILSAFE',
          unit: '29',
          description: 'only disarm uses this…',
        },
        {
          name: 'TOYMODELANDTHROTTLE',
          unit: '30',
          description: 'only disarm uses this…',
        },
        {
          name: 'TOYMODELANDFORCE',
          unit: '31',
          description: 'only disarm uses this…',
        },
        {
          name: 'LANDING',
          unit: '32',
          description: 'only disarm uses this…',
        },
        {
          name: 'DEADRECKON_FAILSAFE',
          unit: '33',
          description: 'only disarm uses this…',
        },
        {
          name: 'BLACKBOX',
          unit: '34',
          description: '',
        },
        {
          name: 'DDS',
          unit: '35',
          description: '',
        },
        {
          name: 'AUTO_ARM_ONCE',
          unit: '36',
          description: '',
        },
        {
          name: 'TURTLE_MODE',
          unit: '37',
          description: '',
        },
        {
          name: 'TOYMODE',
          unit: '38',
          description: '',
        },
        {
          name: 'UNKNOWN',
          unit: '100',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'arsp',
    title: 'ARSP',
    description: 'Airspeed sensor data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'Airspeed sensor instance number',
        },
        {
          name: 'Airspeed',
          unit: 'm/s',
          description: 'Current airspeed',
        },
        {
          name: 'DiffPress',
          unit: 'Pa',
          description: 'Pressure difference between static and dynamic port',
        },
        {
          name: 'Temp',
          unit: 'degC',
          description: 'Temperature used for calculation',
        },
        {
          name: 'RawPress',
          unit: 'Pa',
          description: 'Raw pressure less offset',
        },
        {
          name: 'Offset',
          unit: 'Pa',
          description: 'Offset from parameter',
        },
        {
          name: 'U',
          unit: 'True if sensor is being used',
          description: '',
        },
        {
          name: 'H',
          unit: 'True if sensor is healthy',
          description: '',
        },
        {
          name: 'Hp',
          unit: 'Probability sensor is healthy',
          description: '',
        },
        {
          name: 'TR',
          unit: 'innovation test ratio',
          description: '',
        },
        {
          name: 'Pri',
          unit: 'True if sensor is the primary sensor',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'asm1',
    title: 'ASM1',
    description: 'AirSim simulation data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'TUS',
          unit: 'Simulation’s timestamp',
          description: '',
        },
        {
          name: 'R',
          unit: 'Simulation’s roll',
          description: '',
        },
        {
          name: 'P',
          unit: 'Simulation’s pitch',
          description: '',
        },
        {
          name: 'Y',
          unit: 'Simulation’s yaw',
          description: '',
        },
        {
          name: 'GX',
          unit: 'Simulated gyroscope, X-axis',
          description: '',
        },
        {
          name: 'GY',
          unit: 'Simulated gyroscope, Y-axis',
          description: '',
        },
        {
          name: 'GZ',
          unit: 'Simulated gyroscope, Z-axis',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'asm2',
    title: 'ASM2',
    description: 'More AirSim simulation data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'AX',
          unit: 'simulation’s acceleration, X-axis',
          description: '',
        },
        {
          name: 'AY',
          unit: 'simulation’s acceleration, Y-axis',
          description: '',
        },
        {
          name: 'AZ',
          unit: 'simulation’s acceleration, Z-axis',
          description: '',
        },
        {
          name: 'VX',
          unit: 'simulation’s velocity, X-axis',
          description: '',
        },
        {
          name: 'VY',
          unit: 'simulation’s velocity, Y-axis',
          description: '',
        },
        {
          name: 'VZ',
          unit: 'simulation’s velocity, Z-axis',
          description: '',
        },
        {
          name: 'PX',
          unit: 'simulation’s position, X-axis',
          description: '',
        },
        {
          name: 'PY',
          unit: 'simulation’s position, Y-axis',
          description: '',
        },
        {
          name: 'PZ',
          unit: 'simulation’s position, Z-axis',
          description: '',
        },
        {
          name: 'Alt',
          unit: 'simulation’s gps altitude',
          description: '',
        },
        {
          name: 'SD',
          unit: 'simulation’s earth-frame speed-down',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'atde',
    title: 'ATDE',
    description: 'AutoTune data packet',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Angle',
          unit: 'deg',
          description: 'current angle',
        },
        {
          name: 'Rate',
          unit: 'deg/s',
          description: 'current angular rate',
        },
      ],
    ],
  },
  {
    id: 'atrp',
    title: 'ATRP',
    description: 'Plane AutoTune',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Axis',
          unit: 'instance',
          description: 'tuning axis',
        },
        {
          name: 'State',
          unit: 'tuning state',
          description: '',
        },
        {
          name: 'Sur',
          unit: 'deg',
          description: 'control surface deflection',
        },
        {
          name: 'PSlew',
          unit: 'deg/s',
          description: 'P slew rate',
        },
        {
          name: 'DSlew',
          unit: 'deg/s',
          description: 'D slew rate',
        },
        {
          name: 'FF0',
          unit: 'FF value single sample',
          description: '',
        },
        {
          name: 'FF',
          unit: 'FF value',
          description: '',
        },
        {
          name: 'P',
          unit: 'P value',
          description: '',
        },
        {
          name: 'I',
          unit: 'I value',
          description: '',
        },
        {
          name: 'D',
          unit: 'D value',
          description: '',
        },
        {
          name: 'Action',
          unit: 'action taken',
          description: '',
        },
        {
          name: 'RMAX',
          unit: 'deg/s',
          description: 'Rate maximum',
        },
        {
          name: 'TAU',
          unit: 's',
          description: 'time constant',
        },
      ],
    ],
  },
  {
    id: 'atsc',
    title: 'ATSC',
    description: 'Scale factors for attitude controller',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'AngPScX',
          unit: 'Angle P scale X',
          description: '',
        },
        {
          name: 'AngPScY',
          unit: 'Angle P scale Y',
          description: '',
        },
        {
          name: 'AngPScZ',
          unit: 'Angle P scale Z',
          description: '',
        },
        {
          name: 'PDScX',
          unit: 'PD scale X',
          description: '',
        },
        {
          name: 'PDScY',
          unit: 'PD scale Y',
          description: '',
        },
        {
          name: 'PDScZ',
          unit: 'PD scale Z',
          description: '',
        },
        {
          name: 'IScX',
          unit: 'I scale X',
          description: '',
        },
        {
          name: 'IScY',
          unit: 'I scale Y',
          description: '',
        },
        {
          name: 'IScZ',
          unit: 'I scale Z',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'att',
    title: 'ATT',
    description: 'Canonical vehicle attitude',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'DesRoll',
          unit: 'deg',
          description: 'vehicle desired roll',
        },
        {
          name: 'Roll',
          unit: 'deg',
          description: 'achieved vehicle roll',
        },
        {
          name: 'DesPitch',
          unit: 'deg',
          description: 'vehicle desired pitch',
        },
        {
          name: 'Pitch',
          unit: 'deg',
          description: 'achieved vehicle pitch',
        },
        {
          name: 'DesYaw',
          unit: 'degheading',
          description: 'vehicle desired yaw',
        },
        {
          name: 'Yaw',
          unit: 'degheading',
          description: 'achieved vehicle yaw',
        },
        {
          name: 'AEKF',
          unit: 'active EKF type',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'atun',
    title: 'ATUN',
    description: 'Copter/QuadPlane AutoTune',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Axis',
          unit: 'which axis is currently being tuned',
          description: '',
        },
        {
          name: 'TuneStep',
          unit: 'step in autotune process',
          description: '',
        },
        {
          name: 'Targ',
          unit: 'deg',
          description: 'target angle or rate, depending on tuning step',
        },
        {
          name: 'Min',
          unit: 'deg',
          description: 'measured minimum target angle or rate',
        },
        {
          name: 'Max',
          unit: 'deg',
          description: 'measured maximum target angle or rate',
        },
        {
          name: 'RP',
          unit: 'new rate gain P term',
          description: '',
        },
        {
          name: 'RD',
          unit: 'new rate gain D term',
          description: '',
        },
        {
          name: 'SP',
          unit: 'new angle P term',
          description: '',
        },
        {
          name: 'ddt',
          unit: 'm/s/s',
          description: 'maximum measured twitching acceleration',
        },
      ],
    ],
  },
  {
    id: 'auxf',
    title: 'AUXF',
    description: 'Auxiliary function invocation information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'function',
          unit: 'instance',
          description: 'ID of triggered function Values:',
        },
        {
          name: 'DO_NOTHING',
          unit: '0',
          description: 'aux switch disabled',
        },
        {
          name: 'FLIP',
          unit: '2',
          description: 'flip',
        },
        {
          name: 'SIMPLE_MODE',
          unit: '3',
          description: 'change to simple mode',
        },
        {
          name: 'RTL',
          unit: '4',
          description: 'change to RTL flight mode',
        },
        {
          name: 'SAVE_TRIM',
          unit: '5',
          description: 'save current position as level',
        },
        {
          name: 'SAVE_WP',
          unit: '7',
          description: 'save mission waypoint or RTL if in auto mode',
        },
        {
          name: 'CAMERA_TRIGGER',
          unit: '9',
          description: 'trigger camera servo or relay',
        },
        {
          name: 'RANGEFINDER',
          unit: '10',
          description:
            'allow enabling or disabling rangefinder in flight which helps avoid surface tracking when you are far above the ground',
        },
        {
          name: 'FENCE',
          unit: '11',
          description: 'allow enabling or disabling fence in flight',
        },
        {
          name: 'RESETTOARMEDYAW',
          unit: '12',
          description: 'UNUSED',
        },
        {
          name: 'SUPERSIMPLE_MODE',
          unit: '13',
          description: 'change to simple mode in middle, super simple at top',
        },
        {
          name: 'ACRO_TRAINER',
          unit: '14',
          description: 'low = disabled, middle = leveled, high = leveled and limited',
        },
        {
          name: 'SPRAYER',
          unit: '15',
          description: 'enable/disable the crop sprayer',
        },
        {
          name: 'AUTO',
          unit: '16',
          description: 'change to auto flight mode',
        },
        {
          name: 'AUTOTUNE_MODE',
          unit: '17',
          description: 'auto tune',
        },
        {
          name: 'LAND',
          unit: '18',
          description: 'change to LAND flight mode',
        },
        {
          name: 'GRIPPER',
          unit: '19',
          description: 'Operate cargo grippers low=off, middle=neutral, high=on',
        },
        {
          name: 'PARACHUTE_ENABLE',
          unit: '21',
          description: 'Parachute enable/disable',
        },
        {
          name: 'PARACHUTE_RELEASE',
          unit: '22',
          description: 'Parachute release',
        },
        {
          name: 'PARACHUTE_3POS',
          unit: '23',
          description: 'Parachute disable, enable, release with 3 position switch',
        },
        {
          name: 'MISSION_RESET',
          unit: '24',
          description: 'Reset auto mission to start from first command',
        },
        {
          name: 'ATTCON_FEEDFWD',
          unit: '25',
          description: 'enable/disable the roll and pitch rate feed forward',
        },
        {
          name: 'ATTCON_ACCEL_LIM',
          unit: '26',
          description: 'enable/disable the roll, pitch and yaw accel limiting',
        },
        {
          name: 'RETRACT_MOUNT1',
          unit: '27',
          description: 'Retract Mount1',
        },
        {
          name: 'RELAY',
          unit: '28',
          description: 'Relay pin on/off (only supports first relay)',
        },
        {
          name: 'LANDING_GEAR',
          unit: '29',
          description: 'Landing gear controller',
        },
        {
          name: 'LOST_VEHICLE_SOUND',
          unit: '30',
          description: 'Play lost vehicle sound',
        },
        {
          name: 'MOTOR_ESTOP',
          unit: '31',
          description: 'Emergency Stop Switch',
        },
        {
          name: 'MOTOR_INTERLOCK',
          unit: '32',
          description: 'Motor On/Off switch',
        },
        {
          name: 'BRAKE',
          unit: '33',
          description: 'Brake flight mode',
        },
        {
          name: 'RELAY2',
          unit: '34',
          description: 'Relay2 pin on/off',
        },
        {
          name: 'RELAY3',
          unit: '35',
          description: 'Relay3 pin on/off',
        },
        {
          name: 'RELAY4',
          unit: '36',
          description: 'Relay4 pin on/off',
        },
        {
          name: 'THROW',
          unit: '37',
          description: 'change to THROW flight mode',
        },
        {
          name: 'AVOID_ADSB',
          unit: '38',
          description: 'enable AP_Avoidance library',
        },
        {
          name: 'PRECISION_LOITER',
          unit: '39',
          description: 'enable precision loiter',
        },
        {
          name: 'AVOID_PROXIMITY',
          unit: '40',
          description: 'enable object avoidance using proximity sensors (ie. horizontal lidar)',
        },
        {
          name: 'ARMDISARM_UNUSED',
          unit: '41',
          description: 'UNUSED',
        },
        {
          name: 'SMART_RTL',
          unit: '42',
          description: 'change to SmartRTL flight mode',
        },
        {
          name: 'INVERTED',
          unit: '43',
          description: 'enable inverted flight',
        },
        {
          name: 'WINCH_ENABLE',
          unit: '44',
          description: 'winch enable/disable',
        },
        {
          name: 'WINCH_CONTROL',
          unit: '45',
          description: 'winch control',
        },
        {
          name: 'RC_OVERRIDE_ENABLE',
          unit: '46',
          description: 'enable RC Override',
        },
        {
          name: 'USER_FUNC1',
          unit: '47',
          description: 'user function #1',
        },
        {
          name: 'USER_FUNC2',
          unit: '48',
          description: 'user function #2',
        },
        {
          name: 'USER_FUNC3',
          unit: '49',
          description: 'user function #3',
        },
        {
          name: 'LEARN_CRUISE',
          unit: '50',
          description: 'learn cruise throttle (Rover)',
        },
        {
          name: 'MANUAL',
          unit: '51',
          description: 'manual mode',
        },
        {
          name: 'ACRO',
          unit: '52',
          description: 'acro mode',
        },
        {
          name: 'STEERING',
          unit: '53',
          description: 'steering mode',
        },
        {
          name: 'HOLD',
          unit: '54',
          description: 'hold mode',
        },
        {
          name: 'GUIDED',
          unit: '55',
          description: 'guided mode',
        },
        {
          name: 'LOITER',
          unit: '56',
          description: 'loiter mode',
        },
        {
          name: 'FOLLOW',
          unit: '57',
          description: 'follow mode',
        },
        {
          name: 'CLEAR_WP',
          unit: '58',
          description: 'clear waypoints',
        },
        {
          name: 'SIMPLE',
          unit: '59',
          description: 'simple mode',
        },
        {
          name: 'ZIGZAG',
          unit: '60',
          description: 'zigzag mode',
        },
        {
          name: 'ZIGZAG_SaveWP',
          unit: '61',
          description: 'zigzag save waypoint',
        },
        {
          name: 'COMPASS_LEARN',
          unit: '62',
          description: 'learn compass offsets',
        },
        {
          name: 'SAILBOAT_TACK',
          unit: '63',
          description: 'rover sailboat tack',
        },
        {
          name: 'REVERSE_THROTTLE',
          unit: '64',
          description: 'reverse throttle input',
        },
        {
          name: 'GPS_DISABLE',
          unit: '65',
          description: 'disable GPS for testing',
        },
        {
          name: 'RELAY5',
          unit: '66',
          description: 'Relay5 pin on/off',
        },
        {
          name: 'RELAY6',
          unit: '67',
          description: 'Relay6 pin on/off',
        },
        {
          name: 'STABILIZE',
          unit: '68',
          description: 'stabilize mode',
        },
        {
          name: 'POSHOLD',
          unit: '69',
          description: 'poshold mode',
        },
        {
          name: 'ALTHOLD',
          unit: '70',
          description: 'althold mode',
        },
        {
          name: 'FLOWHOLD',
          unit: '71',
          description: 'flowhold mode',
        },
        {
          name: 'CIRCLE',
          unit: '72',
          description: 'circle mode',
        },
        {
          name: 'DRIFT',
          unit: '73',
          description: 'drift mode',
        },
        {
          name: 'SAILBOAT_MOTOR_3POS',
          unit: '74',
          description: 'Sailboat motoring 3pos',
        },
        {
          name: 'SURFACE_TRACKING',
          unit: '75',
          description: 'Surface tracking upwards or downwards',
        },
        {
          name: 'STANDBY',
          unit: '76',
          description: 'Standby mode',
        },
        {
          name: 'TAKEOFF',
          unit: '77',
          description: 'takeoff',
        },
        {
          name: 'RUNCAM_CONTROL',
          unit: '78',
          description: 'control RunCam device',
        },
        {
          name: 'RUNCAM_OSD_CONTROL',
          unit: '79',
          description: 'control RunCam OSD',
        },
        {
          name: 'VISODOM_ALIGN',
          unit: '80',
          description: 'align visual odometry camera’s attitude to AHRS',
        },
        {
          name: 'DISARM',
          unit: '81',
          description: 'disarm vehicle',
        },
        {
          name: 'Q_ASSIST',
          unit: '82',
          description: 'disable, enable and force Q assist',
        },
        {
          name: 'ZIGZAG_Auto',
          unit: '83',
          description: 'zigzag auto switch',
        },
        {
          name: 'AIRMODE',
          unit: '84',
          description: 'enable / disable airmode for copter',
        },
        {
          name: 'GENERATOR',
          unit: '85',
          description: 'generator control',
        },
        {
          name: 'TER_DISABLE',
          unit: '86',
          description: 'disable terrain following in CRUISE/FBWB modes',
        },
        {
          name: 'CROW_SELECT',
          unit: '87',
          description: 'select CROW mode for diff spoilers;high disables,mid forces progressive',
        },
        {
          name: 'SOARING',
          unit: '88',
          description: 'three-position switch to set soaring mode',
        },
        {
          name: 'LANDING_FLARE',
          unit: '89',
          description: 'force flare, throttle forced idle, pitch to LAND_PITCH_DEG, tilts up',
        },
        {
          name: 'EKF_SOURCE_SET',
          unit: '90',
          description: 'change EKF data source set between primary, secondary and tertiary',
        },
        {
          name: 'ARSPD_CALIBRATE',
          unit: '91',
          description: 'calibrate airspeed ratio',
        },
        {
          name: 'FBWA',
          unit: '92',
          description: 'Fly-By-Wire-A',
        },
        {
          name: 'RELOCATE_MISSION',
          unit: '93',
          description: 'used in separate branch MISSION_RELATIVE',
        },
        {
          name: 'VTX_POWER',
          unit: '94',
          description: 'VTX power level',
        },
        {
          name: 'FBWA_TAILDRAGGER',
          unit: '95',
          description:
            'enables FBWA taildragger takeoff mode. Once this feature is enabled it will stay enabled until the aircraft goes above TKOFF_TDRAG_SPD1 airspeed, changes mode, or the pitch goes above the initial pitch when this is engaged or goes below 0 pitch. When enabled the elevator will be forced to TKOFF_TDRAG_ELEV. This option allows for easier takeoffs on taildraggers in FBWA mode, and also makes it easier to test auto-takeoff steering handling in FBWA.',
        },
        {
          name: 'MODE_SWITCH_RESET',
          unit: '96',
          description: 'trigger re-reading of mode switch',
        },
        {
          name: 'WIND_VANE_DIR_OFSSET',
          unit: '97',
          description: 'flag for windvane direction offset input, used with windvane type 2',
        },
        {
          name: 'TRAINING',
          unit: '98',
          description: 'mode training',
        },
        {
          name: 'AUTO_RTL',
          unit: '99',
          description: 'AUTO RTL via DO_LAND_START',
        },
        {
          name: 'KILL_IMU1',
          unit: '100',
          description: 'disable first IMU (for IMU failure testing)',
        },
        {
          name: 'KILL_IMU2',
          unit: '101',
          description: 'disable second IMU (for IMU failure testing)',
        },
        {
          name: 'CAM_MODE_TOGGLE',
          unit: '102',
          description: 'Momentary switch to cycle camera modes',
        },
        {
          name: 'EKF_LANE_SWITCH',
          unit: '103',
          description: 'trigger lane switch attempt',
        },
        {
          name: 'EKF_YAW_RESET',
          unit: '104',
          description: 'trigger yaw reset attempt',
        },
        {
          name: 'GPS_DISABLE_YAW',
          unit: '105',
          description: 'disable GPS yaw for testing',
        },
        {
          name: 'DISABLE_AIRSPEED_USE',
          unit: '106',
          description: 'equivalent to AIRSPEED_USE 0',
        },
        {
          name: 'FW_AUTOTUNE',
          unit: '107',
          description: 'fixed wing auto tune',
        },
        {
          name: 'QRTL',
          unit: '108',
          description: 'QRTL mode',
        },
        {
          name: 'CUSTOM_CONTROLLER',
          unit: '109',
          description: 'use Custom Controller',
        },
        {
          name: 'KILL_IMU3',
          unit: '110',
          description: 'disable third IMU (for IMU failure testing)',
        },
        {
          name: 'LOWEHEISER_STARTER',
          unit: '111',
          description: 'allows for manually running starter',
        },
        {
          name: 'AHRS_TYPE',
          unit: '112',
          description: 'change AHRS_EKF_TYPE',
        },
        {
          name: 'RETRACT_MOUNT2',
          unit: '113',
          description: 'Retract Mount2',
        },
        {
          name: 'CRUISE',
          unit: '150',
          description: 'CRUISE mode',
        },
        {
          name: 'TURTLE',
          unit: '151',
          description: 'Turtle mode - flip over after crash',
        },
        {
          name: 'SIMPLE_HEADING_RESET',
          unit: '152',
          description: 'reset simple mode reference heading to current',
        },
        {
          name: 'ARMDISARM',
          unit: '153',
          description: 'arm or disarm vehicle',
        },
        {
          name: 'ARMDISARM_AIRMODE',
          unit: '154',
          description: 'arm or disarm vehicle enabling airmode',
        },
        {
          name: 'TRIM_TO_CURRENT_SERVO_RC',
          unit: '155',
          description: 'trim to current servo and RC',
        },
        {
          name: 'TORQEEDO_CLEAR_ERR',
          unit: '156',
          description: 'clear torqeedo error',
        },
        {
          name: 'EMERGENCY_LANDING_EN',
          unit: '157',
          description: 'Force long FS action to FBWA for landing out of range',
        },
        {
          name: 'OPTFLOW_CAL',
          unit: '158',
          description: 'optical flow calibration',
        },
        {
          name: 'FORCEFLYING',
          unit: '159',
          description:
            'enable or disable land detection for GPS based manual modes preventing land detection and maintainting set_throttle_mix_max',
        },
        {
          name: 'WEATHER_VANE_ENABLE',
          unit: '160',
          description: 'enable/disable weathervaning',
        },
        {
          name: 'TURBINE_START',
          unit: '161',
          description: 'initialize turbine start sequence',
        },
        {
          name: 'FFT_NOTCH_TUNE',
          unit: '162',
          description: 'FFT notch tuning function',
        },
        {
          name: 'MOUNT_LOCK',
          unit: '163',
          description: 'Mount yaw lock vs follow',
        },
        {
          name: 'LOG_PAUSE',
          unit: '164',
          description: 'Pauses logging if under logging rate control',
        },
        {
          name: 'ARM_EMERGENCY_STOP',
          unit: '165',
          description: 'ARM on high, MOTOR_ESTOP on low',
        },
        {
          name: 'CAMERA_REC_VIDEO',
          unit: '166',
          description: 'start recording on high, stop recording on low',
        },
        {
          name: 'CAMERA_ZOOM',
          unit: '167',
          description: 'camera zoom high = zoom in, middle = hold, low = zoom out',
        },
        {
          name: 'CAMERA_MANUAL_FOCUS',
          unit: '168',
          description:
            'camera manual focus. high = long shot, middle = stop focus, low = close shot',
        },
        {
          name: 'CAMERA_AUTO_FOCUS',
          unit: '169',
          description: 'camera auto focus',
        },
        {
          name: 'QSTABILIZE',
          unit: '170',
          description: 'QuadPlane QStabilize mode',
        },
        {
          name: 'MAG_CAL',
          unit: '171',
          description: 'Calibrate compasses (disarmed only)',
        },
        {
          name: 'BATTERY_MPPT_ENABLE',
          unit: '172',
          description:
            'Battery MPPT Power enable. high = ON, mid = auto (controlled by mppt/batt driver), low = OFF. This effects all MPPTs.',
        },
        {
          name: 'PLANE_AUTO_LANDING_ABORT',
          unit: '173',
          description:
            'Abort Glide-slope or VTOL landing during payload place or do_land type mission items',
        },
        {
          name: 'CAMERA_IMAGE_TRACKING',
          unit: '174',
          description: 'camera image tracking',
        },
        {
          name: 'CAMERA_LENS',
          unit: '175',
          description: 'camera lens selection',
        },
        {
          name: 'VFWD_THR_OVERRIDE',
          unit: '176',
          description: 'force enabled VTOL forward throttle method',
        },
        {
          name: 'MOUNT_LRF_ENABLE',
          unit: '177',
          description: 'mount LRF enable/disable',
        },
        {
          name: 'FLIGHTMODE_PAUSE',
          unit: '178',
          description: 'e.g. pause movement towards waypoint',
        },
        {
          name: 'ICE_START_STOP',
          unit: '179',
          description: 'AP_ICEngine start stop',
        },
        {
          name: 'AUTOTUNE_TEST_GAINS',
          unit: '180',
          description: 'auto tune tuning switch to test or revert gains',
        },
        {
          name: 'QUICKTUNE',
          unit: '181',
          description: 'quicktune 3 position switch',
        },
        {
          name: 'AHRS_AUTO_TRIM',
          unit: '182',
          description: 'in-flight AHRS autotrim',
        },
        {
          name: 'AUTOLAND',
          unit: '183',
          description: 'Fixed Wing AUTOLAND Mode',
        },
        {
          name: 'SYSTEMID',
          unit: '184',
          description: 'system ID as an aux switch',
        },
        {
          name: 'ROLL',
          unit: '201',
          description: 'roll input',
        },
        {
          name: 'PITCH',
          unit: '202',
          description: 'pitch input',
        },
        {
          name: 'THROTTLE',
          unit: '203',
          description: 'throttle pilot input',
        },
        {
          name: 'YAW',
          unit: '204',
          description: 'yaw pilot input',
        },
        {
          name: 'MAINSAIL',
          unit: '207',
          description: 'mainsail input',
        },
        {
          name: 'FLAP',
          unit: '208',
          description: 'flap input',
        },
        {
          name: 'FWD_THR',
          unit: '209',
          description: 'VTOL manual forward throttle',
        },
        {
          name: 'AIRBRAKE',
          unit: '210',
          description: 'manual airbrake control',
        },
        {
          name: 'WALKING_HEIGHT',
          unit: '211',
          description: 'walking robot height input',
        },
        {
          name: 'MOUNT1_ROLL',
          unit: '212',
          description: 'mount1 roll input',
        },
        {
          name: 'MOUNT1_PITCH',
          unit: '213',
          description: 'mount1 pitch input',
        },
        {
          name: 'MOUNT1_YAW',
          unit: '214',
          description: 'mount1 yaw input',
        },
        {
          name: 'MOUNT2_ROLL',
          unit: '215',
          description: 'mount2 roll input',
        },
        {
          name: 'MOUNT2_PITCH',
          unit: '216',
          description: 'mount3 pitch input',
        },
        {
          name: 'MOUNT2_YAW',
          unit: '217',
          description: 'mount4 yaw input',
        },
        {
          name: 'LOWEHEISER_THROTTLE',
          unit: '218',
          description: 'allows for throttle on slider',
        },
        {
          name: 'TRANSMITTER_TUNING',
          unit: '219',
          description: 'use a transmitter knob or slider for in-flight tuning',
        },
        {
          name: 'TRANSMITTER_TUNING2',
          unit: '220',
          description: 'use another transmitter knob or slider for in-flight tuning',
        },
        {
          name: 'SCRIPTING_1',
          unit: '300',
          description: '',
        },
        {
          name: 'SCRIPTING_2',
          unit: '301',
          description: '',
        },
        {
          name: 'SCRIPTING_3',
          unit: '302',
          description: '',
        },
        {
          name: 'SCRIPTING_4',
          unit: '303',
          description: '',
        },
        {
          name: 'SCRIPTING_5',
          unit: '304',
          description: '',
        },
        {
          name: 'SCRIPTING_6',
          unit: '305',
          description: '',
        },
        {
          name: 'SCRIPTING_7',
          unit: '306',
          description: '',
        },
        {
          name: 'SCRIPTING_8',
          unit: '307',
          description: '',
        },
        {
          name: 'SCRIPTING_9',
          unit: '308',
          description: '',
        },
        {
          name: 'SCRIPTING_10',
          unit: '309',
          description: '',
        },
        {
          name: 'SCRIPTING_11',
          unit: '310',
          description: '',
        },
        {
          name: 'SCRIPTING_12',
          unit: '311',
          description: '',
        },
        {
          name: 'SCRIPTING_13',
          unit: '312',
          description: '',
        },
        {
          name: 'SCRIPTING_14',
          unit: '313',
          description: '',
        },
        {
          name: 'SCRIPTING_15',
          unit: '314',
          description: '',
        },
        {
          name: 'SCRIPTING_16',
          unit: '315',
          description: '',
        },
        {
          name: 'STOP_RESTART_SCRIPTING',
          unit: '316',
          description: 'emergency scripting disablement',
        },
        {
          name: 'AUX_FUNCTION_MAX',
          unit: '317',
          description: '',
        },
        {
          name: 'pos',
          unit: 'enum',
          description: 'switch position when function triggered Values:',
        },
        {
          name: 'LOW',
          unit: '0',
          description: 'indicates auxiliary switch is in the low position (pwm <1200)',
        },
        {
          name: 'MIDDLE',
          unit: '1',
          description: 'indicates auxiliary switch is in the middle position (pwm >1200, <1800)',
        },
        {
          name: 'HIGH',
          unit: '2',
          description: 'indicates auxiliary switch is in the high position (pwm >1800)',
        },
        {
          name: 'source',
          unit: 'enum',
          description: 'source of auxiliary function invocation Values:',
        },
        {
          name: 'INIT',
          unit: '0',
          description: 'Source index is RC channel index',
        },
        {
          name: 'RC',
          unit: '1',
          description: 'Source index is RC channel index',
        },
        {
          name: 'BUTTON',
          unit: '2',
          description: 'Source index is button index',
        },
        {
          name: 'MAVLINK',
          unit: '3',
          description: 'Source index is MAVLink channel number',
        },
        {
          name: 'MISSION',
          unit: '4',
          description: 'Source index is mission item index',
        },
        {
          name: 'SCRIPTING',
          unit: '5',
          description: 'Source index is not used (always 0)',
        },
        {
          name: 'index',
          unit: 'index within source. 0 indexed. Invalid for scripting.',
          description: '',
        },
        {
          name: 'result',
          unit: 'true if function was successful',
          description: '',
        },
      ],
      [
        {
          name: 'DO_NOTHING',
          unit: '0',
          description: 'aux switch disabled',
        },
        {
          name: 'FLIP',
          unit: '2',
          description: 'flip',
        },
        {
          name: 'SIMPLE_MODE',
          unit: '3',
          description: 'change to simple mode',
        },
        {
          name: 'RTL',
          unit: '4',
          description: 'change to RTL flight mode',
        },
        {
          name: 'SAVE_TRIM',
          unit: '5',
          description: 'save current position as level',
        },
        {
          name: 'SAVE_WP',
          unit: '7',
          description: 'save mission waypoint or RTL if in auto mode',
        },
        {
          name: 'CAMERA_TRIGGER',
          unit: '9',
          description: 'trigger camera servo or relay',
        },
        {
          name: 'RANGEFINDER',
          unit: '10',
          description:
            'allow enabling or disabling rangefinder in flight which helps avoid surface tracking when you are far above the ground',
        },
        {
          name: 'FENCE',
          unit: '11',
          description: 'allow enabling or disabling fence in flight',
        },
        {
          name: 'RESETTOARMEDYAW',
          unit: '12',
          description: 'UNUSED',
        },
        {
          name: 'SUPERSIMPLE_MODE',
          unit: '13',
          description: 'change to simple mode in middle, super simple at top',
        },
        {
          name: 'ACRO_TRAINER',
          unit: '14',
          description: 'low = disabled, middle = leveled, high = leveled and limited',
        },
        {
          name: 'SPRAYER',
          unit: '15',
          description: 'enable/disable the crop sprayer',
        },
        {
          name: 'AUTO',
          unit: '16',
          description: 'change to auto flight mode',
        },
        {
          name: 'AUTOTUNE_MODE',
          unit: '17',
          description: 'auto tune',
        },
        {
          name: 'LAND',
          unit: '18',
          description: 'change to LAND flight mode',
        },
        {
          name: 'GRIPPER',
          unit: '19',
          description: 'Operate cargo grippers low=off, middle=neutral, high=on',
        },
        {
          name: 'PARACHUTE_ENABLE',
          unit: '21',
          description: 'Parachute enable/disable',
        },
        {
          name: 'PARACHUTE_RELEASE',
          unit: '22',
          description: 'Parachute release',
        },
        {
          name: 'PARACHUTE_3POS',
          unit: '23',
          description: 'Parachute disable, enable, release with 3 position switch',
        },
        {
          name: 'MISSION_RESET',
          unit: '24',
          description: 'Reset auto mission to start from first command',
        },
        {
          name: 'ATTCON_FEEDFWD',
          unit: '25',
          description: 'enable/disable the roll and pitch rate feed forward',
        },
        {
          name: 'ATTCON_ACCEL_LIM',
          unit: '26',
          description: 'enable/disable the roll, pitch and yaw accel limiting',
        },
        {
          name: 'RETRACT_MOUNT1',
          unit: '27',
          description: 'Retract Mount1',
        },
        {
          name: 'RELAY',
          unit: '28',
          description: 'Relay pin on/off (only supports first relay)',
        },
        {
          name: 'LANDING_GEAR',
          unit: '29',
          description: 'Landing gear controller',
        },
        {
          name: 'LOST_VEHICLE_SOUND',
          unit: '30',
          description: 'Play lost vehicle sound',
        },
        {
          name: 'MOTOR_ESTOP',
          unit: '31',
          description: 'Emergency Stop Switch',
        },
        {
          name: 'MOTOR_INTERLOCK',
          unit: '32',
          description: 'Motor On/Off switch',
        },
        {
          name: 'BRAKE',
          unit: '33',
          description: 'Brake flight mode',
        },
        {
          name: 'RELAY2',
          unit: '34',
          description: 'Relay2 pin on/off',
        },
        {
          name: 'RELAY3',
          unit: '35',
          description: 'Relay3 pin on/off',
        },
        {
          name: 'RELAY4',
          unit: '36',
          description: 'Relay4 pin on/off',
        },
        {
          name: 'THROW',
          unit: '37',
          description: 'change to THROW flight mode',
        },
        {
          name: 'AVOID_ADSB',
          unit: '38',
          description: 'enable AP_Avoidance library',
        },
        {
          name: 'PRECISION_LOITER',
          unit: '39',
          description: 'enable precision loiter',
        },
        {
          name: 'AVOID_PROXIMITY',
          unit: '40',
          description: 'enable object avoidance using proximity sensors (ie. horizontal lidar)',
        },
        {
          name: 'ARMDISARM_UNUSED',
          unit: '41',
          description: 'UNUSED',
        },
        {
          name: 'SMART_RTL',
          unit: '42',
          description: 'change to SmartRTL flight mode',
        },
        {
          name: 'INVERTED',
          unit: '43',
          description: 'enable inverted flight',
        },
        {
          name: 'WINCH_ENABLE',
          unit: '44',
          description: 'winch enable/disable',
        },
        {
          name: 'WINCH_CONTROL',
          unit: '45',
          description: 'winch control',
        },
        {
          name: 'RC_OVERRIDE_ENABLE',
          unit: '46',
          description: 'enable RC Override',
        },
        {
          name: 'USER_FUNC1',
          unit: '47',
          description: 'user function #1',
        },
        {
          name: 'USER_FUNC2',
          unit: '48',
          description: 'user function #2',
        },
        {
          name: 'USER_FUNC3',
          unit: '49',
          description: 'user function #3',
        },
        {
          name: 'LEARN_CRUISE',
          unit: '50',
          description: 'learn cruise throttle (Rover)',
        },
        {
          name: 'MANUAL',
          unit: '51',
          description: 'manual mode',
        },
        {
          name: 'ACRO',
          unit: '52',
          description: 'acro mode',
        },
        {
          name: 'STEERING',
          unit: '53',
          description: 'steering mode',
        },
        {
          name: 'HOLD',
          unit: '54',
          description: 'hold mode',
        },
        {
          name: 'GUIDED',
          unit: '55',
          description: 'guided mode',
        },
        {
          name: 'LOITER',
          unit: '56',
          description: 'loiter mode',
        },
        {
          name: 'FOLLOW',
          unit: '57',
          description: 'follow mode',
        },
        {
          name: 'CLEAR_WP',
          unit: '58',
          description: 'clear waypoints',
        },
        {
          name: 'SIMPLE',
          unit: '59',
          description: 'simple mode',
        },
        {
          name: 'ZIGZAG',
          unit: '60',
          description: 'zigzag mode',
        },
        {
          name: 'ZIGZAG_SaveWP',
          unit: '61',
          description: 'zigzag save waypoint',
        },
        {
          name: 'COMPASS_LEARN',
          unit: '62',
          description: 'learn compass offsets',
        },
        {
          name: 'SAILBOAT_TACK',
          unit: '63',
          description: 'rover sailboat tack',
        },
        {
          name: 'REVERSE_THROTTLE',
          unit: '64',
          description: 'reverse throttle input',
        },
        {
          name: 'GPS_DISABLE',
          unit: '65',
          description: 'disable GPS for testing',
        },
        {
          name: 'RELAY5',
          unit: '66',
          description: 'Relay5 pin on/off',
        },
        {
          name: 'RELAY6',
          unit: '67',
          description: 'Relay6 pin on/off',
        },
        {
          name: 'STABILIZE',
          unit: '68',
          description: 'stabilize mode',
        },
        {
          name: 'POSHOLD',
          unit: '69',
          description: 'poshold mode',
        },
        {
          name: 'ALTHOLD',
          unit: '70',
          description: 'althold mode',
        },
        {
          name: 'FLOWHOLD',
          unit: '71',
          description: 'flowhold mode',
        },
        {
          name: 'CIRCLE',
          unit: '72',
          description: 'circle mode',
        },
        {
          name: 'DRIFT',
          unit: '73',
          description: 'drift mode',
        },
        {
          name: 'SAILBOAT_MOTOR_3POS',
          unit: '74',
          description: 'Sailboat motoring 3pos',
        },
        {
          name: 'SURFACE_TRACKING',
          unit: '75',
          description: 'Surface tracking upwards or downwards',
        },
        {
          name: 'STANDBY',
          unit: '76',
          description: 'Standby mode',
        },
        {
          name: 'TAKEOFF',
          unit: '77',
          description: 'takeoff',
        },
        {
          name: 'RUNCAM_CONTROL',
          unit: '78',
          description: 'control RunCam device',
        },
        {
          name: 'RUNCAM_OSD_CONTROL',
          unit: '79',
          description: 'control RunCam OSD',
        },
        {
          name: 'VISODOM_ALIGN',
          unit: '80',
          description: 'align visual odometry camera’s attitude to AHRS',
        },
        {
          name: 'DISARM',
          unit: '81',
          description: 'disarm vehicle',
        },
        {
          name: 'Q_ASSIST',
          unit: '82',
          description: 'disable, enable and force Q assist',
        },
        {
          name: 'ZIGZAG_Auto',
          unit: '83',
          description: 'zigzag auto switch',
        },
        {
          name: 'AIRMODE',
          unit: '84',
          description: 'enable / disable airmode for copter',
        },
        {
          name: 'GENERATOR',
          unit: '85',
          description: 'generator control',
        },
        {
          name: 'TER_DISABLE',
          unit: '86',
          description: 'disable terrain following in CRUISE/FBWB modes',
        },
        {
          name: 'CROW_SELECT',
          unit: '87',
          description: 'select CROW mode for diff spoilers;high disables,mid forces progressive',
        },
        {
          name: 'SOARING',
          unit: '88',
          description: 'three-position switch to set soaring mode',
        },
        {
          name: 'LANDING_FLARE',
          unit: '89',
          description: 'force flare, throttle forced idle, pitch to LAND_PITCH_DEG, tilts up',
        },
        {
          name: 'EKF_SOURCE_SET',
          unit: '90',
          description: 'change EKF data source set between primary, secondary and tertiary',
        },
        {
          name: 'ARSPD_CALIBRATE',
          unit: '91',
          description: 'calibrate airspeed ratio',
        },
        {
          name: 'FBWA',
          unit: '92',
          description: 'Fly-By-Wire-A',
        },
        {
          name: 'RELOCATE_MISSION',
          unit: '93',
          description: 'used in separate branch MISSION_RELATIVE',
        },
        {
          name: 'VTX_POWER',
          unit: '94',
          description: 'VTX power level',
        },
        {
          name: 'FBWA_TAILDRAGGER',
          unit: '95',
          description:
            'enables FBWA taildragger takeoff mode. Once this feature is enabled it will stay enabled until the aircraft goes above TKOFF_TDRAG_SPD1 airspeed, changes mode, or the pitch goes above the initial pitch when this is engaged or goes below 0 pitch. When enabled the elevator will be forced to TKOFF_TDRAG_ELEV. This option allows for easier takeoffs on taildraggers in FBWA mode, and also makes it easier to test auto-takeoff steering handling in FBWA.',
        },
        {
          name: 'MODE_SWITCH_RESET',
          unit: '96',
          description: 'trigger re-reading of mode switch',
        },
        {
          name: 'WIND_VANE_DIR_OFSSET',
          unit: '97',
          description: 'flag for windvane direction offset input, used with windvane type 2',
        },
        {
          name: 'TRAINING',
          unit: '98',
          description: 'mode training',
        },
        {
          name: 'AUTO_RTL',
          unit: '99',
          description: 'AUTO RTL via DO_LAND_START',
        },
        {
          name: 'KILL_IMU1',
          unit: '100',
          description: 'disable first IMU (for IMU failure testing)',
        },
        {
          name: 'KILL_IMU2',
          unit: '101',
          description: 'disable second IMU (for IMU failure testing)',
        },
        {
          name: 'CAM_MODE_TOGGLE',
          unit: '102',
          description: 'Momentary switch to cycle camera modes',
        },
        {
          name: 'EKF_LANE_SWITCH',
          unit: '103',
          description: 'trigger lane switch attempt',
        },
        {
          name: 'EKF_YAW_RESET',
          unit: '104',
          description: 'trigger yaw reset attempt',
        },
        {
          name: 'GPS_DISABLE_YAW',
          unit: '105',
          description: 'disable GPS yaw for testing',
        },
        {
          name: 'DISABLE_AIRSPEED_USE',
          unit: '106',
          description: 'equivalent to AIRSPEED_USE 0',
        },
        {
          name: 'FW_AUTOTUNE',
          unit: '107',
          description: 'fixed wing auto tune',
        },
        {
          name: 'QRTL',
          unit: '108',
          description: 'QRTL mode',
        },
        {
          name: 'CUSTOM_CONTROLLER',
          unit: '109',
          description: 'use Custom Controller',
        },
        {
          name: 'KILL_IMU3',
          unit: '110',
          description: 'disable third IMU (for IMU failure testing)',
        },
        {
          name: 'LOWEHEISER_STARTER',
          unit: '111',
          description: 'allows for manually running starter',
        },
        {
          name: 'AHRS_TYPE',
          unit: '112',
          description: 'change AHRS_EKF_TYPE',
        },
        {
          name: 'RETRACT_MOUNT2',
          unit: '113',
          description: 'Retract Mount2',
        },
        {
          name: 'CRUISE',
          unit: '150',
          description: 'CRUISE mode',
        },
        {
          name: 'TURTLE',
          unit: '151',
          description: 'Turtle mode - flip over after crash',
        },
        {
          name: 'SIMPLE_HEADING_RESET',
          unit: '152',
          description: 'reset simple mode reference heading to current',
        },
        {
          name: 'ARMDISARM',
          unit: '153',
          description: 'arm or disarm vehicle',
        },
        {
          name: 'ARMDISARM_AIRMODE',
          unit: '154',
          description: 'arm or disarm vehicle enabling airmode',
        },
        {
          name: 'TRIM_TO_CURRENT_SERVO_RC',
          unit: '155',
          description: 'trim to current servo and RC',
        },
        {
          name: 'TORQEEDO_CLEAR_ERR',
          unit: '156',
          description: 'clear torqeedo error',
        },
        {
          name: 'EMERGENCY_LANDING_EN',
          unit: '157',
          description: 'Force long FS action to FBWA for landing out of range',
        },
        {
          name: 'OPTFLOW_CAL',
          unit: '158',
          description: 'optical flow calibration',
        },
        {
          name: 'FORCEFLYING',
          unit: '159',
          description:
            'enable or disable land detection for GPS based manual modes preventing land detection and maintainting set_throttle_mix_max',
        },
        {
          name: 'WEATHER_VANE_ENABLE',
          unit: '160',
          description: 'enable/disable weathervaning',
        },
        {
          name: 'TURBINE_START',
          unit: '161',
          description: 'initialize turbine start sequence',
        },
        {
          name: 'FFT_NOTCH_TUNE',
          unit: '162',
          description: 'FFT notch tuning function',
        },
        {
          name: 'MOUNT_LOCK',
          unit: '163',
          description: 'Mount yaw lock vs follow',
        },
        {
          name: 'LOG_PAUSE',
          unit: '164',
          description: 'Pauses logging if under logging rate control',
        },
        {
          name: 'ARM_EMERGENCY_STOP',
          unit: '165',
          description: 'ARM on high, MOTOR_ESTOP on low',
        },
        {
          name: 'CAMERA_REC_VIDEO',
          unit: '166',
          description: 'start recording on high, stop recording on low',
        },
        {
          name: 'CAMERA_ZOOM',
          unit: '167',
          description: 'camera zoom high = zoom in, middle = hold, low = zoom out',
        },
        {
          name: 'CAMERA_MANUAL_FOCUS',
          unit: '168',
          description:
            'camera manual focus. high = long shot, middle = stop focus, low = close shot',
        },
        {
          name: 'CAMERA_AUTO_FOCUS',
          unit: '169',
          description: 'camera auto focus',
        },
        {
          name: 'QSTABILIZE',
          unit: '170',
          description: 'QuadPlane QStabilize mode',
        },
        {
          name: 'MAG_CAL',
          unit: '171',
          description: 'Calibrate compasses (disarmed only)',
        },
        {
          name: 'BATTERY_MPPT_ENABLE',
          unit: '172',
          description:
            'Battery MPPT Power enable. high = ON, mid = auto (controlled by mppt/batt driver), low = OFF. This effects all MPPTs.',
        },
        {
          name: 'PLANE_AUTO_LANDING_ABORT',
          unit: '173',
          description:
            'Abort Glide-slope or VTOL landing during payload place or do_land type mission items',
        },
        {
          name: 'CAMERA_IMAGE_TRACKING',
          unit: '174',
          description: 'camera image tracking',
        },
        {
          name: 'CAMERA_LENS',
          unit: '175',
          description: 'camera lens selection',
        },
        {
          name: 'VFWD_THR_OVERRIDE',
          unit: '176',
          description: 'force enabled VTOL forward throttle method',
        },
        {
          name: 'MOUNT_LRF_ENABLE',
          unit: '177',
          description: 'mount LRF enable/disable',
        },
        {
          name: 'FLIGHTMODE_PAUSE',
          unit: '178',
          description: 'e.g. pause movement towards waypoint',
        },
        {
          name: 'ICE_START_STOP',
          unit: '179',
          description: 'AP_ICEngine start stop',
        },
        {
          name: 'AUTOTUNE_TEST_GAINS',
          unit: '180',
          description: 'auto tune tuning switch to test or revert gains',
        },
        {
          name: 'QUICKTUNE',
          unit: '181',
          description: 'quicktune 3 position switch',
        },
        {
          name: 'AHRS_AUTO_TRIM',
          unit: '182',
          description: 'in-flight AHRS autotrim',
        },
        {
          name: 'AUTOLAND',
          unit: '183',
          description: 'Fixed Wing AUTOLAND Mode',
        },
        {
          name: 'SYSTEMID',
          unit: '184',
          description: 'system ID as an aux switch',
        },
        {
          name: 'ROLL',
          unit: '201',
          description: 'roll input',
        },
        {
          name: 'PITCH',
          unit: '202',
          description: 'pitch input',
        },
        {
          name: 'THROTTLE',
          unit: '203',
          description: 'throttle pilot input',
        },
        {
          name: 'YAW',
          unit: '204',
          description: 'yaw pilot input',
        },
        {
          name: 'MAINSAIL',
          unit: '207',
          description: 'mainsail input',
        },
        {
          name: 'FLAP',
          unit: '208',
          description: 'flap input',
        },
        {
          name: 'FWD_THR',
          unit: '209',
          description: 'VTOL manual forward throttle',
        },
        {
          name: 'AIRBRAKE',
          unit: '210',
          description: 'manual airbrake control',
        },
        {
          name: 'WALKING_HEIGHT',
          unit: '211',
          description: 'walking robot height input',
        },
        {
          name: 'MOUNT1_ROLL',
          unit: '212',
          description: 'mount1 roll input',
        },
        {
          name: 'MOUNT1_PITCH',
          unit: '213',
          description: 'mount1 pitch input',
        },
        {
          name: 'MOUNT1_YAW',
          unit: '214',
          description: 'mount1 yaw input',
        },
        {
          name: 'MOUNT2_ROLL',
          unit: '215',
          description: 'mount2 roll input',
        },
        {
          name: 'MOUNT2_PITCH',
          unit: '216',
          description: 'mount3 pitch input',
        },
        {
          name: 'MOUNT2_YAW',
          unit: '217',
          description: 'mount4 yaw input',
        },
        {
          name: 'LOWEHEISER_THROTTLE',
          unit: '218',
          description: 'allows for throttle on slider',
        },
        {
          name: 'TRANSMITTER_TUNING',
          unit: '219',
          description: 'use a transmitter knob or slider for in-flight tuning',
        },
        {
          name: 'TRANSMITTER_TUNING2',
          unit: '220',
          description: 'use another transmitter knob or slider for in-flight tuning',
        },
        {
          name: 'SCRIPTING_1',
          unit: '300',
          description: '',
        },
        {
          name: 'SCRIPTING_2',
          unit: '301',
          description: '',
        },
        {
          name: 'SCRIPTING_3',
          unit: '302',
          description: '',
        },
        {
          name: 'SCRIPTING_4',
          unit: '303',
          description: '',
        },
        {
          name: 'SCRIPTING_5',
          unit: '304',
          description: '',
        },
        {
          name: 'SCRIPTING_6',
          unit: '305',
          description: '',
        },
        {
          name: 'SCRIPTING_7',
          unit: '306',
          description: '',
        },
        {
          name: 'SCRIPTING_8',
          unit: '307',
          description: '',
        },
        {
          name: 'SCRIPTING_9',
          unit: '308',
          description: '',
        },
        {
          name: 'SCRIPTING_10',
          unit: '309',
          description: '',
        },
        {
          name: 'SCRIPTING_11',
          unit: '310',
          description: '',
        },
        {
          name: 'SCRIPTING_12',
          unit: '311',
          description: '',
        },
        {
          name: 'SCRIPTING_13',
          unit: '312',
          description: '',
        },
        {
          name: 'SCRIPTING_14',
          unit: '313',
          description: '',
        },
        {
          name: 'SCRIPTING_15',
          unit: '314',
          description: '',
        },
        {
          name: 'SCRIPTING_16',
          unit: '315',
          description: '',
        },
        {
          name: 'STOP_RESTART_SCRIPTING',
          unit: '316',
          description: 'emergency scripting disablement',
        },
        {
          name: 'AUX_FUNCTION_MAX',
          unit: '317',
          description: '',
        },
      ],
      [
        {
          name: 'LOW',
          unit: '0',
          description: 'indicates auxiliary switch is in the low position (pwm <1200)',
        },
        {
          name: 'MIDDLE',
          unit: '1',
          description: 'indicates auxiliary switch is in the middle position (pwm >1200, <1800)',
        },
        {
          name: 'HIGH',
          unit: '2',
          description: 'indicates auxiliary switch is in the high position (pwm >1800)',
        },
      ],
      [
        {
          name: 'INIT',
          unit: '0',
          description: 'Source index is RC channel index',
        },
        {
          name: 'RC',
          unit: '1',
          description: 'Source index is RC channel index',
        },
        {
          name: 'BUTTON',
          unit: '2',
          description: 'Source index is button index',
        },
        {
          name: 'MAVLINK',
          unit: '3',
          description: 'Source index is MAVLink channel number',
        },
        {
          name: 'MISSION',
          unit: '4',
          description: 'Source index is mission item index',
        },
        {
          name: 'SCRIPTING',
          unit: '5',
          description: 'Source index is not used (always 0)',
        },
      ],
    ],
  },
  {
    id: 'bard',
    title: 'BARD',
    description: 'Barometer dynamic data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'barometer sensor instance number',
        },
        {
          name: 'DynPrX',
          unit: 'Pa',
          description: 'calculated dynamic pressure in the bodyframe X-axis',
        },
        {
          name: 'DynPrY',
          unit: 'Pa',
          description: 'calculated dynamic pressure in the bodyframe Y-axis',
        },
        {
          name: 'DynPrZ',
          unit: 'Pa',
          description: 'calculated dynamic pressure in the bodyframe Z-axis',
        },
      ],
    ],
  },
  {
    id: 'baro',
    title: 'BARO',
    description: 'Gathered Barometer data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'barometer sensor instance number',
        },
        {
          name: 'Alt',
          unit: 'm',
          description: 'calculated altitude',
        },
        {
          name: 'AltAMSL',
          unit: 'm',
          description: 'altitude AMSL',
        },
        {
          name: 'Press',
          unit: 'Pa',
          description: 'measured atmospheric pressure',
        },
        {
          name: 'Temp',
          unit: 'degC',
          description: 'measured atmospheric temperature',
        },
        {
          name: 'CRt',
          unit: 'm/s',
          description: 'derived climb rate from primary barometer',
        },
        {
          name: 'SMS',
          unit: 'ms',
          description: 'time last sample was taken',
        },
        {
          name: 'Offset',
          unit: 'm',
          description:
            'raw adjustment of barometer altitude, zeroed on calibration, possibly set by GCS',
        },
        {
          name: 'GndTemp',
          unit: 'degC',
          description: 'temperature on ground, specified by parameter or measured while on ground',
        },
        {
          name: 'H',
          unit: 'true if barometer is considered healthy',
          description: '',
        },
        {
          name: 'CPress',
          unit: 'Pa',
          description: 'compensated atmospheric pressure',
        },
      ],
    ],
  },
  {
    id: 'bat',
    title: 'BAT',
    description: 'Gathered battery data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Inst',
          unit: 'instance',
          description: 'battery instance number',
        },
        {
          name: 'Volt',
          unit: 'V',
          description: 'measured voltage',
        },
        {
          name: 'VoltR',
          unit: 'V',
          description: 'estimated resting voltage',
        },
        {
          name: 'Curr',
          unit: 'A',
          description: 'measured current',
        },
        {
          name: 'CurrTot',
          unit: 'mAh',
          description: 'consumed Ah, current * time',
        },
        {
          name: 'EnrgTot',
          unit: 'W.h',
          description: 'consumed Wh, energy this battery has expended',
        },
        {
          name: 'Temp',
          unit: 'degC',
          description: 'measured temperature',
        },
        {
          name: 'Res',
          unit: 'Ohm',
          description: 'estimated battery resistance',
        },
        {
          name: 'RemPct',
          unit: '%',
          description: 'remaining percentage',
        },
        {
          name: 'H',
          unit: 'health',
          description: '',
        },
        {
          name: 'SH',
          unit: '%',
          description: 'state of health percentage. 0 if unknown',
        },
      ],
    ],
  },
  {
    id: 'bbx1',
    title: 'BBX1',
    description: 'BlackBox data1',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Lat',
          unit: 'deglatitude',
          description: 'Latitude',
        },
        {
          name: 'Lon',
          unit: 'deglongitude',
          description: 'Longitude',
        },
        {
          name: 'Alt',
          unit: 'm',
          description: 'altitude above sea level',
        },
        {
          name: 'Roll',
          unit: 'deg',
          description: 'euler roll',
        },
        {
          name: 'Pitch',
          unit: 'deg',
          description: 'euler pitch',
        },
        {
          name: 'Yaw',
          unit: 'deg',
          description: 'euler yaw',
        },
        {
          name: 'VN',
          unit: 'm/s',
          description: 'velocity north',
        },
        {
          name: 'VE',
          unit: 'm/s',
          description: 'velocity east',
        },
        {
          name: 'VD',
          unit: 'm/s',
          description: 'velocity down',
        },
      ],
    ],
  },
  {
    id: 'bbx2',
    title: 'BBX2',
    description: 'BlackBox data2',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'GyrX',
          unit: 'deg/s',
          description: 'X axis gyro',
        },
        {
          name: 'GyrY',
          unit: 'deg/s',
          description: 'Y axis gyro',
        },
        {
          name: 'GyrZ',
          unit: 'deg/s',
          description: 'Z axis gyro',
        },
        {
          name: 'AccX',
          unit: 'm/s/s',
          description: 'accel X axis (front)',
        },
        {
          name: 'AccY',
          unit: 'm/s/s',
          description: 'accel Y axis (right)',
        },
        {
          name: 'AccZ',
          unit: 'm/s/s',
          description: 'accel Z axis (down)',
        },
      ],
    ],
  },
  {
    id: 'bcl',
    title: 'BCL',
    description: 'Battery cell voltage information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Instance',
          unit: 'instance',
          description: 'battery instance number',
        },
        {
          name: 'Volt',
          unit: 'V',
          description: 'battery voltage',
        },
        {
          name: 'V1',
          unit: 'mV',
          description: 'first cell voltage',
        },
        {
          name: 'V2',
          unit: 'mV',
          description: 'second cell voltage',
        },
        {
          name: 'V3',
          unit: 'mV',
          description: 'third cell voltage',
        },
        {
          name: 'V4',
          unit: 'mV',
          description: 'fourth cell voltage',
        },
        {
          name: 'V5',
          unit: 'mV',
          description: 'fifth cell voltage',
        },
        {
          name: 'V6',
          unit: 'mV',
          description: 'sixth cell voltage',
        },
        {
          name: 'V7',
          unit: 'mV',
          description: 'seventh cell voltage',
        },
        {
          name: 'V8',
          unit: 'mV',
          description: 'eighth cell voltage',
        },
        {
          name: 'V9',
          unit: 'mV',
          description: 'ninth cell voltage',
        },
        {
          name: 'V10',
          unit: 'mV',
          description: 'tenth cell voltage',
        },
        {
          name: 'V11',
          unit: 'mV',
          description: 'eleventh cell voltage',
        },
        {
          name: 'V12',
          unit: 'mV',
          description: 'twelfth cell voltage',
        },
      ],
    ],
  },
  {
    id: 'bcl2',
    title: 'BCL2',
    description: 'Battery cell voltage information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Instance',
          unit: 'instance',
          description: 'battery instance number',
        },
        {
          name: 'V13',
          unit: 'mV',
          description: 'thirteenth cell voltage',
        },
        {
          name: 'V14',
          unit: 'mV',
          description: 'fourteenth cell voltage',
        },
      ],
    ],
  },
  {
    id: 'bcn',
    title: 'BCN',
    description: 'Beacon information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Health',
          unit: 'True if beacon sensor is healthy',
          description: '',
        },
        {
          name: 'Cnt',
          unit: 'Number of beacons being used',
          description: '',
        },
        {
          name: 'D0',
          unit: 'm',
          description: 'Distance to first beacon',
        },
        {
          name: 'D1',
          unit: 'm',
          description: 'Distance to second beacon',
        },
        {
          name: 'D2',
          unit: 'm',
          description: 'Distance to third beacon',
        },
        {
          name: 'D3',
          unit: 'm',
          description: 'Distance to fourth beacon',
        },
        {
          name: 'PosX',
          unit: 'm',
          description: 'Calculated beacon position, x-axis',
        },
        {
          name: 'PosY',
          unit: 'm',
          description: 'Calculated beacon position, y-axis',
        },
        {
          name: 'PosZ',
          unit: 'm',
          description: 'Calculated beacon position, z-axis',
        },
      ],
    ],
  },
  {
    id: 'cafd',
    title: 'CAFD',
    description: 'CANFD Frame',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Bus',
          unit: 'instance',
          description: 'bus number',
        },
        {
          name: 'Id',
          unit: 'frame identifier',
          description: '',
        },
        {
          name: 'DLC',
          unit: 'data length code',
          description: '',
        },
        {
          name: 'D0',
          unit: 'data 0',
          description: '',
        },
        {
          name: 'D1',
          unit: 'data 1',
          description: '',
        },
        {
          name: 'D2',
          unit: 'data 2',
          description: '',
        },
        {
          name: 'D3',
          unit: 'data 3',
          description: '',
        },
        {
          name: 'D4',
          unit: 'data 4',
          description: '',
        },
        {
          name: 'D5',
          unit: 'data 5',
          description: '',
        },
        {
          name: 'D6',
          unit: 'data 6',
          description: '',
        },
        {
          name: 'D7',
          unit: 'data 7',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'cam',
    title: 'CAM',
    description: 'Camera shutter information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'Instance number',
        },
        {
          name: 'Img',
          unit: 'Image number',
          description: '',
        },
        {
          name: 'GPSTime',
          unit: 'milliseconds since start of GPS week',
          description: '',
        },
        {
          name: 'GPSWeek',
          unit: 'weeks since 5 Jan 1980',
          description: '',
        },
        {
          name: 'Lat',
          unit: 'deglatitude',
          description: 'current latitude',
        },
        {
          name: 'Lng',
          unit: 'deglongitude',
          description: 'current longitude',
        },
        {
          name: 'Alt',
          unit: 'm',
          description: 'current altitude',
        },
        {
          name: 'RelAlt',
          unit: 'm',
          description: 'current altitude relative to home',
        },
        {
          name: 'GPSAlt',
          unit: 'm',
          description: 'altitude as reported by GPS',
        },
        {
          name: 'R',
          unit: 'deg',
          description: 'current vehicle roll',
        },
        {
          name: 'P',
          unit: 'deg',
          description: 'current vehicle pitch',
        },
        {
          name: 'Y',
          unit: 'deg',
          description: 'current vehicle yaw',
        },
      ],
    ],
  },
  {
    id: 'cand',
    title: 'CAND',
    description: 'Info from GetNodeInfo request',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Driver',
          unit: 'Driver index',
          description: '',
        },
        {
          name: 'NodeId',
          unit: 'instance',
          description: 'Node ID',
        },
        {
          name: 'UID1',
          unit: 'Hardware ID, part 1',
          description: '',
        },
        {
          name: 'UID2',
          unit: 'Hardware ID, part 2',
          description: '',
        },
        {
          name: 'Name',
          unit: 'char[64]',
          description: 'Name string',
        },
        {
          name: 'Major',
          unit: 'major revision id',
          description: '',
        },
        {
          name: 'Minor',
          unit: 'minor revision id',
          description: '',
        },
        {
          name: 'Version',
          unit: 'AP_Periph git hash',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'canf',
    title: 'CANF',
    description: 'CAN Frame',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Bus',
          unit: 'instance',
          description: 'bus number',
        },
        {
          name: 'Id',
          unit: 'frame identifier',
          description: '',
        },
        {
          name: 'DLC',
          unit: 'data length code',
          description: '',
        },
        {
          name: 'B0',
          unit: 'byte 0',
          description: '',
        },
        {
          name: 'B1',
          unit: 'byte 1',
          description: '',
        },
        {
          name: 'B2',
          unit: 'byte 2',
          description: '',
        },
        {
          name: 'B3',
          unit: 'byte 3',
          description: '',
        },
        {
          name: 'B4',
          unit: 'byte 4',
          description: '',
        },
        {
          name: 'B5',
          unit: 'byte 5',
          description: '',
        },
        {
          name: 'B6',
          unit: 'byte 6',
          description: '',
        },
        {
          name: 'B7',
          unit: 'byte 7',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'cans',
    title: 'CANS',
    description: 'CAN Bus Statistics',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'driver index',
        },
        {
          name: 'T',
          unit: 'transmit success count',
          description: '',
        },
        {
          name: 'Trq',
          unit: 'transmit request count',
          description: '',
        },
        {
          name: 'Trej',
          unit: 'transmit reject count',
          description: '',
        },
        {
          name: 'Tov',
          unit: 'transmit overflow count',
          description: '',
        },
        {
          name: 'Tto',
          unit: 'transmit timeout count',
          description: '',
        },
        {
          name: 'Tab',
          unit: 'transmit abort count',
          description: '',
        },
        {
          name: 'R',
          unit: 'receive count',
          description: '',
        },
        {
          name: 'Rov',
          unit: 'receive overflow count',
          description: '',
        },
        {
          name: 'Rer',
          unit: 'receive error count',
          description: '',
        },
        {
          name: 'Bo',
          unit: 'bus offset error count',
          description: '',
        },
        {
          name: 'Etx',
          unit: 'ESC successful send count',
          description: '',
        },
        {
          name: 'Stx',
          unit: 'Servo successful send count',
          description: '',
        },
        {
          name: 'Ftx',
          unit: 'ESC/Servo failed-to-send count',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'cc',
    title: 'CC',
    description: 'Custom Controller data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'Type',
          unit: 'enum',
          description: 'controller type Values:',
        },
        {
          name: 'CONT_NONE',
          unit: '0',
          description: '',
        },
        {
          name: 'CONT_EMPTY',
          unit: '1',
          description: '',
        },
        {
          name: 'CONT_PID',
          unit: '2',
          description: '',
        },
        {
          name: 'Act',
          unit: 'true if controller is active',
          description: '',
        },
      ],
      [
        {
          name: 'CONT_NONE',
          unit: '0',
          description: '',
        },
        {
          name: 'CONT_EMPTY',
          unit: '1',
          description: '',
        },
        {
          name: 'CONT_PID',
          unit: '2',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'cmd',
    title: 'CMD',
    description: 'Uploaded mission command information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'CTot',
          unit: 'Total number of mission commands',
          description: '',
        },
        {
          name: 'CNum',
          unit: 'This command’s offset in mission',
          description: '',
        },
        {
          name: 'CId',
          unit: 'Command type',
          description: '',
        },
        {
          name: 'Prm1',
          unit: 'Parameter 1',
          description: '',
        },
        {
          name: 'Prm2',
          unit: 'Parameter 2',
          description: '',
        },
        {
          name: 'Prm3',
          unit: 'Parameter 3',
          description: '',
        },
        {
          name: 'Prm4',
          unit: 'Parameter 4',
          description: '',
        },
        {
          name: 'Lat',
          unit: 'deglatitude',
          description: 'Command latitude',
        },
        {
          name: 'Lng',
          unit: 'deglongitude',
          description: 'Command longitude',
        },
        {
          name: 'Alt',
          unit: 'm',
          description: 'Command altitude',
        },
        {
          name: 'Frame',
          unit: 'Frame used for position',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'csrv',
    title: 'CSRV',
    description: 'Servo feedback data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Id',
          unit: 'instance',
          description: 'Servo number this data relates to',
        },
        {
          name: 'Pos',
          unit: 'deg',
          description: 'Current servo position',
        },
        {
          name: 'Force',
          unit: 'N.m',
          description: 'Force being applied',
        },
        {
          name: 'Speed',
          unit: 'deg/s',
          description: 'Current servo movement speed',
        },
        {
          name: 'Pow',
          unit: '%',
          description: 'Amount of rated power being applied',
        },
        {
          name: 'PosCmd',
          unit: 'deg',
          description: 'commanded servo position',
        },
        {
          name: 'V',
          unit: 'V',
          description: 'Voltage',
        },
        {
          name: 'A',
          unit: 'A',
          description: 'Current',
        },
        {
          name: 'MotT',
          unit: 'degC',
          description: 'motor temperature',
        },
        {
          name: 'PCBT',
          unit: 'degC',
          description: 'PCB temperature',
        },
        {
          name: 'Err',
          unit: 'error flags',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'ctun',
    title: 'CTUN',
    description: 'Control Tuning information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'NavRoll',
          unit: 'deg',
          description: 'desired roll',
        },
        {
          name: 'Roll',
          unit: 'deg',
          description: 'achieved roll',
        },
        {
          name: 'NavPitch',
          unit: 'deg',
          description: 'desired pitch assuming pitch trims are already applied',
        },
        {
          name: 'Pitch',
          unit: 'deg',
          description:
            'achieved pitch assuming pitch trims are already applied,ie “0deg” is level flight trimmed pitch attitude as shown on artificial horizon level line.',
        },
        {
          name: 'ThO',
          unit: 'scaled output throttle',
          description: '',
        },
        {
          name: 'RdO',
          unit: 'scaled output rudder',
          description: '',
        },
        {
          name: 'ThD',
          unit: 'demanded speed-height-controller throttle',
          description: '',
        },
        {
          name: 'As',
          unit: 'm/s',
          description:
            'airspeed estimate (or measurement if airspeed sensor healthy and ARSPD_USE>0)',
        },
        {
          name: 'AsT',
          unit: 'enum',
          description: 'airspeed type ( old estimate or source of new estimate) Values:',
        },
        {
          name: 'NO_NEW_ESTIMATE',
          unit: '0',
          description: '',
        },
        {
          name: 'AIRSPEED_SENSOR',
          unit: '1',
          description: '',
        },
        {
          name: 'DCM_SYNTHETIC',
          unit: '2',
          description: '',
        },
        {
          name: 'EKF3_SYNTHETIC',
          unit: '3',
          description: '',
        },
        {
          name: 'SIM',
          unit: '4',
          description: '',
        },
        {
          name: 'SAs',
          unit: 'm/s',
          description: 'DCM’s airspeed estimate, NaN if not available',
        },
        {
          name: 'E2T',
          unit: 'equivalent to true airspeed ratio',
          description: '',
        },
        {
          name: 'GU',
          unit: 'cm/s',
          description: 'groundspeed undershoot when flying with minimum groundspeed',
        },
      ],
      [
        {
          name: 'NO_NEW_ESTIMATE',
          unit: '0',
          description: '',
        },
        {
          name: 'AIRSPEED_SENSOR',
          unit: '1',
          description: '',
        },
        {
          name: 'DCM_SYNTHETIC',
          unit: '2',
          description: '',
        },
        {
          name: 'EKF3_SYNTHETIC',
          unit: '3',
          description: '',
        },
        {
          name: 'SIM',
          unit: '4',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'dcm',
    title: 'DCM',
    description: 'DCM Estimator Data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Roll',
          unit: 'deg',
          description: 'estimated roll',
        },
        {
          name: 'Pitch',
          unit: 'deg',
          description: 'estimated pitch',
        },
        {
          name: 'Yaw',
          unit: 'deg',
          description: 'estimated yaw',
        },
        {
          name: 'ErrRP',
          unit: 'deg',
          description: 'lowest estimated gyro drift error',
        },
        {
          name: 'ErrYaw',
          unit: 'degheading',
          description: 'difference between measured yaw and DCM yaw estimate',
        },
        {
          name: 'VWN',
          unit: 'm/s',
          description: 'wind velocity, to-the-North component',
        },
        {
          name: 'VWE',
          unit: 'm/s',
          description: 'wind velocity, to-the-East component',
        },
        {
          name: 'VWD',
          unit: 'm/s',
          description: 'wind velocity, Up-to-Down component',
        },
      ],
    ],
  },
  {
    id: 'dms',
    title: 'DMS',
    description: 'DataFlash-Over-MAVLink statistics',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'N',
          unit: 'Current block number',
          description: '',
        },
        {
          name: 'Dp',
          unit: 'Number of times we rejected a write to the backend',
          description: '',
        },
        {
          name: 'RT',
          unit: 'Number of blocks sent from the retry queue',
          description: '',
        },
        {
          name: 'RS',
          unit: 'Number of resends of unacknowledged data made',
          description: '',
        },
        {
          name: 'Fa',
          unit: 'Average number of blocks on the free list',
          description: '',
        },
        {
          name: 'Fmn',
          unit: 'Minimum number of blocks on the free list',
          description: '',
        },
        {
          name: 'Fmx',
          unit: 'Maximum number of blocks on the free list',
          description: '',
        },
        {
          name: 'Pa',
          unit: 'Average number of blocks on the pending list',
          description: '',
        },
        {
          name: 'Pmn',
          unit: 'Minimum number of blocks on the pending list',
          description: '',
        },
        {
          name: 'Pmx',
          unit: 'Maximum number of blocks on the pending list',
          description: '',
        },
        {
          name: 'Sa',
          unit: 'Average number of blocks on the sent list',
          description: '',
        },
        {
          name: 'Smn',
          unit: 'Minimum number of blocks on the sent list',
          description: '',
        },
        {
          name: 'Smx',
          unit: 'Maximum number of blocks on the sent list',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'dsf',
    title: 'DSF',
    description: 'Onboard logging statistics',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Dp',
          unit: 'Number of times we rejected a write to the backend',
          description: '',
        },
        {
          name: 'Blk',
          unit: 'Current block number',
          description: '',
        },
        {
          name: 'Bytes',
          unit: 'B',
          description: 'Current write offset',
        },
        {
          name: 'FMn',
          unit: 'Minimum free space in write buffer in last time period',
          description: '',
        },
        {
          name: 'FMx',
          unit: 'Maximum free space in write buffer in last time period',
          description: '',
        },
        {
          name: 'FAv',
          unit: 'Average free space in write buffer in last time period',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'dstl',
    title: 'DSTL',
    description: 'Deepstall Landing data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Stg',
          unit: 'Deepstall landing stage',
          description: '',
        },
        {
          name: 'THdg',
          unit: 'degheading',
          description: 'Target heading',
        },
        {
          name: 'Lat',
          unit: 'deglatitude',
          description: 'Landing point latitude',
        },
        {
          name: 'Lng',
          unit: 'deglongitude',
          description: 'Landing point longitude',
        },
        {
          name: 'Alt',
          unit: 'm',
          description: 'Landing point altitude',
        },
        {
          name: 'XT',
          unit: 'Crosstrack error',
          description: '',
        },
        {
          name: 'Travel',
          unit: 'Expected travel distance vehicle will travel from this point',
          description: '',
        },
        {
          name: 'L1I',
          unit: 'L1 controller crosstrack integrator value',
          description: '',
        },
        {
          name: 'Loiter',
          unit: 'wind estimate loiter angle flown',
          description: '',
        },
        {
          name: 'Des',
          unit: 'Deepstall steering PID desired value',
          description: '',
        },
        {
          name: 'P',
          unit: 'Deepstall steering PID Proportional response component',
          description: '',
        },
        {
          name: 'I',
          unit: 'Deepstall steering PID Integral response component',
          description: '',
        },
        {
          name: 'D',
          unit: 'Deepstall steering PID Derivative response component',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'eahr',
    title: 'EAHR',
    description: 'External AHRS data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Roll',
          unit: 'deg',
          description: 'euler roll',
        },
        {
          name: 'Pitch',
          unit: 'deg',
          description: 'euler pitch',
        },
        {
          name: 'Yaw',
          unit: 'deg',
          description: 'euler yaw',
        },
        {
          name: 'VN',
          unit: 'm/s',
          description: 'velocity north',
        },
        {
          name: 'VE',
          unit: 'm/s',
          description: 'velocity east',
        },
        {
          name: 'VD',
          unit: 'm/s',
          description: 'velocity down',
        },
        {
          name: 'Lat',
          unit: 'deglatitude',
          description: 'latitude',
        },
        {
          name: 'Lon',
          unit: 'deglongitude',
          description: 'longitude',
        },
        {
          name: 'Alt',
          unit: 'm',
          description: 'altitude AMSL',
        },
        {
          name: 'Flg',
          unit: 'nav status flags',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'eahv',
    title: 'EAHV',
    description: 'External AHRS variances',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'Vel',
          unit: 'velocity variance',
          description: '',
        },
        {
          name: 'Pos',
          unit: 'position variance',
          description: '',
        },
        {
          name: 'Hgt',
          unit: 'height variance',
          description: '',
        },
        {
          name: 'MagX',
          unit: 'magnetic variance, X',
          description: '',
        },
        {
          name: 'MagY',
          unit: 'magnetic variance, Y',
          description: '',
        },
        {
          name: 'MagZ',
          unit: 'magnetic variance, Z',
          description: '',
        },
        {
          name: 'TAS',
          unit: 'true airspeed variance',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'ecyl',
    title: 'ECYL',
    description: 'EFI per-cylinder information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Inst',
          unit: 'instance',
          description: 'Cylinder this data belongs to',
        },
        {
          name: 'IgnT',
          unit: 'deg',
          description: 'Ignition timing',
        },
        {
          name: 'InjT',
          unit: 'ms',
          description: 'Injection time',
        },
        {
          name: 'CHT',
          unit: 'degC',
          description: 'Cylinder head temperature',
        },
        {
          name: 'EGT',
          unit: 'degC',
          description: 'Exhaust gas temperature',
        },
        {
          name: 'Lambda',
          unit: 'Estimated lambda coefficient (dimensionless ratio)',
          description: '',
        },
        {
          name: 'CHT2',
          unit: 'degC',
          description: 'Cylinder2 head temperature',
        },
        {
          name: 'EGT2',
          unit: 'degC',
          description: 'Cylinder2 Exhaust gas temperature',
        },
        {
          name: 'IDX',
          unit: 'Index of the publishing ECU',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'edt2',
    title: 'EDT2',
    description: 'Status received from ESCs via Extended DShot telemetry v2',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'microseconds since system startup',
        },
        {
          name: 'Instance',
          unit: 'instance',
          description: 'ESC instance number',
        },
        {
          name: 'Stress',
          unit: 'current stress level (commutation effort), scaled to [0..255]',
          description: '',
        },
        {
          name: 'MaxStress',
          unit: 'maximum stress level (commutation effort) since arming, scaled to [0..15]',
          description: '',
        },
        {
          name: 'Status',
          unit: 'bitmask',
          description: 'status bits Bitmask values:',
        },
        {
          name: 'HAS_STRESS_DATA',
          unit: '1',
          description: 'true if the message contains up-to-date stress data',
        },
        {
          name: 'HAS_STATUS_DATA',
          unit: '2',
          description: 'true if the message contains up-to-date status data',
        },
        {
          name: 'ALERT_BIT',
          unit: '4',
          description: 'true if the last status had the “alert” bit (e.g. demag)',
        },
        {
          name: 'WARNING_BIT',
          unit: '8',
          description: 'true if the last status had the “warning” bit (e.g. desync)',
        },
        {
          name: 'ERROR_BIT',
          unit: '16',
          description: 'true if the last status had the “error” bit (e.g. stall)',
        },
      ],
      [
        {
          name: 'HAS_STRESS_DATA',
          unit: '1',
          description: 'true if the message contains up-to-date stress data',
        },
        {
          name: 'HAS_STATUS_DATA',
          unit: '2',
          description: 'true if the message contains up-to-date status data',
        },
        {
          name: 'ALERT_BIT',
          unit: '4',
          description: 'true if the last status had the “alert” bit (e.g. demag)',
        },
        {
          name: 'WARNING_BIT',
          unit: '8',
          description: 'true if the last status had the “warning” bit (e.g. desync)',
        },
        {
          name: 'ERROR_BIT',
          unit: '16',
          description: 'true if the last status had the “error” bit (e.g. stall)',
        },
      ],
    ],
  },
  {
    id: 'efi',
    title: 'EFI',
    description: 'Electronic Fuel Injection system data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'LP',
          unit: '%',
          description: 'Reported engine load',
        },
        {
          name: 'Rpm',
          unit: 'rpm',
          description: 'Reported engine RPM',
        },
        {
          name: 'SDT',
          unit: 'ms',
          description: 'Spark Dwell Time',
        },
        {
          name: 'ATM',
          unit: 'Pa',
          description: 'Atmospheric pressure',
        },
        {
          name: 'IMP',
          unit: 'Pa',
          description: 'Intake manifold pressure',
        },
        {
          name: 'IMT',
          unit: 'degC',
          description: 'Intake manifold temperature',
        },
        {
          name: 'ECT',
          unit: 'degC',
          description: 'Engine Coolant Temperature',
        },
        {
          name: 'OilP',
          unit: 'Pa',
          description: 'Oil Pressure',
        },
        {
          name: 'OilT',
          unit: 'degC',
          description: 'Oil temperature',
        },
        {
          name: 'FP',
          unit: 'Pa',
          description: 'Fuel Pressure',
        },
        {
          name: 'FCR',
          unit: 'Fuel Consumption Rate',
          description: '',
        },
        {
          name: 'CFV',
          unit: 'Consumed fuel volume',
          description: '',
        },
        {
          name: 'TPS',
          unit: '%',
          description: 'Throttle Position',
        },
        {
          name: 'IDX',
          unit: 'Index of the publishing ECU',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'efi2',
    title: 'EFI2',
    description: 'Electronic Fuel Injection system data - redux',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Healthy',
          unit: 'True if EFI is healthy',
          description: '',
        },
        {
          name: 'ES',
          unit: 'Engine state',
          description: '',
        },
        {
          name: 'GE',
          unit: 'General error',
          description: '',
        },
        {
          name: 'CSE',
          unit: 'Crankshaft sensor status',
          description: '',
        },
        {
          name: 'TS',
          unit: 'Temperature status',
          description: '',
        },
        {
          name: 'FPS',
          unit: 'Fuel pressure status',
          description: '',
        },
        {
          name: 'OPS',
          unit: 'Oil pressure status',
          description: '',
        },
        {
          name: 'DS',
          unit: 'Detonation status',
          description: '',
        },
        {
          name: 'MS',
          unit: 'Misfire status',
          description: '',
        },
        {
          name: 'DebS',
          unit: 'Debris status',
          description: '',
        },
        {
          name: 'SPU',
          unit: 'Spark plug usage',
          description: '',
        },
        {
          name: 'IDX',
          unit: 'Index of the publishing ECU',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'efis',
    title: 'EFIS',
    description: 'Electronic Fuel Injection data - Hirth specific Status information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'EET',
          unit: 'bitmask',
          description: 'Error Excess Temperature Bitfield Bitmask values:',
        },
        {
          name: 'CHT_1_LOW',
          unit: '1',
          description: 'true if CHT1 is too low',
        },
        {
          name: 'CHT_1_HIGH',
          unit: '2',
          description: 'true if CHT1 is too high',
        },
        {
          name: 'CHT_1_ENRC_ACTIVE',
          unit: '4',
          description: 'true if CHT1 Enrichment is active',
        },
        {
          name: 'CHT_2_LOW',
          unit: '8',
          description: 'true if CHT2 is too low',
        },
        {
          name: 'CHT_2_HIGH',
          unit: '16',
          description: 'true if CHT2 is too high',
        },
        {
          name: 'CHT_2_ENRC_ACTIVE',
          unit: '32',
          description: 'true if CHT2 Enrichment is active',
        },
        {
          name: 'EGT_1_LOW',
          unit: '64',
          description: 'true if EGT1 is too low',
        },
        {
          name: 'EGT_1_HIGH',
          unit: '128',
          description: 'true if EGT1 is too high',
        },
        {
          name: 'EGT_1_ENRC_ACTIVE',
          unit: '256',
          description: 'true if EGT1 Enrichment is active',
        },
        {
          name: 'EGT_2_LOW',
          unit: '512',
          description: 'true if EGT2 is too low',
        },
        {
          name: 'EGT_2_HIGH',
          unit: '1024',
          description: 'true if EGT2 is too high',
        },
        {
          name: 'EGT_2_ENRC_ACTIVE',
          unit: '2048',
          description: 'true if EGT2 Enrichment is active',
        },
        {
          name: 'FLAG',
          unit: 'bitmask',
          description: 'Sensor Status Bitfield Bitmask values:',
        },
        {
          name: 'ENGINE_TEMP_SENSOR_OK',
          unit: '2',
          description: 'true if engine temperature sensor is OK',
        },
        {
          name: 'AIR_TEMP_SENSOR_OK',
          unit: '4',
          description: 'true if air temperature sensor is OK',
        },
        {
          name: 'AIR_PRESSURE_SENSOR_OK',
          unit: '8',
          description: 'true if intake air pressure sensor is OK',
        },
        {
          name: 'THROTTLE_SENSOR_OK',
          unit: '16',
          description: 'true if throttle sensor is OK',
        },
        {
          name: 'CRF',
          unit: 'CRC failure count',
          description: '',
        },
        {
          name: 'AKF',
          unit: 'ACK failure count',
          description: '',
        },
        {
          name: 'Up',
          unit: 'Uptime between 2 messages',
          description: '',
        },
        {
          name: 'ThO',
          unit: 'Throttle output as received by the engine',
          description: '',
        },
        {
          name: 'ThM',
          unit: 'Modified throttle_to_hirth output sent to the engine',
          description: '',
        },
      ],
      [
        {
          name: 'CHT_1_LOW',
          unit: '1',
          description: 'true if CHT1 is too low',
        },
        {
          name: 'CHT_1_HIGH',
          unit: '2',
          description: 'true if CHT1 is too high',
        },
        {
          name: 'CHT_1_ENRC_ACTIVE',
          unit: '4',
          description: 'true if CHT1 Enrichment is active',
        },
        {
          name: 'CHT_2_LOW',
          unit: '8',
          description: 'true if CHT2 is too low',
        },
        {
          name: 'CHT_2_HIGH',
          unit: '16',
          description: 'true if CHT2 is too high',
        },
        {
          name: 'CHT_2_ENRC_ACTIVE',
          unit: '32',
          description: 'true if CHT2 Enrichment is active',
        },
        {
          name: 'EGT_1_LOW',
          unit: '64',
          description: 'true if EGT1 is too low',
        },
        {
          name: 'EGT_1_HIGH',
          unit: '128',
          description: 'true if EGT1 is too high',
        },
        {
          name: 'EGT_1_ENRC_ACTIVE',
          unit: '256',
          description: 'true if EGT1 Enrichment is active',
        },
        {
          name: 'EGT_2_LOW',
          unit: '512',
          description: 'true if EGT2 is too low',
        },
        {
          name: 'EGT_2_HIGH',
          unit: '1024',
          description: 'true if EGT2 is too high',
        },
        {
          name: 'EGT_2_ENRC_ACTIVE',
          unit: '2048',
          description: 'true if EGT2 Enrichment is active',
        },
      ],
      [
        {
          name: 'ENGINE_TEMP_SENSOR_OK',
          unit: '2',
          description: 'true if engine temperature sensor is OK',
        },
        {
          name: 'AIR_TEMP_SENSOR_OK',
          unit: '4',
          description: 'true if air temperature sensor is OK',
        },
        {
          name: 'AIR_PRESSURE_SENSOR_OK',
          unit: '8',
          description: 'true if intake air pressure sensor is OK',
        },
        {
          name: 'THROTTLE_SENSOR_OK',
          unit: '16',
          description: 'true if throttle sensor is OK',
        },
      ],
    ],
  },
  {
    id: 'err',
    title: 'ERR',
    description: 'Specifically coded error messages',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Subsys',
          unit: 'enum',
          description: 'Subsystem in which the error occurred Values:',
        },
        {
          name: 'MAIN',
          unit: '1',
          description: '',
        },
        {
          name: 'RADIO',
          unit: '2',
          description: '',
        },
        {
          name: 'COMPASS',
          unit: '3',
          description: '',
        },
        {
          name: 'OPTFLOW',
          unit: '4',
          description: 'not used',
        },
        {
          name: 'FAILSAFE_RADIO',
          unit: '5',
          description: '',
        },
        {
          name: 'FAILSAFE_BATT',
          unit: '6',
          description: '',
        },
        {
          name: 'FAILSAFE_GPS',
          unit: '7',
          description: 'not used',
        },
        {
          name: 'FAILSAFE_GCS',
          unit: '8',
          description: '',
        },
        {
          name: 'FAILSAFE_FENCE',
          unit: '9',
          description: '',
        },
        {
          name: 'FLIGHT_MODE',
          unit: '10',
          description: '',
        },
        {
          name: 'GPS',
          unit: '11',
          description: '',
        },
        {
          name: 'CRASH_CHECK',
          unit: '12',
          description: '',
        },
        {
          name: 'FLIP',
          unit: '13',
          description: '',
        },
        {
          name: 'AUTOTUNE',
          unit: '14',
          description: 'not used',
        },
        {
          name: 'PARACHUTES',
          unit: '15',
          description: '',
        },
        {
          name: 'EKFCHECK',
          unit: '16',
          description: '',
        },
        {
          name: 'FAILSAFE_EKFINAV',
          unit: '17',
          description: '',
        },
        {
          name: 'BARO',
          unit: '18',
          description: '',
        },
        {
          name: 'CPU',
          unit: '19',
          description: '',
        },
        {
          name: 'FAILSAFE_ADSB',
          unit: '20',
          description: '',
        },
        {
          name: 'TERRAIN',
          unit: '21',
          description: '',
        },
        {
          name: 'NAVIGATION',
          unit: '22',
          description: '',
        },
        {
          name: 'FAILSAFE_TERRAIN',
          unit: '23',
          description: '',
        },
        {
          name: 'EKF_PRIMARY',
          unit: '24',
          description: '',
        },
        {
          name: 'THRUST_LOSS_CHECK',
          unit: '25',
          description: '',
        },
        {
          name: 'FAILSAFE_SENSORS',
          unit: '26',
          description: '',
        },
        {
          name: 'FAILSAFE_LEAK',
          unit: '27',
          description: '',
        },
        {
          name: 'PILOT_INPUT',
          unit: '28',
          description: '',
        },
        {
          name: 'FAILSAFE_VIBE',
          unit: '29',
          description: '',
        },
        {
          name: 'INTERNAL_ERROR',
          unit: '30',
          description: '',
        },
        {
          name: 'FAILSAFE_DEADRECKON',
          unit: '31',
          description: '',
        },
        {
          name: 'ECode',
          unit: 'Subsystem-specific error code',
          description: '',
        },
      ],
      [
        {
          name: 'MAIN',
          unit: '1',
          description: '',
        },
        {
          name: 'RADIO',
          unit: '2',
          description: '',
        },
        {
          name: 'COMPASS',
          unit: '3',
          description: '',
        },
        {
          name: 'OPTFLOW',
          unit: '4',
          description: 'not used',
        },
        {
          name: 'FAILSAFE_RADIO',
          unit: '5',
          description: '',
        },
        {
          name: 'FAILSAFE_BATT',
          unit: '6',
          description: '',
        },
        {
          name: 'FAILSAFE_GPS',
          unit: '7',
          description: 'not used',
        },
        {
          name: 'FAILSAFE_GCS',
          unit: '8',
          description: '',
        },
        {
          name: 'FAILSAFE_FENCE',
          unit: '9',
          description: '',
        },
        {
          name: 'FLIGHT_MODE',
          unit: '10',
          description: '',
        },
        {
          name: 'GPS',
          unit: '11',
          description: '',
        },
        {
          name: 'CRASH_CHECK',
          unit: '12',
          description: '',
        },
        {
          name: 'FLIP',
          unit: '13',
          description: '',
        },
        {
          name: 'AUTOTUNE',
          unit: '14',
          description: 'not used',
        },
        {
          name: 'PARACHUTES',
          unit: '15',
          description: '',
        },
        {
          name: 'EKFCHECK',
          unit: '16',
          description: '',
        },
        {
          name: 'FAILSAFE_EKFINAV',
          unit: '17',
          description: '',
        },
        {
          name: 'BARO',
          unit: '18',
          description: '',
        },
        {
          name: 'CPU',
          unit: '19',
          description: '',
        },
        {
          name: 'FAILSAFE_ADSB',
          unit: '20',
          description: '',
        },
        {
          name: 'TERRAIN',
          unit: '21',
          description: '',
        },
        {
          name: 'NAVIGATION',
          unit: '22',
          description: '',
        },
        {
          name: 'FAILSAFE_TERRAIN',
          unit: '23',
          description: '',
        },
        {
          name: 'EKF_PRIMARY',
          unit: '24',
          description: '',
        },
        {
          name: 'THRUST_LOSS_CHECK',
          unit: '25',
          description: '',
        },
        {
          name: 'FAILSAFE_SENSORS',
          unit: '26',
          description: '',
        },
        {
          name: 'FAILSAFE_LEAK',
          unit: '27',
          description: '',
        },
        {
          name: 'PILOT_INPUT',
          unit: '28',
          description: '',
        },
        {
          name: 'FAILSAFE_VIBE',
          unit: '29',
          description: '',
        },
        {
          name: 'INTERNAL_ERROR',
          unit: '30',
          description: '',
        },
        {
          name: 'FAILSAFE_DEADRECKON',
          unit: '31',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'esc',
    title: 'ESC',
    description: 'Feedback received from ESCs',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'microseconds since system startup',
        },
        {
          name: 'Instance',
          unit: 'instance',
          description: 'ESC instance number',
        },
        {
          name: 'RPM',
          unit: 'rpm',
          description: 'reported motor rotation rate',
        },
        {
          name: 'RawRPM',
          unit: 'rpm',
          description: 'reported motor rotation rate without slew adjustment',
        },
        {
          name: 'Volt',
          unit: 'V',
          description: 'Perceived input voltage for the ESC',
        },
        {
          name: 'Curr',
          unit: 'A',
          description: 'Perceived current through the ESC',
        },
        {
          name: 'Temp',
          unit: 'degC',
          description: 'ESC temperature in centi-degrees C',
        },
        {
          name: 'CTot',
          unit: 'mAh',
          description: 'current consumed total mAh',
        },
        {
          name: 'MotTemp',
          unit: 'degC',
          description: 'measured motor temperature in centi-degrees C',
        },
        {
          name: 'Err',
          unit: '%',
          description: 'error rate',
        },
      ],
    ],
  },
  {
    id: 'escx',
    title: 'ESCX',
    description: 'ESC extended telemetry data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Instance',
          unit: 'instance',
          description: 'starts from 0',
        },
        {
          name: 'inpct',
          unit: '%',
          description: 'input duty cycle in percent',
        },
        {
          name: 'outpct',
          unit: '%',
          description: 'output duty cycle in percent',
        },
        {
          name: 'flags',
          unit: 'manufacturer-specific status flags',
          description: '',
        },
        {
          name: 'Pwr',
          unit: '%',
          description: 'Power percentage',
        },
      ],
    ],
  },
  {
    id: 'ev',
    title: 'EV',
    description: 'Specifically coded event messages',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Id',
          unit: 'enum',
          description: 'Event identifier Values:',
        },
        {
          name: 'ARMED',
          unit: '10',
          description: '',
        },
        {
          name: 'DISARMED',
          unit: '11',
          description: '',
        },
        {
          name: 'AUTO_ARMED',
          unit: '15',
          description: '',
        },
        {
          name: 'LAND_COMPLETE_MAYBE',
          unit: '17',
          description: '',
        },
        {
          name: 'LAND_COMPLETE',
          unit: '18',
          description: '',
        },
        {
          name: 'LOST_GPS',
          unit: '19',
          description: '',
        },
        {
          name: 'FLIP_START',
          unit: '21',
          description: '',
        },
        {
          name: 'FLIP_END',
          unit: '22',
          description: '',
        },
        {
          name: 'SET_HOME',
          unit: '25',
          description: '',
        },
        {
          name: 'SET_SIMPLE_ON',
          unit: '26',
          description: '',
        },
        {
          name: 'SET_SIMPLE_OFF',
          unit: '27',
          description: '',
        },
        {
          name: 'NOT_LANDED',
          unit: '28',
          description: '',
        },
        {
          name: 'SET_SUPERSIMPLE_ON',
          unit: '29',
          description: '',
        },
        {
          name: 'AUTOTUNE_INITIALISED',
          unit: '30',
          description: '',
        },
        {
          name: 'AUTOTUNE_OFF',
          unit: '31',
          description: '',
        },
        {
          name: 'AUTOTUNE_RESTART',
          unit: '32',
          description: '',
        },
        {
          name: 'AUTOTUNE_SUCCESS',
          unit: '33',
          description: '',
        },
        {
          name: 'AUTOTUNE_FAILED',
          unit: '34',
          description: '',
        },
        {
          name: 'AUTOTUNE_REACHED_LIMIT',
          unit: '35',
          description: '',
        },
        {
          name: 'AUTOTUNE_PILOT_TESTING',
          unit: '36',
          description: '',
        },
        {
          name: 'AUTOTUNE_SAVEDGAINS',
          unit: '37',
          description: '',
        },
        {
          name: 'SAVE_TRIM',
          unit: '38',
          description: '',
        },
        {
          name: 'SAVEWP_ADD_WP',
          unit: '39',
          description: '',
        },
        {
          name: 'FENCE_ENABLE',
          unit: '41',
          description: '',
        },
        {
          name: 'FENCE_DISABLE',
          unit: '42',
          description: '',
        },
        {
          name: 'ACRO_TRAINER_OFF',
          unit: '43',
          description: '',
        },
        {
          name: 'ACRO_TRAINER_LEVELING',
          unit: '44',
          description: '',
        },
        {
          name: 'ACRO_TRAINER_LIMITED',
          unit: '45',
          description: '',
        },
        {
          name: 'GRIPPER_GRAB',
          unit: '46',
          description: '',
        },
        {
          name: 'GRIPPER_RELEASE',
          unit: '47',
          description: '',
        },
        {
          name: 'PARACHUTE_DISABLED',
          unit: '49',
          description: '',
        },
        {
          name: 'PARACHUTE_ENABLED',
          unit: '50',
          description: '',
        },
        {
          name: 'PARACHUTE_RELEASED',
          unit: '51',
          description: '',
        },
        {
          name: 'LANDING_GEAR_DEPLOYED',
          unit: '52',
          description: '',
        },
        {
          name: 'LANDING_GEAR_RETRACTED',
          unit: '53',
          description: '',
        },
        {
          name: 'MOTORS_EMERGENCY_STOPPED',
          unit: '54',
          description: '',
        },
        {
          name: 'MOTORS_EMERGENCY_STOP_CLEARED',
          unit: '55',
          description: '',
        },
        {
          name: 'MOTORS_INTERLOCK_DISABLED',
          unit: '56',
          description: '',
        },
        {
          name: 'MOTORS_INTERLOCK_ENABLED',
          unit: '57',
          description: '',
        },
        {
          name: 'ROTOR_RUNUP_COMPLETE',
          unit: '58',
          description: 'Heli only',
        },
        {
          name: 'ROTOR_SPEED_BELOW_CRITICAL',
          unit: '59',
          description: 'Heli only',
        },
        {
          name: 'EKF_ALT_RESET',
          unit: '60',
          description: '',
        },
        {
          name: 'LAND_CANCELLED_BY_PILOT',
          unit: '61',
          description: '',
        },
        {
          name: 'EKF_YAW_RESET',
          unit: '62',
          description: '',
        },
        {
          name: 'AVOIDANCE_ADSB_ENABLE',
          unit: '63',
          description: '',
        },
        {
          name: 'AVOIDANCE_ADSB_DISABLE',
          unit: '64',
          description: '',
        },
        {
          name: 'AVOIDANCE_PROXIMITY_ENABLE',
          unit: '65',
          description: '',
        },
        {
          name: 'AVOIDANCE_PROXIMITY_DISABLE',
          unit: '66',
          description: '',
        },
        {
          name: 'GPS_PRIMARY_CHANGED',
          unit: '67',
          description: '',
        },
        {
          name: 'ZIGZAG_STORE_A',
          unit: '71',
          description: '',
        },
        {
          name: 'ZIGZAG_STORE_B',
          unit: '72',
          description: '',
        },
        {
          name: 'LAND_REPO_ACTIVE',
          unit: '73',
          description: '',
        },
        {
          name: 'STANDBY_ENABLE',
          unit: '74',
          description: '',
        },
        {
          name: 'STANDBY_DISABLE',
          unit: '75',
          description: '',
        },
        {
          name: 'FENCE_ALT_MAX_ENABLE',
          unit: '76',
          description: '',
        },
        {
          name: 'FENCE_ALT_MAX_DISABLE',
          unit: '77',
          description: '',
        },
        {
          name: 'FENCE_CIRCLE_ENABLE',
          unit: '78',
          description: '',
        },
        {
          name: 'FENCE_CIRCLE_DISABLE',
          unit: '79',
          description: '',
        },
        {
          name: 'FENCE_ALT_MIN_ENABLE',
          unit: '80',
          description: '',
        },
        {
          name: 'FENCE_ALT_MIN_DISABLE',
          unit: '81',
          description: '',
        },
        {
          name: 'FENCE_POLYGON_ENABLE',
          unit: '82',
          description: '',
        },
        {
          name: 'FENCE_POLYGON_DISABLE',
          unit: '83',
          description: '',
        },
        {
          name: 'EK3_SOURCES_SET_TO_PRIMARY',
          unit: '85',
          description: '',
        },
        {
          name: 'EK3_SOURCES_SET_TO_SECONDARY',
          unit: '86',
          description: '',
        },
        {
          name: 'EK3_SOURCES_SET_TO_TERTIARY',
          unit: '87',
          description: '',
        },
        {
          name: 'AIRSPEED_PRIMARY_CHANGED',
          unit: '90',
          description: '',
        },
        {
          name: 'SURFACED',
          unit: '163',
          description: '',
        },
        {
          name: 'NOT_SURFACED',
          unit: '164',
          description: '',
        },
        {
          name: 'BOTTOMED',
          unit: '165',
          description: '',
        },
        {
          name: 'NOT_BOTTOMED',
          unit: '166',
          description: '',
        },
      ],
      [
        {
          name: 'ARMED',
          unit: '10',
          description: '',
        },
        {
          name: 'DISARMED',
          unit: '11',
          description: '',
        },
        {
          name: 'AUTO_ARMED',
          unit: '15',
          description: '',
        },
        {
          name: 'LAND_COMPLETE_MAYBE',
          unit: '17',
          description: '',
        },
        {
          name: 'LAND_COMPLETE',
          unit: '18',
          description: '',
        },
        {
          name: 'LOST_GPS',
          unit: '19',
          description: '',
        },
        {
          name: 'FLIP_START',
          unit: '21',
          description: '',
        },
        {
          name: 'FLIP_END',
          unit: '22',
          description: '',
        },
        {
          name: 'SET_HOME',
          unit: '25',
          description: '',
        },
        {
          name: 'SET_SIMPLE_ON',
          unit: '26',
          description: '',
        },
        {
          name: 'SET_SIMPLE_OFF',
          unit: '27',
          description: '',
        },
        {
          name: 'NOT_LANDED',
          unit: '28',
          description: '',
        },
        {
          name: 'SET_SUPERSIMPLE_ON',
          unit: '29',
          description: '',
        },
        {
          name: 'AUTOTUNE_INITIALISED',
          unit: '30',
          description: '',
        },
        {
          name: 'AUTOTUNE_OFF',
          unit: '31',
          description: '',
        },
        {
          name: 'AUTOTUNE_RESTART',
          unit: '32',
          description: '',
        },
        {
          name: 'AUTOTUNE_SUCCESS',
          unit: '33',
          description: '',
        },
        {
          name: 'AUTOTUNE_FAILED',
          unit: '34',
          description: '',
        },
        {
          name: 'AUTOTUNE_REACHED_LIMIT',
          unit: '35',
          description: '',
        },
        {
          name: 'AUTOTUNE_PILOT_TESTING',
          unit: '36',
          description: '',
        },
        {
          name: 'AUTOTUNE_SAVEDGAINS',
          unit: '37',
          description: '',
        },
        {
          name: 'SAVE_TRIM',
          unit: '38',
          description: '',
        },
        {
          name: 'SAVEWP_ADD_WP',
          unit: '39',
          description: '',
        },
        {
          name: 'FENCE_ENABLE',
          unit: '41',
          description: '',
        },
        {
          name: 'FENCE_DISABLE',
          unit: '42',
          description: '',
        },
        {
          name: 'ACRO_TRAINER_OFF',
          unit: '43',
          description: '',
        },
        {
          name: 'ACRO_TRAINER_LEVELING',
          unit: '44',
          description: '',
        },
        {
          name: 'ACRO_TRAINER_LIMITED',
          unit: '45',
          description: '',
        },
        {
          name: 'GRIPPER_GRAB',
          unit: '46',
          description: '',
        },
        {
          name: 'GRIPPER_RELEASE',
          unit: '47',
          description: '',
        },
        {
          name: 'PARACHUTE_DISABLED',
          unit: '49',
          description: '',
        },
        {
          name: 'PARACHUTE_ENABLED',
          unit: '50',
          description: '',
        },
        {
          name: 'PARACHUTE_RELEASED',
          unit: '51',
          description: '',
        },
        {
          name: 'LANDING_GEAR_DEPLOYED',
          unit: '52',
          description: '',
        },
        {
          name: 'LANDING_GEAR_RETRACTED',
          unit: '53',
          description: '',
        },
        {
          name: 'MOTORS_EMERGENCY_STOPPED',
          unit: '54',
          description: '',
        },
        {
          name: 'MOTORS_EMERGENCY_STOP_CLEARED',
          unit: '55',
          description: '',
        },
        {
          name: 'MOTORS_INTERLOCK_DISABLED',
          unit: '56',
          description: '',
        },
        {
          name: 'MOTORS_INTERLOCK_ENABLED',
          unit: '57',
          description: '',
        },
        {
          name: 'ROTOR_RUNUP_COMPLETE',
          unit: '58',
          description: 'Heli only',
        },
        {
          name: 'ROTOR_SPEED_BELOW_CRITICAL',
          unit: '59',
          description: 'Heli only',
        },
        {
          name: 'EKF_ALT_RESET',
          unit: '60',
          description: '',
        },
        {
          name: 'LAND_CANCELLED_BY_PILOT',
          unit: '61',
          description: '',
        },
        {
          name: 'EKF_YAW_RESET',
          unit: '62',
          description: '',
        },
        {
          name: 'AVOIDANCE_ADSB_ENABLE',
          unit: '63',
          description: '',
        },
        {
          name: 'AVOIDANCE_ADSB_DISABLE',
          unit: '64',
          description: '',
        },
        {
          name: 'AVOIDANCE_PROXIMITY_ENABLE',
          unit: '65',
          description: '',
        },
        {
          name: 'AVOIDANCE_PROXIMITY_DISABLE',
          unit: '66',
          description: '',
        },
        {
          name: 'GPS_PRIMARY_CHANGED',
          unit: '67',
          description: '',
        },
        {
          name: 'ZIGZAG_STORE_A',
          unit: '71',
          description: '',
        },
        {
          name: 'ZIGZAG_STORE_B',
          unit: '72',
          description: '',
        },
        {
          name: 'LAND_REPO_ACTIVE',
          unit: '73',
          description: '',
        },
        {
          name: 'STANDBY_ENABLE',
          unit: '74',
          description: '',
        },
        {
          name: 'STANDBY_DISABLE',
          unit: '75',
          description: '',
        },
        {
          name: 'FENCE_ALT_MAX_ENABLE',
          unit: '76',
          description: '',
        },
        {
          name: 'FENCE_ALT_MAX_DISABLE',
          unit: '77',
          description: '',
        },
        {
          name: 'FENCE_CIRCLE_ENABLE',
          unit: '78',
          description: '',
        },
        {
          name: 'FENCE_CIRCLE_DISABLE',
          unit: '79',
          description: '',
        },
        {
          name: 'FENCE_ALT_MIN_ENABLE',
          unit: '80',
          description: '',
        },
        {
          name: 'FENCE_ALT_MIN_DISABLE',
          unit: '81',
          description: '',
        },
        {
          name: 'FENCE_POLYGON_ENABLE',
          unit: '82',
          description: '',
        },
        {
          name: 'FENCE_POLYGON_DISABLE',
          unit: '83',
          description: '',
        },
        {
          name: 'EK3_SOURCES_SET_TO_PRIMARY',
          unit: '85',
          description: '',
        },
        {
          name: 'EK3_SOURCES_SET_TO_SECONDARY',
          unit: '86',
          description: '',
        },
        {
          name: 'EK3_SOURCES_SET_TO_TERTIARY',
          unit: '87',
          description: '',
        },
        {
          name: 'AIRSPEED_PRIMARY_CHANGED',
          unit: '90',
          description: '',
        },
        {
          name: 'SURFACED',
          unit: '163',
          description: '',
        },
        {
          name: 'NOT_SURFACED',
          unit: '164',
          description: '',
        },
        {
          name: 'BOTTOMED',
          unit: '165',
          description: '',
        },
        {
          name: 'NOT_BOTTOMED',
          unit: '166',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'fcn',
    title: 'FCN',
    description: 'Filter Center Message - per motor',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'microseconds since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'instance',
        },
        {
          name: 'NF',
          unit: 'total number of active harmonic notches',
          description: '',
        },
        {
          name: 'CF1',
          unit: 'Hz',
          description: 'First harmonic centre frequency for motor 1',
        },
        {
          name: 'CF2',
          unit: 'Hz',
          description: 'First harmonic centre frequency for motor 2',
        },
        {
          name: 'CF3',
          unit: 'Hz',
          description: 'First harmonic centre frequency for motor 3',
        },
        {
          name: 'CF4',
          unit: 'Hz',
          description: 'First harmonic centre frequency for motor 4',
        },
        {
          name: 'CF5',
          unit: 'Hz',
          description: 'First harmonic centre frequency for motor 5',
        },
        {
          name: 'CF6',
          unit: 'Hz',
          description: 'First harmonic centre frequency for motor 6',
        },
        {
          name: 'HF1',
          unit: 'Hz',
          description: 'Second harmonic centre frequency for motor 1',
        },
        {
          name: 'HF2',
          unit: 'Hz',
          description: 'Second harmonic centre frequency for motor 2',
        },
        {
          name: 'HF3',
          unit: 'Hz',
          description: 'Second harmonic centre frequency for motor 3',
        },
        {
          name: 'HF4',
          unit: 'Hz',
          description: 'Second harmonic centre frequency for motor 4',
        },
        {
          name: 'HF5',
          unit: 'Hz',
          description: 'Second harmonic centre frequency for motor 5',
        },
        {
          name: 'HF6',
          unit: 'Hz',
          description: 'Second harmonic centre frequency for motor 6',
        },
      ],
    ],
  },
  {
    id: 'fcns',
    title: 'FCNS',
    description: 'Filter Center Message',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'microseconds since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'instance',
        },
        {
          name: 'CF',
          unit: 'Hz',
          description: 'notch centre frequency',
        },
        {
          name: 'HF',
          unit: 'Hz',
          description: '2nd harmonic frequency',
        },
      ],
    ],
  },
  {
    id: 'fenc',
    title: 'FENC',
    description: 'Fence status - development diagnostic message',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'EN',
          unit: 'bitmask of enabled fences',
          description: '',
        },
        {
          name: 'AE',
          unit: 'bitmask of automatically enabled fences',
          description: '',
        },
        {
          name: 'CF',
          unit: 'bitmask of configured-in-parameters fences',
          description: '',
        },
        {
          name: 'EF',
          unit: 'bitmask of enabled fences',
          description: '',
        },
        {
          name: 'DF',
          unit: 'bitmask of currently disabled fences',
          description: '',
        },
        {
          name: 'Alt',
          unit: 'current vehicle altitude',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'file',
    title: 'FILE',
    description: 'File data',
    tables: [
      [
        {
          name: 'FileName',
          unit: 'char[16]',
          description: 'File name',
        },
        {
          name: 'Offset',
          unit: 'Offset into the file of this block',
          description: '',
        },
        {
          name: 'Length',
          unit: 'Length of this data block',
          description: '',
        },
        {
          name: 'Data',
          unit: 'char[64]',
          description: 'File data of this block',
        },
      ],
    ],
  },
  {
    id: 'fmt',
    title: 'FMT',
    description: 'Message defining the format of messages in this file',
    tables: [
      [
        {
          name: 'Type',
          unit: 'unique-to-this-log identifier for message being defined',
          description: '',
        },
        {
          name: 'Length',
          unit: 'B',
          description: 'the number of bytes taken up by this message (including all headers)',
        },
        {
          name: 'Name',
          unit: 'char[4]',
          description: 'name of the message being defined',
        },
        {
          name: 'Format',
          unit: 'char[16]',
          description: 'character string defining the C-storage-type of the fields in this message',
        },
        {
          name: 'Columns',
          unit: 'char[64]',
          description: 'the labels of the message being defined',
        },
      ],
    ],
  },
  {
    id: 'fmtu',
    title: 'FMTU',
    description: 'Message defining units and multipliers used for fields of other messages',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'FmtType',
          unit: 'numeric reference to associated FMT message',
          description: '',
        },
        {
          name: 'UnitIds',
          unit: 'char[16]',
          description:
            'each character refers to a UNIT message. The unit at an offset corresponds to the field at the same offset in FMT.Format',
        },
        {
          name: 'MultIds',
          unit: 'char[16]',
          description:
            'each character refers to a MULT message. The multiplier at an offset corresponds to the field at the same offset in FMT.Format',
        },
      ],
    ],
  },
  {
    id: 'fnce',
    title: 'FNCE',
    description: 'currently loaded Geo Fence points',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Tot',
          unit: 'total number of stored items',
          description: '',
        },
        {
          name: 'Seq',
          unit: 'index in current sequence',
          description: '',
        },
        {
          name: 'Type',
          unit: 'point type',
          description: '',
        },
        {
          name: 'Lat',
          unit: 'deglatitude',
          description: 'point latitude',
        },
        {
          name: 'Lng',
          unit: 'deglongitude',
          description: 'point longitude',
        },
        {
          name: 'Count',
          unit: 'vertex cound in polygon if applicable',
          description: '',
        },
        {
          name: 'Radius',
          unit: 'm',
          description: 'radius of circle if applicable',
        },
      ],
    ],
  },
  {
    id: 'foll',
    title: 'FOLL',
    description: 'Follow library diagnostic data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup (microseconds)',
        },
        {
          name: 'Lat',
          unit: 'deglatitude',
          description: 'Target latitude (degrees * 1E7)',
        },
        {
          name: 'Lon',
          unit: 'deglongitude',
          description: 'Target longitude (degrees * 1E7)',
        },
        {
          name: 'Alt',
          unit: 'cm',
          description: 'Target absolute altitude (centimeters)',
        },
        {
          name: 'VelN',
          unit: 'm/s',
          description: 'Target velocity, North (m/s)',
        },
        {
          name: 'VelE',
          unit: 'm/s',
          description: 'Target velocity, East (m/s)',
        },
        {
          name: 'VelD',
          unit: 'm/s',
          description: 'Target velocity, Down (m/s)',
        },
        {
          name: 'LatE',
          unit: 'deglatitude',
          description: 'Vehicle estimated latitude (degrees * 1E7)',
        },
        {
          name: 'LonE',
          unit: 'deglongitude',
          description: 'Vehicle estimated longitude (degrees * 1E7)',
        },
        {
          name: 'AltE',
          unit: 'cm',
          description: 'Vehicle estimated altitude (centimeters)',
        },
        {
          name: 'FrmE',
          unit: 'Vehicle estimated altitude Frame',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'ftn',
    title: 'FTN',
    description: 'Filter Tuning Message - per motor',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'microseconds since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'instance',
        },
        {
          name: 'NDn',
          unit: 'number of active harmonic notches',
          description: '',
        },
        {
          name: 'NF1',
          unit: 'Hz',
          description: 'desired harmonic notch centre frequency for motor 1',
        },
        {
          name: 'NF2',
          unit: 'Hz',
          description: 'desired harmonic notch centre frequency for motor 2',
        },
        {
          name: 'NF3',
          unit: 'Hz',
          description: 'desired harmonic notch centre frequency for motor 3',
        },
        {
          name: 'NF4',
          unit: 'Hz',
          description: 'desired harmonic notch centre frequency for motor 4',
        },
        {
          name: 'NF5',
          unit: 'Hz',
          description: 'desired harmonic notch centre frequency for motor 5',
        },
        {
          name: 'NF6',
          unit: 'Hz',
          description: 'desired harmonic notch centre frequency for motor 6',
        },
        {
          name: 'NF7',
          unit: 'Hz',
          description: 'desired harmonic notch centre frequency for motor 7',
        },
        {
          name: 'NF8',
          unit: 'Hz',
          description: 'desired harmonic notch centre frequency for motor 8',
        },
        {
          name: 'NF9',
          unit: 'Hz',
          description: 'desired harmonic notch centre frequency for motor 9',
        },
        {
          name: 'NF10',
          unit: 'Hz',
          description: 'desired harmonic notch centre frequency for motor 10',
        },
        {
          name: 'NF11',
          unit: 'Hz',
          description: 'desired harmonic notch centre frequency for motor 11',
        },
        {
          name: 'NF12',
          unit: 'Hz',
          description: 'desired harmonic notch centre frequency for motor 12',
        },
      ],
    ],
  },
  {
    id: 'ftn1',
    title: 'FTN1',
    description: 'FFT Filter Tuning',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'microseconds since system startup',
        },
        {
          name: 'PkAvg',
          unit: 'Hz',
          description:
            'peak noise frequency as an energy-weighted average of roll and pitch peak frequencies',
        },
        {
          name: 'BwAvg',
          unit: 'Hz',
          description:
            'bandwidth of weighted peak frequency where edges are determined by FFT_ATT_REF',
        },
        {
          name: 'SnX',
          unit: 'signal-to-noise ratio on the roll axis',
          description: '',
        },
        {
          name: 'SnY',
          unit: 'signal-to-noise ratio on the pitch axis',
          description: '',
        },
        {
          name: 'SnZ',
          unit: 'signal-to-noise ratio on the yaw axis',
          description: '',
        },
        {
          name: 'FtX',
          unit: '%',
          description:
            'harmonic fit on roll of the highest noise peak to the second highest noise peak',
        },
        {
          name: 'FtY',
          unit: '%',
          description:
            'harmonic fit on pitch of the highest noise peak to the second highest noise peak',
        },
        {
          name: 'FtZ',
          unit: '%',
          description:
            'harmonic fit on yaw of the highest noise peak to the second highest noise peak',
        },
        {
          name: 'FHX',
          unit: 'FFT health, X-axis',
          description: '',
        },
        {
          name: 'FHY',
          unit: 'FFT health, Y-axis',
          description: '',
        },
        {
          name: 'FHZ',
          unit: 'FFT health, Z-axis',
          description: '',
        },
        {
          name: 'Tc',
          unit: 'μs',
          description: 'FFT cycle time',
        },
      ],
    ],
  },
  {
    id: 'ftn2',
    title: 'FTN2',
    description: 'FFT Noise Frequency Peak',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'microseconds since system startup',
        },
        {
          name: 'Id',
          unit: 'instance',
          description:
            'peak id where 0 is the centre peak, 1 is the lower shoulder and 2 is the upper shoulder',
        },
        {
          name: 'PkX',
          unit: 'Hz',
          description: 'noise frequency of the peak on roll',
        },
        {
          name: 'PkY',
          unit: 'Hz',
          description: 'noise frequency of the peak on pitch',
        },
        {
          name: 'PkZ',
          unit: 'Hz',
          description: 'noise frequency of the peak on yaw',
        },
        {
          name: 'BwX',
          unit: 'Hz',
          description:
            'bandwidth of the peak frequency on roll where edges are determined by FFT_ATT_REF',
        },
        {
          name: 'BwY',
          unit: 'Hz',
          description:
            'bandwidth of the peak frequency on pitch where edges are determined by FFT_ATT_REF',
        },
        {
          name: 'BwZ',
          unit: 'Hz',
          description:
            'bandwidth of the peak frequency on yaw where edges are determined by FFT_ATT_REF',
        },
        {
          name: 'SnX',
          unit: 'signal-to-noise ratio on the roll axis',
          description: '',
        },
        {
          name: 'SnY',
          unit: 'signal-to-noise ratio on the pitch axis',
          description: '',
        },
        {
          name: 'SnZ',
          unit: 'signal-to-noise ratio on the yaw axis',
          description: '',
        },
        {
          name: 'EnX',
          unit: 'power spectral density bin energy of the peak on roll',
          description: '',
        },
        {
          name: 'EnY',
          unit: 'power spectral density bin energy of the peak on roll',
          description: '',
        },
        {
          name: 'EnZ',
          unit: 'power spectral density bin energy of the peak on roll',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'ftn3',
    title: 'FTN3',
    description: 'Additional FFT Noise Frequency Peak',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'microseconds since system startup',
        },
        {
          name: 'Id',
          unit: 'instance',
          description: 'update axis',
        },
        {
          name: 'Pk1',
          unit: 'Hz',
          description: 'Peak 1 frequency',
        },
        {
          name: 'Pk2',
          unit: 'Hz',
          description: 'Peak 2 frequency',
        },
        {
          name: 'Pk3',
          unit: 'Hz',
          description: 'Peak 3 Frequency',
        },
        {
          name: 'Bw1',
          unit: 'Hz',
          description: 'Peak 1 noise bandwidth',
        },
        {
          name: 'Bw2',
          unit: 'Hz',
          description: 'Peak 2 noise bandwidth',
        },
        {
          name: 'Bw3',
          unit: 'Hz',
          description: 'Peak 3 noise bandwidth',
        },
        {
          name: 'En1',
          unit: 'Peak 1 Maximum energy',
          description: '',
        },
        {
          name: 'En2',
          unit: 'Peak 2 Maximum energy',
          description: '',
        },
        {
          name: 'En3',
          unit: 'Peak 3 Maximum energy',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'ftns',
    title: 'FTNS',
    description: 'Filter Tuning Message',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'microseconds since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'instance',
        },
        {
          name: 'NF',
          unit: 'Hz',
          description: 'desired harmonic notch centre frequency',
        },
      ],
    ],
  },
  {
    id: 'fwdt',
    title: 'FWDT',
    description: 'Forward Throttle calculations',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'fts',
          unit: 'forward throttle scaler',
          description: '',
        },
        {
          name: 'qfplcd',
          unit: 'quadplane forward pitch limit',
          description: '',
        },
        {
          name: 'npllcd',
          unit: 'navigation pitch lower limit',
          description: '',
        },
        {
          name: 'npcd',
          unit: 'demanded navigation pitch',
          description: '',
        },
        {
          name: 'qft',
          unit: 'quadplane forward throttle',
          description: '',
        },
        {
          name: 'npulcd',
          unit: 'upper limit for navigation pitch',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'glt',
    title: 'GLT',
    description: 'Simulated Glider Angles and coefficients',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'Alpha',
          unit: 'alpha angle',
          description: '',
        },
        {
          name: 'Beta',
          unit: 'beta angle',
          description: '',
        },
        {
          name: 'Cl',
          unit: 'lift coefficent',
          description: '',
        },
        {
          name: 'Cm',
          unit: 'roll coffecient',
          description: '',
        },
        {
          name: 'Cn',
          unit: 'yaw coefficient',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'gpa',
    title: 'GPA',
    description: 'GPS accuracy information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'GPS instance number',
        },
        {
          name: 'VDop',
          unit: 'vertical dilution of precision',
          description: '',
        },
        {
          name: 'HAcc',
          unit: 'm',
          description: 'horizontal position accuracy',
        },
        {
          name: 'VAcc',
          unit: 'm',
          description: 'vertical position accuracy',
        },
        {
          name: 'SAcc',
          unit: 'm/s',
          description: 'speed accuracy',
        },
        {
          name: 'YAcc',
          unit: 'deg',
          description: 'yaw accuracy',
        },
        {
          name: 'VV',
          unit: 'true if vertical velocity is available',
          description: '',
        },
        {
          name: 'SMS',
          unit: 'ms',
          description: 'time since system startup this sample was taken',
        },
        {
          name: 'Delta',
          unit: 'ms',
          description: 'system time delta between the last two reported positions',
        },
        {
          name: 'AEl',
          unit: 'm',
          description: 'altitude above WGS-84 ellipsoid; INT32_MIN (-2147483648) if unknown',
        },
        {
          name: 'RTCMFU',
          unit: 'RTCM fragments used',
          description: '',
        },
        {
          name: 'RTCMFD',
          unit: 'RTCM fragments discarded',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'gps',
    title: 'GPS',
    description: 'Information received from GNSS systems attached to the autopilot',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'GPS instance number',
        },
        {
          name: 'Status',
          unit: 'enum',
          description: 'GPS Fix type; 2D fix, 3D fix etc. Values:',
        },
        {
          name: 'NO_GPS',
          unit: '0',
          description: 'No GPS connected/detected',
        },
        {
          name: 'NO_FIX',
          unit: '1',
          description: 'Receiving valid GPS messages but no lock',
        },
        {
          name: 'GPS_OK_FIX_2D',
          unit: '2',
          description: 'Receiving valid messages and 2D lock',
        },
        {
          name: 'GPS_OK_FIX_3D',
          unit: '3',
          description: 'Receiving valid messages and 3D lock',
        },
        {
          name: 'GPS_OK_FIX_3D_DGPS',
          unit: '4',
          description: 'Receiving valid messages and 3D lock with differential improvements',
        },
        {
          name: 'GPS_OK_FIX_3D_RTK_FLOAT',
          unit: '5',
          description: 'Receiving valid messages and 3D RTK Float',
        },
        {
          name: 'GPS_OK_FIX_3D_RTK_FIXED',
          unit: '6',
          description: 'Receiving valid messages and 3D RTK Fixed',
        },
        {
          name: 'GMS',
          unit: 'ms',
          description: 'milliseconds since start of GPS Week',
        },
        {
          name: 'GWk',
          unit: 'weeks since 5 Jan 1980',
          description: '',
        },
        {
          name: 'NSats',
          unit: 'satellites',
          description: 'number of satellites visible',
        },
        {
          name: 'HDop',
          unit: 'horizontal dilution of precision',
          description: '',
        },
        {
          name: 'Lat',
          unit: 'deglatitude',
          description: 'latitude',
        },
        {
          name: 'Lng',
          unit: 'deglongitude',
          description: 'longitude',
        },
        {
          name: 'Alt',
          unit: 'm',
          description: 'altitude',
        },
        {
          name: 'Spd',
          unit: 'm/s',
          description: 'ground speed',
        },
        {
          name: 'GCrs',
          unit: 'degheading',
          description: 'ground course',
        },
        {
          name: 'VZ',
          unit: 'm/s',
          description: 'vertical speed',
        },
        {
          name: 'Yaw',
          unit: 'degheading',
          description: 'vehicle yaw',
        },
        {
          name: 'U',
          unit: 'boolean value indicating whether this GPS is in use',
          description: '',
        },
      ],
      [
        {
          name: 'NO_GPS',
          unit: '0',
          description: 'No GPS connected/detected',
        },
        {
          name: 'NO_FIX',
          unit: '1',
          description: 'Receiving valid GPS messages but no lock',
        },
        {
          name: 'GPS_OK_FIX_2D',
          unit: '2',
          description: 'Receiving valid messages and 2D lock',
        },
        {
          name: 'GPS_OK_FIX_3D',
          unit: '3',
          description: 'Receiving valid messages and 3D lock',
        },
        {
          name: 'GPS_OK_FIX_3D_DGPS',
          unit: '4',
          description: 'Receiving valid messages and 3D lock with differential improvements',
        },
        {
          name: 'GPS_OK_FIX_3D_RTK_FLOAT',
          unit: '5',
          description: 'Receiving valid messages and 3D RTK Float',
        },
        {
          name: 'GPS_OK_FIX_3D_RTK_FIXED',
          unit: '6',
          description: 'Receiving valid messages and 3D RTK Fixed',
        },
      ],
    ],
  },
  {
    id: 'gpyw',
    title: 'GPYW',
    description: 'GPS Yaw',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Id',
          unit: 'instance',
          description: 'instance',
        },
        {
          name: 'RHD',
          unit: 'deg',
          description: 'reported heading,deg',
        },
        {
          name: 'RDist',
          unit: 'm',
          description: 'antenna separation,m',
        },
        {
          name: 'RDown',
          unit: 'm',
          description: 'vertical antenna separation,m',
        },
        {
          name: 'MinCDown',
          unit: 'm',
          description: 'minimum tolerable vertical antenna separation,m',
        },
        {
          name: 'MaxCDown',
          unit: 'm',
          description: 'maximum tolerable vertical antenna separation,m',
        },
        {
          name: 'OK',
          unit: '1 if have yaw',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'graw',
    title: 'GRAW',
    description: 'Raw uBlox data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'WkMS',
          unit: 'ms',
          description: 'receiver TimeOfWeek measurement',
        },
        {
          name: 'Week',
          unit: 'GPS week',
          description: '',
        },
        {
          name: 'numSV',
          unit: 'satellites',
          description: 'number of space vehicles seen',
        },
        {
          name: 'sv',
          unit: 'space vehicle number of first vehicle',
          description: '',
        },
        {
          name: 'cpMes',
          unit: 'carrier phase measurement',
          description: '',
        },
        {
          name: 'prMes',
          unit: 'pseudorange measurement',
          description: '',
        },
        {
          name: 'doMes',
          unit: 'Doppler measurement',
          description: '',
        },
        {
          name: 'mesQI',
          unit: 'measurement quality index',
          description: '',
        },
        {
          name: 'cno',
          unit: 'carrier-to-noise density ratio',
          description: '',
        },
        {
          name: 'lli',
          unit: 'loss of lock indicator',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'grxh',
    title: 'GRXH',
    description: 'Raw uBlox data - header',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'rcvTime',
          unit: 'receiver TimeOfWeek measurement',
          description: '',
        },
        {
          name: 'week',
          unit: 'GPS week',
          description: '',
        },
        {
          name: 'leapS',
          unit: 'GPS leap seconds',
          description: '',
        },
        {
          name: 'numMeas',
          unit: 'number of space-vehicle measurements to follow',
          description: '',
        },
        {
          name: 'recStat',
          unit: 'receiver tracking status bitfield',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'grxs',
    title: 'GRXS',
    description: 'Raw uBlox data - space-vehicle data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'prMes',
          unit: 'Pseudorange measurement',
          description: '',
        },
        {
          name: 'cpMes',
          unit: 'Carrier phase measurement',
          description: '',
        },
        {
          name: 'doMes',
          unit: 'Doppler measurement',
          description: '',
        },
        {
          name: 'gnss',
          unit: 'GNSS identifier',
          description: '',
        },
        {
          name: 'sv',
          unit: 'Satellite identifier',
          description: '',
        },
        {
          name: 'freq',
          unit: 'GLONASS frequency slot',
          description: '',
        },
        {
          name: 'lock',
          unit: 'carrier phase locktime counter',
          description: '',
        },
        {
          name: 'cno',
          unit: 'carrier-to-noise density ratio',
          description: '',
        },
        {
          name: 'prD',
          unit: 'estimated pseudorange measurement standard deviation',
          description: '',
        },
        {
          name: 'cpD',
          unit: 'estimated carrier phase measurement standard deviation',
          description: '',
        },
        {
          name: 'doD',
          unit: 'estimated Doppler measurement standard deviation',
          description: '',
        },
        {
          name: 'trk',
          unit: 'tracking status bitfield',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'gyr',
    title: 'GYR',
    description: 'IMU gyroscope data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'gyroscope sensor instance number',
        },
        {
          name: 'SampleUS',
          unit: 'μs',
          description: 'time since system startup this sample was taken',
        },
        {
          name: 'GyrX',
          unit: 'rad/s',
          description: 'measured rotation rate about X axis',
        },
        {
          name: 'GyrY',
          unit: 'rad/s',
          description: 'measured rotation rate about Y axis',
        },
        {
          name: 'GyrZ',
          unit: 'rad/s',
          description: 'measured rotation rate about Z axis',
        },
      ],
    ],
  },
  {
    id: 'heat',
    title: 'HEAT',
    description: 'IMU Heater data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'Temp',
          unit: 'Current IMU temperature',
          description: '',
        },
        {
          name: 'Targ',
          unit: 'Target IMU temperature',
          description: '',
        },
        {
          name: 'P',
          unit: 'Proportional portion of response',
          description: '',
        },
        {
          name: 'I',
          unit: 'Integral portion of response',
          description: '',
        },
        {
          name: 'Out',
          unit: 'Controller output to heating element',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'hrsc',
    title: 'HRSC',
    description: 'Helicopter related messages',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'Instance, 0=Main, 1=Tail',
        },
        {
          name: 'DRRPM',
          unit: 'Desired rotor speed',
          description: '',
        },
        {
          name: 'ERRPM',
          unit: 'Estimated rotor speed',
          description: '',
        },
        {
          name: 'Gov',
          unit: 'Governor Output',
          description: '',
        },
        {
          name: 'Throt',
          unit: 'Throttle output',
          description: '',
        },
        {
          name: 'Ramp',
          unit: 'throttle ramp up',
          description: '',
        },
        {
          name: 'Stat',
          unit: 'RSC state',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'hygr',
    title: 'HYGR',
    description: 'Hygrometer data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Id',
          unit: 'instance',
          description: 'sensor ID',
        },
        {
          name: 'Humidity',
          unit: '%',
          description: 'percentage humidity',
        },
        {
          name: 'Temp',
          unit: 'degC',
          description: 'temperature in degrees C',
        },
      ],
    ],
  },
  {
    id: 'icmb',
    title: 'ICMB',
    description: 'ICM20789 diagnostics',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'Traw',
          unit: 'raw temperature from sensor',
          description: '',
        },
        {
          name: 'Praw',
          unit: 'raw pressure from sensor',
          description: '',
        },
        {
          name: 'P',
          unit: 'pressure',
          description: '',
        },
        {
          name: 'T',
          unit: 'temperature',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'ie24',
    title: 'IE24',
    description: 'Intelligent Energy Fuel Cell generator (legacy protocol)',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'FUEL',
          unit: '1e2 %',
          description: 'Fuel remaining',
        },
        {
          name: 'SPMPWR',
          unit: 'Watt',
          description: 'stack power module power draw',
        },
        {
          name: 'POUT',
          unit: 'Watt',
          description: 'output power',
        },
        {
          name: 'ERR',
          unit: 'enum',
          description: 'error codes Values:',
        },
        {
          name: 'MINOR_INTERNAL',
          unit: '1',
          description: 'Minor internal error is to be ignored',
        },
        {
          name: 'REDUCED_POWER',
          unit: '10',
          description: '',
        },
        {
          name: 'SPM_LOST',
          unit: '11',
          description: '',
        },
        {
          name: 'PRESSURE_LOW',
          unit: '20',
          description: '',
        },
        {
          name: 'BATTERY_LOW',
          unit: '21',
          description: '',
        },
        {
          name: 'PRESSURE_ALERT',
          unit: '30',
          description: '',
        },
        {
          name: 'START_DENIED',
          unit: '31',
          description: '',
        },
        {
          name: 'SYSTEM_CRITICAL',
          unit: '32',
          description: '',
        },
        {
          name: 'PRESSURE_CRITICAL',
          unit: '33',
          description: '',
        },
        {
          name: 'BATTERY_CRITICAL',
          unit: '40',
          description: '',
        },
      ],
      [
        {
          name: 'MINOR_INTERNAL',
          unit: '1',
          description: 'Minor internal error is to be ignored',
        },
        {
          name: 'REDUCED_POWER',
          unit: '10',
          description: '',
        },
        {
          name: 'SPM_LOST',
          unit: '11',
          description: '',
        },
        {
          name: 'PRESSURE_LOW',
          unit: '20',
          description: '',
        },
        {
          name: 'BATTERY_LOW',
          unit: '21',
          description: '',
        },
        {
          name: 'PRESSURE_ALERT',
          unit: '30',
          description: '',
        },
        {
          name: 'START_DENIED',
          unit: '31',
          description: '',
        },
        {
          name: 'SYSTEM_CRITICAL',
          unit: '32',
          description: '',
        },
        {
          name: 'PRESSURE_CRITICAL',
          unit: '33',
          description: '',
        },
        {
          name: 'BATTERY_CRITICAL',
          unit: '40',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'iefc',
    title: 'IEFC',
    description: 'Intelligent Energy Fuel Cell generator',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Tank',
          unit: '%',
          description: 'Fuel remaining',
        },
        {
          name: 'Inlet',
          unit: 'Inlet pressure',
          description: '',
        },
        {
          name: 'BattV',
          unit: 'V',
          description: 'battery voltage',
        },
        {
          name: 'OutPwr',
          unit: 'Watt',
          description: 'output power',
        },
        {
          name: 'SPMPwr',
          unit: 'Watt',
          description: 'stack power module power draw',
        },
        {
          name: 'FNo',
          unit: 'fault number',
          description: '',
        },
        {
          name: 'BPwr',
          unit: 'Watt',
          description: 'battery power draw',
        },
        {
          name: 'State',
          unit: 'enum',
          description: 'generator state Values:',
        },
        {
          name: 'FCPM_Off',
          unit: '0',
          description: '',
        },
        {
          name: 'Starting',
          unit: '1',
          description: '',
        },
        {
          name: 'Running',
          unit: '2',
          description: '',
        },
        {
          name: 'Stopping',
          unit: '3',
          description: '',
        },
        {
          name: 'Go_to_Sleep',
          unit: '4',
          description: '',
        },
        {
          name: 'F1',
          unit: 'enum',
          description: 'error code Values:',
        },
        {
          name: 'MINOR_INTERNAL',
          unit: '1',
          description: 'Minor internal error is to be ignored',
        },
        {
          name: 'REDUCED_POWER',
          unit: '10',
          description: '',
        },
        {
          name: 'SPM_LOST',
          unit: '11',
          description: '',
        },
        {
          name: 'PRESSURE_LOW',
          unit: '20',
          description: '',
        },
        {
          name: 'BATTERY_LOW',
          unit: '21',
          description: '',
        },
        {
          name: 'PRESSURE_ALERT',
          unit: '30',
          description: '',
        },
        {
          name: 'START_DENIED',
          unit: '31',
          description: '',
        },
        {
          name: 'SYSTEM_CRITICAL',
          unit: '32',
          description: '',
        },
        {
          name: 'PRESSURE_CRITICAL',
          unit: '33',
          description: '',
        },
        {
          name: 'BATTERY_CRITICAL',
          unit: '40',
          description: '',
        },
        {
          name: 'F2',
          unit: 'sub-error code',
          description: '',
        },
      ],
      [
        {
          name: 'FCPM_Off',
          unit: '0',
          description: '',
        },
        {
          name: 'Starting',
          unit: '1',
          description: '',
        },
        {
          name: 'Running',
          unit: '2',
          description: '',
        },
        {
          name: 'Stopping',
          unit: '3',
          description: '',
        },
        {
          name: 'Go_to_Sleep',
          unit: '4',
          description: '',
        },
      ],
      [
        {
          name: 'MINOR_INTERNAL',
          unit: '1',
          description: 'Minor internal error is to be ignored',
        },
        {
          name: 'REDUCED_POWER',
          unit: '10',
          description: '',
        },
        {
          name: 'SPM_LOST',
          unit: '11',
          description: '',
        },
        {
          name: 'PRESSURE_LOW',
          unit: '20',
          description: '',
        },
        {
          name: 'BATTERY_LOW',
          unit: '21',
          description: '',
        },
        {
          name: 'PRESSURE_ALERT',
          unit: '30',
          description: '',
        },
        {
          name: 'START_DENIED',
          unit: '31',
          description: '',
        },
        {
          name: 'SYSTEM_CRITICAL',
          unit: '32',
          description: '',
        },
        {
          name: 'PRESSURE_CRITICAL',
          unit: '33',
          description: '',
        },
        {
          name: 'BATTERY_CRITICAL',
          unit: '40',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'ilb1',
    title: 'ILB1',
    description: 'InertialLabs AHRS data1',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'GMS',
          unit: 'GPS INS time (round)',
          description: '',
        },
        {
          name: 'GyrX',
          unit: 'deg/s',
          description: 'Gyro X',
        },
        {
          name: 'GyrY',
          unit: 'deg/s',
          description: 'Gyro Y',
        },
        {
          name: 'GyrZ',
          unit: 'deg/s',
          description: 'Gyro z',
        },
        {
          name: 'AccX',
          unit: 'm/s/s',
          description: 'Accelerometer X',
        },
        {
          name: 'AccY',
          unit: 'm/s/s',
          description: 'Accelerometer Y',
        },
        {
          name: 'AccZ',
          unit: 'm/s/s',
          description: 'Accelerometer Z',
        },
      ],
    ],
  },
  {
    id: 'ilb2',
    title: 'ILB2',
    description: 'InertialLabs AHRS data2',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'GMS',
          unit: 'GPS INS time (round)',
          description: '',
        },
        {
          name: 'MagX',
          unit: 'Magnetometer X',
          description: '',
        },
        {
          name: 'MagY',
          unit: 'Magnetometer Y',
          description: '',
        },
        {
          name: 'MagZ',
          unit: 'Magnetometer Z',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'ilb3',
    title: 'ILB3',
    description: 'InertialLabs AHRS data3',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'GMS',
          unit: 'GPS INS time (round)',
          description: '',
        },
        {
          name: 'Press',
          unit: 'Pa',
          description: 'Static pressure',
        },
        {
          name: 'Diff',
          unit: 'Pa',
          description: 'Differential pressure',
        },
        {
          name: 'Temp',
          unit: 'degC',
          description: 'Temperature',
        },
        {
          name: 'Alt',
          unit: 'm',
          description: 'Baro altitude',
        },
        {
          name: 'TAS',
          unit: 'm/s',
          description: 'true airspeed',
        },
        {
          name: 'VWN',
          unit: 'm/s',
          description: 'Wind velocity north',
        },
        {
          name: 'VWE',
          unit: 'm/s',
          description: 'Wind velocity east',
        },
        {
          name: 'VWD',
          unit: 'm/s',
          description: 'Wind velocity down',
        },
        {
          name: 'ADU',
          unit: 'Air Data Unit status',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'ilb4',
    title: 'ILB4',
    description: 'InertialLabs AHRS data4',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'GMS',
          unit: 'GNSS Position timestamp',
          description: '',
        },
        {
          name: 'GWk',
          unit: 'GPS Week',
          description: '',
        },
        {
          name: 'NSat',
          unit: 'Number of satellites',
          description: '',
        },
        {
          name: 'NewGPS',
          unit: 'Indicator of new update of GPS data',
          description: '',
        },
        {
          name: 'Lat',
          unit: 'deglatitude',
          description: 'GNSS Latitude',
        },
        {
          name: 'Lng',
          unit: 'deglongitude',
          description: 'GNSS Longitude',
        },
        {
          name: 'Alt',
          unit: 'm',
          description: 'GNSS Altitude',
        },
        {
          name: 'GCrs',
          unit: 'degheading',
          description: 'GNSS Track over ground',
        },
        {
          name: 'Spd',
          unit: 'm/s',
          description: 'GNSS Horizontal speed',
        },
        {
          name: 'VZ',
          unit: 'm/s',
          description: 'GNSS Vertical speed',
        },
      ],
    ],
  },
  {
    id: 'ilb5',
    title: 'ILB5',
    description: 'InertialLabs AHRS data5',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'GMS',
          unit: 'GNSS Position timestamp',
          description: '',
        },
        {
          name: 'FType',
          unit: 'fix type',
          description: '',
        },
        {
          name: 'GSS',
          unit: 'GNSS spoofing status',
          description: '',
        },
        {
          name: 'GJS',
          unit: 'GNSS jamming status',
          description: '',
        },
        {
          name: 'GI1',
          unit: 'GNSS Info1',
          description: '',
        },
        {
          name: 'GI2',
          unit: 'GNSS Info2',
          description: '',
        },
        {
          name: 'GAPS',
          unit: 'GNSS Angles position type',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'ilb6',
    title: 'ILB6',
    description: 'InertialLabs AHRS data6',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'GMS',
          unit: 'GNSS Position timestamp',
          description: '',
        },
        {
          name: 'GpsHTS',
          unit: 'GNSS Heading timestamp',
          description: '',
        },
        {
          name: 'GpsYaw',
          unit: 'degheading',
          description: 'GNSS Heading',
        },
        {
          name: 'GpsPitch',
          unit: 'deg',
          description: 'GNSS Pitch',
        },
        {
          name: 'GDOP',
          unit: 'GNSS GDOP',
          description: '',
        },
        {
          name: 'PDOP',
          unit: 'GNSS PDOP',
          description: '',
        },
        {
          name: 'HDOP',
          unit: 'GNSS HDOP',
          description: '',
        },
        {
          name: 'VDOP',
          unit: 'GNSS VDOP',
          description: '',
        },
        {
          name: 'TDOP',
          unit: 'GNSS TDOP',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'ilb7',
    title: 'ILB7',
    description: 'InertialLabs AHRS data7',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'GMS',
          unit: 'GPS INS time (round)',
          description: '',
        },
        {
          name: 'Roll',
          unit: 'deg',
          description: 'euler roll',
        },
        {
          name: 'Pitch',
          unit: 'deg',
          description: 'euler pitch',
        },
        {
          name: 'Yaw',
          unit: 'deg',
          description: 'euler yaw',
        },
        {
          name: 'VN',
          unit: 'm/s',
          description: 'velocity north',
        },
        {
          name: 'VE',
          unit: 'm/s',
          description: 'velocity east',
        },
        {
          name: 'VD',
          unit: 'm/s',
          description: 'velocity down',
        },
        {
          name: 'Lat',
          unit: 'deglatitude',
          description: 'latitude',
        },
        {
          name: 'Lng',
          unit: 'deglongitude',
          description: 'longitude',
        },
        {
          name: 'Alt',
          unit: 'm',
          description: 'altitude MSL',
        },
        {
          name: 'USW',
          unit: 'USW1',
          description: '',
        },
        {
          name: 'USW2',
          unit: 'USW2',
          description: '',
        },
        {
          name: 'Vdc',
          unit: 'V',
          description: 'Supply voltage',
        },
      ],
    ],
  },
  {
    id: 'ilb8',
    title: 'ILB8',
    description: 'InertialLabs AHRS data8',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'GMS',
          unit: 'GPS INS time (round)',
          description: '',
        },
        {
          name: 'PVN',
          unit: 'm',
          description: 'position variance north',
        },
        {
          name: 'PVE',
          unit: 'm',
          description: 'position variance east',
        },
        {
          name: 'PVD',
          unit: 'm',
          description: 'position variance down',
        },
        {
          name: 'VVN',
          unit: 'm/s',
          description: 'velocity variance north',
        },
        {
          name: 'VVE',
          unit: 'm/s',
          description: 'velocity variance east',
        },
        {
          name: 'VVD',
          unit: 'm/s',
          description: 'velocity variance down',
        },
      ],
    ],
  },
  {
    id: 'imu',
    title: 'IMU',
    description: 'Inertial Measurement Unit data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'IMU sensor instance number',
        },
        {
          name: 'GyrX',
          unit: 'rad/s',
          description: 'measured rotation rate about X axis',
        },
        {
          name: 'GyrY',
          unit: 'rad/s',
          description: 'measured rotation rate about Y axis',
        },
        {
          name: 'GyrZ',
          unit: 'rad/s',
          description: 'measured rotation rate about Z axis',
        },
        {
          name: 'AccX',
          unit: 'm/s/s',
          description: 'acceleration along X axis',
        },
        {
          name: 'AccY',
          unit: 'm/s/s',
          description: 'acceleration along Y axis',
        },
        {
          name: 'AccZ',
          unit: 'm/s/s',
          description: 'acceleration along Z axis',
        },
        {
          name: 'EG',
          unit: 'gyroscope error count',
          description: '',
        },
        {
          name: 'EA',
          unit: 'accelerometer error count',
          description: '',
        },
        {
          name: 'T',
          unit: 'degC',
          description: 'IMU temperature',
        },
        {
          name: 'GH',
          unit: 'gyroscope health',
          description: '',
        },
        {
          name: 'AH',
          unit: 'accelerometer health',
          description: '',
        },
        {
          name: 'GHz',
          unit: 'Hz',
          description: 'gyroscope measurement rate',
        },
        {
          name: 'AHz',
          unit: 'Hz',
          description: 'accelerometer measurement rate',
        },
      ],
    ],
  },
  {
    id: 'iomc',
    title: 'IOMC',
    description: 'IOMCU diagnostic information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'RSErr',
          unit: 'Status Read error count (zeroed on successful read)',
          description: '',
        },
        {
          name: 'Mem',
          unit: 'Free memory',
          description: '',
        },
        {
          name: 'TS',
          unit: 'IOMCU uptime',
          description: '',
        },
        {
          name: 'NPkt',
          unit: 'Number of packets received by IOMCU',
          description: '',
        },
        {
          name: 'Nerr',
          unit: 'Protocol failures on MCU side',
          description: '',
        },
        {
          name: 'Nerr2',
          unit: 'Reported number of failures on IOMCU side',
          description: '',
        },
        {
          name: 'NDel',
          unit: 'Number of delayed packets received by MCU',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'ireg',
    title: 'IREG',
    description: 'IMU Register unexpected value change',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'DevID',
          unit: 'bus ID',
          description: '',
        },
        {
          name: 'Bank',
          unit: 'device register bank',
          description: '',
        },
        {
          name: 'Reg',
          unit: 'device register',
          description: '',
        },
        {
          name: 'Val',
          unit: 'unexpected value',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'isbd',
    title: 'ISBD',
    description: 'InertialSensor Batch Logging Data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'N',
          unit: 'batch sequence number',
          description: '',
        },
        {
          name: 'seqno',
          unit: 'sample sequence number',
          description: '',
        },
        {
          name: 'x',
          unit: 'm/s/s',
          description: 'x-axis sample value',
        },
        {
          name: 'y',
          unit: 'm/s/s',
          description: 'y-axis sample value',
        },
        {
          name: 'z',
          unit: 'm/s/s',
          description: 'z-axis sample value',
        },
      ],
    ],
  },
  {
    id: 'isbh',
    title: 'ISBH',
    description: 'InertialSensor Batch Logging Header',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'N',
          unit: 'batch sequence number',
          description: '',
        },
        {
          name: 'type',
          unit: 'indicates if this is accel or gyro data',
          description: '',
        },
        {
          name: 'instance',
          unit: 'IMU sensor instance',
          description: '',
        },
        {
          name: 'mul',
          unit: 'multiplier to be applied to samples in this batch',
          description: '',
        },
        {
          name: 'smp_cnt',
          unit: 'samples in this batch',
          description: '',
        },
        {
          name: 'SampleUS',
          unit: 'μs',
          description: 'timestamp of first sample',
        },
        {
          name: 'smp_rate',
          unit: 'Hz',
          description: 'rate at which samples have been collected',
        },
      ],
    ],
  },
  {
    id: 'jsn1',
    title: 'JSN1',
    description: 'Log data received from JSON simulator',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup (us)',
        },
        {
          name: 'TStamp',
          unit: 's',
          description: 'Simulation’s timestamp (s)',
        },
        {
          name: 'R',
          unit: 'rad',
          description: 'Simulation’s roll (rad)',
        },
        {
          name: 'P',
          unit: 'rad',
          description: 'Simulation’s pitch (rad)',
        },
        {
          name: 'Y',
          unit: 'rad',
          description: 'Simulation’s yaw (rad)',
        },
        {
          name: 'GX',
          unit: 'rad/s',
          description: 'Simulated gyroscope, X-axis (rad/sec)',
        },
        {
          name: 'GY',
          unit: 'rad/s',
          description: 'Simulated gyroscope, Y-axis (rad/sec)',
        },
        {
          name: 'GZ',
          unit: 'rad/s',
          description: 'Simulated gyroscope, Z-axis (rad/sec)',
        },
      ],
    ],
  },
  {
    id: 'jsn2',
    title: 'JSN2',
    description: 'Log data received from JSON simulator',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup (us)',
        },
        {
          name: 'VN',
          unit: 'm/s',
          description: 'simulation’s velocity, North-axis (m/s)',
        },
        {
          name: 'VE',
          unit: 'm/s',
          description: 'simulation’s velocity, East-axis (m/s)',
        },
        {
          name: 'VD',
          unit: 'm/s',
          description: 'simulation’s velocity, Down-axis (m/s)',
        },
        {
          name: 'AX',
          unit: 'm/s/s',
          description: 'simulation’s acceleration, X-axis (m/s^2)',
        },
        {
          name: 'AY',
          unit: 'm/s/s',
          description: 'simulation’s acceleration, Y-axis (m/s^2)',
        },
        {
          name: 'AZ',
          unit: 'm/s/s',
          description: 'simulation’s acceleration, Z-axis (m/s^2)',
        },
        {
          name: 'AN',
          unit: 'm/s/s',
          description: 'simulation’s acceleration, North (m/s^2)',
        },
        {
          name: 'AE',
          unit: 'm/s/s',
          description: 'simulation’s acceleration, East (m/s^2)',
        },
        {
          name: 'AD',
          unit: 'm/s/s',
          description: 'simulation’s acceleration, Down (m/s^2)',
        },
      ],
    ],
  },
  {
    id: 'land',
    title: 'LAND',
    description: 'Slope Landing data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'stage',
          unit: 'progress through landing sequence',
          description: '',
        },
        {
          name: 'f1',
          unit: 'Landing flags',
          description: '',
        },
        {
          name: 'f2',
          unit: 'Slope-specific landing flags',
          description: '',
        },
        {
          name: 'slope',
          unit: 'Slope to landing point',
          description: '',
        },
        {
          name: 'slopeInit',
          unit: 'Initial slope to landing point',
          description: '',
        },
        {
          name: 'altO',
          unit: 'Rangefinder correction',
          description: '',
        },
        {
          name: 'fh',
          unit: 'Height for flare timing.',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'lgr',
    title: 'LGR',
    description: 'Landing gear information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'LandingGear',
          unit: 'enum',
          description: 'Current landing gear state Values:',
        },
        {
          name: 'LG_UNKNOWN',
          unit: '-1',
          description: '',
        },
        {
          name: 'LG_RETRACTED',
          unit: '0',
          description: '',
        },
        {
          name: 'LG_DEPLOYED',
          unit: '1',
          description: '',
        },
        {
          name: 'LG_RETRACTING',
          unit: '2',
          description: '',
        },
        {
          name: 'LG_DEPLOYING',
          unit: '3',
          description: '',
        },
        {
          name: 'WeightOnWheels',
          unit: 'enum',
          description: 'Weight on wheels state Values:',
        },
        {
          name: 'LG_WOW_UNKNOWN',
          unit: '-1',
          description: '',
        },
        {
          name: 'LG_NO_WOW',
          unit: '0',
          description: '',
        },
        {
          name: 'LG_WOW',
          unit: '1',
          description: '',
        },
      ],
      [
        {
          name: 'LG_UNKNOWN',
          unit: '-1',
          description: '',
        },
        {
          name: 'LG_RETRACTED',
          unit: '0',
          description: '',
        },
        {
          name: 'LG_DEPLOYED',
          unit: '1',
          description: '',
        },
        {
          name: 'LG_RETRACTING',
          unit: '2',
          description: '',
        },
        {
          name: 'LG_DEPLOYING',
          unit: '3',
          description: '',
        },
      ],
      [
        {
          name: 'LG_WOW_UNKNOWN',
          unit: '-1',
          description: '',
        },
        {
          name: 'LG_NO_WOW',
          unit: '0',
          description: '',
        },
        {
          name: 'LG_WOW',
          unit: '1',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'loec',
    title: 'LOEC',
    description: 'Gathered Loweheiser EFI/Governor telemetry',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'SI',
          unit: 'target system ID',
          description: '',
        },
        {
          name: 'CI',
          unit: 'target component ID',
          description: '',
        },
        {
          name: 'C',
          unit: 'command',
          description: '',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'efi index',
        },
        {
          name: 'ES',
          unit: 'desired engine state (0:EFI off 1:EFI on)',
          description: '',
        },
        {
          name: 'GS',
          unit: 'desired governor state (0:Governor off 1:Governor on)',
          description: '',
        },
        {
          name: 'Thr',
          unit: 'manual throttle value',
          description: '',
        },
        {
          name: 'Strtr',
          unit: 'desired electric start',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'loeg',
    title: 'LOEG',
    description: 'Gathered Loweheiser EFI/Governor telemetry',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'EFI/Gov sensor instance number',
        },
        {
          name: 'VB',
          unit: 'V',
          description: 'battery voltage',
        },
        {
          name: 'CB',
          unit: 'A',
          description: 'battery current',
        },
        {
          name: 'CG',
          unit: 'A',
          description: 'generator current',
        },
        {
          name: 'Th',
          unit: '%',
          description: 'throttle input',
        },
        {
          name: 'EB',
          unit: 'V',
          description: 'EFI battery voltage',
        },
        {
          name: 'RPM',
          unit: 'rpm',
          description: 'generator RPM',
        },
        {
          name: 'PW',
          unit: 'μs',
          description: 'EFI pulse-width',
        },
        {
          name: 'FF',
          unit: 'l/s',
          description: 'fuel flow',
        },
        {
          name: 'FC',
          unit: 'l',
          description: 'fuel consumed',
        },
        {
          name: 'EP',
          unit: 'Pa',
          description: 'EFI pressure',
        },
        {
          name: 'EMT',
          unit: 'degC',
          description: 'EFI manifold air temperature',
        },
        {
          name: 'CHT',
          unit: 'degC',
          description: 'cylinder head temperature',
        },
        {
          name: 'TPS',
          unit: '%',
          description: 'throttle position sensor',
        },
      ],
    ],
  },
  {
    id: 'mag',
    title: 'MAG',
    description: 'Information received from compasses',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'magnetometer sensor instance number',
        },
        {
          name: 'MagX',
          unit: 'mGauss',
          description: 'magnetic field strength in body frame',
        },
        {
          name: 'MagY',
          unit: 'mGauss',
          description: 'magnetic field strength in body frame',
        },
        {
          name: 'MagZ',
          unit: 'mGauss',
          description: 'magnetic field strength in body frame',
        },
        {
          name: 'OfsX',
          unit: 'mGauss',
          description: 'magnetic field offset in body frame',
        },
        {
          name: 'OfsY',
          unit: 'mGauss',
          description: 'magnetic field offset in body frame',
        },
        {
          name: 'OfsZ',
          unit: 'mGauss',
          description: 'magnetic field offset in body frame',
        },
        {
          name: 'MOX',
          unit: 'mGauss',
          description: 'motor interference magnetic field offset in body frame',
        },
        {
          name: 'MOY',
          unit: 'mGauss',
          description: 'motor interference magnetic field offset in body frame',
        },
        {
          name: 'MOZ',
          unit: 'mGauss',
          description: 'motor interference magnetic field offset in body frame',
        },
        {
          name: 'Health',
          unit: 'true if the compass is considered healthy',
          description: '',
        },
        {
          name: 'S',
          unit: 'μs',
          description: 'time measurement was taken',
        },
      ],
    ],
  },
  {
    id: 'magh',
    title: 'MAGH',
    description: 'Magnetometer high resolution data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Node',
          unit: 'instance',
          description: 'CAN node',
        },
        {
          name: 'Sensor',
          unit: 'sensor ID on node',
          description: '',
        },
        {
          name: 'Bus',
          unit: 'CAN bus',
          description: '',
        },
        {
          name: 'Mx',
          unit: 'X axis field',
          description: '',
        },
        {
          name: 'My',
          unit: 'y axis field',
          description: '',
        },
        {
          name: 'Mz',
          unit: 'z axis field',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'mav',
    title: 'MAV',
    description: 'GCS MAVLink link statistics',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'chan',
          unit: 'instance',
          description: 'mavlink channel number',
        },
        {
          name: 'txp',
          unit: 'transmitted packet count',
          description: '',
        },
        {
          name: 'rxp',
          unit: 'received packet count',
          description: '',
        },
        {
          name: 'rxdp',
          unit: 'perceived number of packets we never received',
          description: '',
        },
        {
          name: 'flags',
          unit: 'bitmask',
          description: 'compact representation of some state of the channel Bitmask values:',
        },
        {
          name: 'USING_SIGNING',
          unit: '1',
          description: '',
        },
        {
          name: 'ACTIVE',
          unit: '2',
          description: '',
        },
        {
          name: 'STREAMING',
          unit: '4',
          description: '',
        },
        {
          name: 'PRIVATE',
          unit: '8',
          description: '',
        },
        {
          name: 'LOCKED',
          unit: '16',
          description: '',
        },
        {
          name: 'ss',
          unit: 'ms',
          description:
            'stream slowdown is the number of ms being added to each message to fit within bandwidth',
        },
        {
          name: 'tf',
          unit: 'times buffer was full when a message was going to be sent',
          description: '',
        },
        {
          name: 'mgs',
          unit: 'ms',
          description: 'time MAV_GCS_SYSID heartbeat (or manual control) last seen',
        },
      ],
      [
        {
          name: 'USING_SIGNING',
          unit: '1',
          description: '',
        },
        {
          name: 'ACTIVE',
          unit: '2',
          description: '',
        },
        {
          name: 'STREAMING',
          unit: '4',
          description: '',
        },
        {
          name: 'PRIVATE',
          unit: '8',
          description: '',
        },
        {
          name: 'LOCKED',
          unit: '16',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'mavc',
    title: 'MAVC',
    description: 'MAVLink command we have just executed',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'TS',
          unit: 'target system for command',
          description: '',
        },
        {
          name: 'TC',
          unit: 'target component for command',
          description: '',
        },
        {
          name: 'SS',
          unit: 'source system for command',
          description: '',
        },
        {
          name: 'SC',
          unit: 'source component for command',
          description: '',
        },
        {
          name: 'Fr',
          unit: 'command frame',
          description: '',
        },
        {
          name: 'Cmd',
          unit: 'mavlink command enum value',
          description: '',
        },
        {
          name: 'P1',
          unit: 'first parameter from mavlink packet',
          description: '',
        },
        {
          name: 'P2',
          unit: 'second parameter from mavlink packet',
          description: '',
        },
        {
          name: 'P3',
          unit: 'third parameter from mavlink packet',
          description: '',
        },
        {
          name: 'P4',
          unit: 'fourth parameter from mavlink packet',
          description: '',
        },
        {
          name: 'X',
          unit: 'X coordinate from mavlink packet',
          description: '',
        },
        {
          name: 'Y',
          unit: 'Y coordinate from mavlink packet',
          description: '',
        },
        {
          name: 'Z',
          unit: 'Z coordinate from mavlink packet',
          description: '',
        },
        {
          name: 'Res',
          unit: 'command result being returned from autopilot',
          description: '',
        },
        {
          name: 'WL',
          unit: 'true if this command arrived via a COMMAND_LONG rather than COMMAND_INT',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'mcu',
    title: 'MCU',
    description: 'MCU voltage and temprature monitering',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'MTemp',
          unit: 'degC',
          description: 'Temperature',
        },
        {
          name: 'MVolt',
          unit: 'V',
          description: 'Voltage',
        },
        {
          name: 'MVmin',
          unit: 'V',
          description: 'Voltage min',
        },
        {
          name: 'MVmax',
          unit: 'V',
          description: 'Voltage max',
        },
      ],
    ],
  },
  {
    id: 'mise',
    title: 'MISE',
    description: 'Executed mission command information; emitted when we start to run an item',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'CTot',
          unit: 'Total number of mission commands',
          description: '',
        },
        {
          name: 'CNum',
          unit: 'This command’s offset in mission',
          description: '',
        },
        {
          name: 'CId',
          unit: 'Command type',
          description: '',
        },
        {
          name: 'Prm1',
          unit: 'Parameter 1',
          description: '',
        },
        {
          name: 'Prm2',
          unit: 'Parameter 2',
          description: '',
        },
        {
          name: 'Prm3',
          unit: 'Parameter 3',
          description: '',
        },
        {
          name: 'Prm4',
          unit: 'Parameter 4',
          description: '',
        },
        {
          name: 'Lat',
          unit: 'deglatitude',
          description: 'Command latitude',
        },
        {
          name: 'Lng',
          unit: 'deglongitude',
          description: 'Command longitude',
        },
        {
          name: 'Alt',
          unit: 'm',
          description: 'Command altitude',
        },
        {
          name: 'Frame',
          unit: 'Frame used for position',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'mmo',
    title: 'MMO',
    description: 'MMC3416 compass data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'Nx',
          unit: 'new measurement X axis',
          description: '',
        },
        {
          name: 'Ny',
          unit: 'new measurement Y axis',
          description: '',
        },
        {
          name: 'Nz',
          unit: 'new measurement Z axis',
          description: '',
        },
        {
          name: 'Ox',
          unit: 'new offset X axis',
          description: '',
        },
        {
          name: 'Oy',
          unit: 'new offset Y axis',
          description: '',
        },
        {
          name: 'Oz',
          unit: 'new offset Z axis',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'mnt',
    title: 'MNT',
    description: 'Mount’s desired and actual roll, pitch and yaw angles',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'Instance number',
        },
        {
          name: 'DRoll',
          unit: 'deg',
          description: 'Desired roll',
        },
        {
          name: 'Roll',
          unit: 'deg',
          description: 'Actual roll',
        },
        {
          name: 'DPitch',
          unit: 'deg',
          description: 'Desired pitch',
        },
        {
          name: 'Pitch',
          unit: 'deg',
          description: 'Actual pitch',
        },
        {
          name: 'DYawB',
          unit: 'deg',
          description: 'Desired yaw in body frame',
        },
        {
          name: 'YawB',
          unit: 'deg',
          description: 'Actual yaw in body frame',
        },
        {
          name: 'DYawE',
          unit: 'deg',
          description: 'Desired yaw in earth frame',
        },
        {
          name: 'YawE',
          unit: 'deg',
          description: 'Actual yaw in earth frame',
        },
        {
          name: 'Dist',
          unit: 'm',
          description: 'Rangefinder distance',
        },
      ],
    ],
  },
  {
    id: 'mode',
    title: 'MODE',
    description: 'vehicle control mode information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Mode',
          unit: 'vehicle-specific mode number',
          description: '',
        },
        {
          name: 'ModeNum',
          unit: 'alias for Mode',
          description: '',
        },
        {
          name: 'Rsn',
          unit: 'enum',
          description: 'reason for entering this mode; enumeration value Values:',
        },
        {
          name: 'UNKNOWN',
          unit: '0',
          description: '',
        },
        {
          name: 'RC_COMMAND',
          unit: '1',
          description: '',
        },
        {
          name: 'GCS_COMMAND',
          unit: '2',
          description: '',
        },
        {
          name: 'RADIO_FAILSAFE',
          unit: '3',
          description: '',
        },
        {
          name: 'BATTERY_FAILSAFE',
          unit: '4',
          description: '',
        },
        {
          name: 'GCS_FAILSAFE',
          unit: '5',
          description: '',
        },
        {
          name: 'EKF_FAILSAFE',
          unit: '6',
          description: '',
        },
        {
          name: 'GPS_GLITCH',
          unit: '7',
          description: '',
        },
        {
          name: 'MISSION_END',
          unit: '8',
          description: '',
        },
        {
          name: 'THROTTLE_LAND_ESCAPE',
          unit: '9',
          description: '',
        },
        {
          name: 'FENCE_BREACHED',
          unit: '10',
          description: '',
        },
        {
          name: 'TERRAIN_FAILSAFE',
          unit: '11',
          description: '',
        },
        {
          name: 'BRAKE_TIMEOUT',
          unit: '12',
          description: '',
        },
        {
          name: 'FLIP_COMPLETE',
          unit: '13',
          description: '',
        },
        {
          name: 'AVOIDANCE',
          unit: '14',
          description: '',
        },
        {
          name: 'AVOIDANCE_RECOVERY',
          unit: '15',
          description: '',
        },
        {
          name: 'THROW_COMPLETE',
          unit: '16',
          description: '',
        },
        {
          name: 'TERMINATE',
          unit: '17',
          description: '',
        },
        {
          name: 'TOY_MODE',
          unit: '18',
          description: '',
        },
        {
          name: 'CRASH_FAILSAFE',
          unit: '19',
          description: '',
        },
        {
          name: 'SOARING_FBW_B_WITH_MOTOR_RUNNING',
          unit: '20',
          description: '',
        },
        {
          name: 'SOARING_THERMAL_DETECTED',
          unit: '21',
          description: '',
        },
        {
          name: 'SOARING_THERMAL_ESTIMATE_DETERIORATED',
          unit: '22',
          description: '',
        },
        {
          name: 'VTOL_FAILED_TRANSITION',
          unit: '23',
          description: '',
        },
        {
          name: 'VTOL_FAILED_TAKEOFF',
          unit: '24',
          description: '',
        },
        {
          name: 'FAILSAFE',
          unit: '25',
          description: 'general failsafes, prefer specific failsafes over this as much as possible',
        },
        {
          name: 'INITIALISED',
          unit: '26',
          description: '',
        },
        {
          name: 'SURFACE_COMPLETE',
          unit: '27',
          description: '',
        },
        {
          name: 'BAD_DEPTH',
          unit: '28',
          description: '',
        },
        {
          name: 'LEAK_FAILSAFE',
          unit: '29',
          description: '',
        },
        {
          name: 'SERVOTEST',
          unit: '30',
          description: '',
        },
        {
          name: 'STARTUP',
          unit: '31',
          description: '',
        },
        {
          name: 'SCRIPTING',
          unit: '32',
          description: '',
        },
        {
          name: 'UNAVAILABLE',
          unit: '33',
          description: '',
        },
        {
          name: 'AUTOROTATION_START',
          unit: '34',
          description: '',
        },
        {
          name: 'AUTOROTATION_BAILOUT',
          unit: '35',
          description: '',
        },
        {
          name: 'SOARING_ALT_TOO_HIGH',
          unit: '36',
          description: '',
        },
        {
          name: 'SOARING_ALT_TOO_LOW',
          unit: '37',
          description: '',
        },
        {
          name: 'SOARING_DRIFT_EXCEEDED',
          unit: '38',
          description: '',
        },
        {
          name: 'RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL',
          unit: '39',
          description: '',
        },
        {
          name: 'RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND',
          unit: '40',
          description: '',
        },
        {
          name: 'MISSION_CMD',
          unit: '41',
          description: '',
        },
        {
          name: 'FRSKY_COMMAND',
          unit: '42',
          description: '',
        },
        {
          name: 'FENCE_RETURN_PREVIOUS_MODE',
          unit: '43',
          description: '',
        },
        {
          name: 'QRTL_INSTEAD_OF_RTL',
          unit: '44',
          description: '',
        },
        {
          name: 'AUTO_RTL_EXIT',
          unit: '45',
          description: '',
        },
        {
          name: 'LOITER_ALT_REACHED_QLAND',
          unit: '46',
          description: '',
        },
        {
          name: 'LOITER_ALT_IN_VTOL',
          unit: '47',
          description: '',
        },
        {
          name: 'RADIO_FAILSAFE_RECOVERY',
          unit: '48',
          description: '',
        },
        {
          name: 'QLAND_INSTEAD_OF_RTL',
          unit: '49',
          description: '',
        },
        {
          name: 'DEADRECKON_FAILSAFE',
          unit: '50',
          description: '',
        },
        {
          name: 'MODE_TAKEOFF_FAILSAFE',
          unit: '51',
          description: '',
        },
        {
          name: 'DDS_COMMAND',
          unit: '52',
          description: '',
        },
        {
          name: 'AUX_FUNCTION',
          unit: '53',
          description: '',
        },
        {
          name: 'FIXED_WING_AUTOLAND',
          unit: '54',
          description: '',
        },
        {
          name: 'FENCE_REENABLE',
          unit: '55',
          description: '',
        },
      ],
      [
        {
          name: 'UNKNOWN',
          unit: '0',
          description: '',
        },
        {
          name: 'RC_COMMAND',
          unit: '1',
          description: '',
        },
        {
          name: 'GCS_COMMAND',
          unit: '2',
          description: '',
        },
        {
          name: 'RADIO_FAILSAFE',
          unit: '3',
          description: '',
        },
        {
          name: 'BATTERY_FAILSAFE',
          unit: '4',
          description: '',
        },
        {
          name: 'GCS_FAILSAFE',
          unit: '5',
          description: '',
        },
        {
          name: 'EKF_FAILSAFE',
          unit: '6',
          description: '',
        },
        {
          name: 'GPS_GLITCH',
          unit: '7',
          description: '',
        },
        {
          name: 'MISSION_END',
          unit: '8',
          description: '',
        },
        {
          name: 'THROTTLE_LAND_ESCAPE',
          unit: '9',
          description: '',
        },
        {
          name: 'FENCE_BREACHED',
          unit: '10',
          description: '',
        },
        {
          name: 'TERRAIN_FAILSAFE',
          unit: '11',
          description: '',
        },
        {
          name: 'BRAKE_TIMEOUT',
          unit: '12',
          description: '',
        },
        {
          name: 'FLIP_COMPLETE',
          unit: '13',
          description: '',
        },
        {
          name: 'AVOIDANCE',
          unit: '14',
          description: '',
        },
        {
          name: 'AVOIDANCE_RECOVERY',
          unit: '15',
          description: '',
        },
        {
          name: 'THROW_COMPLETE',
          unit: '16',
          description: '',
        },
        {
          name: 'TERMINATE',
          unit: '17',
          description: '',
        },
        {
          name: 'TOY_MODE',
          unit: '18',
          description: '',
        },
        {
          name: 'CRASH_FAILSAFE',
          unit: '19',
          description: '',
        },
        {
          name: 'SOARING_FBW_B_WITH_MOTOR_RUNNING',
          unit: '20',
          description: '',
        },
        {
          name: 'SOARING_THERMAL_DETECTED',
          unit: '21',
          description: '',
        },
        {
          name: 'SOARING_THERMAL_ESTIMATE_DETERIORATED',
          unit: '22',
          description: '',
        },
        {
          name: 'VTOL_FAILED_TRANSITION',
          unit: '23',
          description: '',
        },
        {
          name: 'VTOL_FAILED_TAKEOFF',
          unit: '24',
          description: '',
        },
        {
          name: 'FAILSAFE',
          unit: '25',
          description: 'general failsafes, prefer specific failsafes over this as much as possible',
        },
        {
          name: 'INITIALISED',
          unit: '26',
          description: '',
        },
        {
          name: 'SURFACE_COMPLETE',
          unit: '27',
          description: '',
        },
        {
          name: 'BAD_DEPTH',
          unit: '28',
          description: '',
        },
        {
          name: 'LEAK_FAILSAFE',
          unit: '29',
          description: '',
        },
        {
          name: 'SERVOTEST',
          unit: '30',
          description: '',
        },
        {
          name: 'STARTUP',
          unit: '31',
          description: '',
        },
        {
          name: 'SCRIPTING',
          unit: '32',
          description: '',
        },
        {
          name: 'UNAVAILABLE',
          unit: '33',
          description: '',
        },
        {
          name: 'AUTOROTATION_START',
          unit: '34',
          description: '',
        },
        {
          name: 'AUTOROTATION_BAILOUT',
          unit: '35',
          description: '',
        },
        {
          name: 'SOARING_ALT_TOO_HIGH',
          unit: '36',
          description: '',
        },
        {
          name: 'SOARING_ALT_TOO_LOW',
          unit: '37',
          description: '',
        },
        {
          name: 'SOARING_DRIFT_EXCEEDED',
          unit: '38',
          description: '',
        },
        {
          name: 'RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL',
          unit: '39',
          description: '',
        },
        {
          name: 'RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND',
          unit: '40',
          description: '',
        },
        {
          name: 'MISSION_CMD',
          unit: '41',
          description: '',
        },
        {
          name: 'FRSKY_COMMAND',
          unit: '42',
          description: '',
        },
        {
          name: 'FENCE_RETURN_PREVIOUS_MODE',
          unit: '43',
          description: '',
        },
        {
          name: 'QRTL_INSTEAD_OF_RTL',
          unit: '44',
          description: '',
        },
        {
          name: 'AUTO_RTL_EXIT',
          unit: '45',
          description: '',
        },
        {
          name: 'LOITER_ALT_REACHED_QLAND',
          unit: '46',
          description: '',
        },
        {
          name: 'LOITER_ALT_IN_VTOL',
          unit: '47',
          description: '',
        },
        {
          name: 'RADIO_FAILSAFE_RECOVERY',
          unit: '48',
          description: '',
        },
        {
          name: 'QLAND_INSTEAD_OF_RTL',
          unit: '49',
          description: '',
        },
        {
          name: 'DEADRECKON_FAILSAFE',
          unit: '50',
          description: '',
        },
        {
          name: 'MODE_TAKEOFF_FAILSAFE',
          unit: '51',
          description: '',
        },
        {
          name: 'DDS_COMMAND',
          unit: '52',
          description: '',
        },
        {
          name: 'AUX_FUNCTION',
          unit: '53',
          description: '',
        },
        {
          name: 'FIXED_WING_AUTOLAND',
          unit: '54',
          description: '',
        },
        {
          name: 'FENCE_REENABLE',
          unit: '55',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'mon',
    title: 'MON',
    description: 'Main loop performance monitoring message.',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Dly',
          unit: 'Loop delay so far',
          description: '',
        },
        {
          name: 'Tsk',
          unit: 'Current task',
          description: '',
        },
        {
          name: 'IErr',
          unit: 'Internal error mask',
          description: '',
        },
        {
          name: 'IErrCnt',
          unit: 'Count of internal error occurances',
          description: '',
        },
        {
          name: 'IErrLn',
          unit: 'Internal Error line',
          description: '',
        },
        {
          name: 'MM',
          unit: 'MAVLink message currently being processed',
          description: '',
        },
        {
          name: 'MC',
          unit: 'MAVLink command currently being processed',
          description: '',
        },
        {
          name: 'SmLn',
          unit: 'If semaphore taken, line of semaphore take call',
          description: '',
        },
        {
          name: 'SPICnt',
          unit: 'count of SPI transactions',
          description: '',
        },
        {
          name: 'I2CCnt',
          unit: 'count of i2c transactions',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'motb',
    title: 'MOTB',
    description: 'Motor mixer information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'LiftMax',
          unit: 'Maximum motor compensation gain',
          description: '',
        },
        {
          name: 'BatVolt',
          unit: 'Ratio between detected battery voltage and maximum battery voltage',
          description: '',
        },
        {
          name: 'ThLimit',
          unit: 'Throttle limit set due to battery current limitations',
          description: '',
        },
        {
          name: 'ThrAvMx',
          unit:
            'Maximum average throttle that can be used to maintain attitude control, derived from throttle mix params',
          description: '',
        },
        {
          name: 'ThrOut',
          unit: 'Throttle output',
          description: '',
        },
        {
          name: 'FailFlags',
          unit: 'bit 0 motor failed, bit 1 motors balanced, should be 2 in normal flight',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'msg',
    title: 'MSG',
    description: 'Textual messages',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Message',
          unit: 'char[64]',
          description: 'message text',
        },
      ],
    ],
  },
  {
    id: 'mult',
    title: 'MULT',
    description: 'Message mapping from single character to numeric multiplier',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Id',
          unit: 'character referenced by FMTU',
          description: '',
        },
        {
          name: 'Mult',
          unit: 'numeric multiplier',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'nkf0',
    title: 'NKF0',
    description: 'EKF2 beacon sensor diagnostics',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF2 core this data is for',
        },
        {
          name: 'ID',
          unit: 'Beacon sensor ID',
          description: '',
        },
        {
          name: 'rng',
          unit: 'm',
          description: 'Beacon range',
        },
        {
          name: 'innov',
          unit: 'Beacon range innovation',
          description: '',
        },
        {
          name: 'SIV',
          unit: 'sqrt of beacon range innovation variance',
          description: '',
        },
        {
          name: 'TR',
          unit: 'Beacon range innovation consistency test ratio',
          description: '',
        },
        {
          name: 'BPN',
          unit: 'm',
          description: 'Beacon north position',
        },
        {
          name: 'BPE',
          unit: 'm',
          description: 'Beacon east position',
        },
        {
          name: 'BPD',
          unit: 'm',
          description: 'Beacon down position',
        },
        {
          name: 'OFH',
          unit: 'm',
          description: 'High estimate of vertical position offset of beacons rel to EKF origin',
        },
        {
          name: 'OFL',
          unit: 'm',
          description: 'Low estimate of vertical position offset of beacons rel to EKF origin',
        },
        {
          name: 'OFN',
          unit: 'm',
          description: 'always zero',
        },
        {
          name: 'OFE',
          unit: 'm',
          description: 'always zero',
        },
        {
          name: 'OFD',
          unit: 'm',
          description: 'always zero',
        },
      ],
    ],
  },
  {
    id: 'nkf1',
    title: 'NKF1',
    description: 'EKF2 estimator outputs',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF2 core this data is for',
        },
        {
          name: 'Roll',
          unit: 'deg',
          description: 'Estimated roll',
        },
        {
          name: 'Pitch',
          unit: 'deg',
          description: 'Estimated pitch',
        },
        {
          name: 'Yaw',
          unit: 'degheading',
          description: 'Estimated yaw',
        },
        {
          name: 'VN',
          unit: 'm/s',
          description: 'Estimated velocity (North component)',
        },
        {
          name: 'VE',
          unit: 'm/s',
          description: 'Estimated velocity (East component)',
        },
        {
          name: 'VD',
          unit: 'm/s',
          description: 'Estimated velocity (Down component)',
        },
        {
          name: 'dPD',
          unit: 'm/s',
          description: 'Filtered derivative of vertical position (down)',
        },
        {
          name: 'PN',
          unit: 'm',
          description: 'Estimated distance from origin (North component)',
        },
        {
          name: 'PE',
          unit: 'm',
          description: 'Estimated distance from origin (East component)',
        },
        {
          name: 'PD',
          unit: 'm',
          description: 'Estimated distance from origin (Down component)',
        },
        {
          name: 'GX',
          unit: 'deg/s',
          description: 'Estimated gyro bias, X axis',
        },
        {
          name: 'GY',
          unit: 'deg/s',
          description: 'Estimated gyro bias, Y axis',
        },
        {
          name: 'GZ',
          unit: 'deg/s',
          description: 'Estimated gyro bias, Z axis',
        },
        {
          name: 'OH',
          unit: 'm',
          description: 'Height of origin above WGS-84',
        },
      ],
    ],
  },
  {
    id: 'nkf2',
    title: 'NKF2',
    description: 'EKF2 estimator secondary outputs',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF2 core this data is for',
        },
        {
          name: 'AZbias',
          unit: 'Estimated accelerometer Z bias',
          description: '',
        },
        {
          name: 'GSX',
          unit: 'Gyro Scale Factor (X-axis)',
          description: '',
        },
        {
          name: 'GSY',
          unit: 'Gyro Scale Factor (Y-axis)',
          description: '',
        },
        {
          name: 'GSZ',
          unit: 'Gyro Scale Factor (Z-axis)',
          description: '',
        },
        {
          name: 'VWN',
          unit: 'm/s',
          description: 'Estimated wind velocity (moving-to-North component)',
        },
        {
          name: 'VWE',
          unit: 'm/s',
          description: 'Estimated wind velocity (moving-to-East component)',
        },
        {
          name: 'MN',
          unit: 'mGauss',
          description: 'Magnetic field strength (North component)',
        },
        {
          name: 'ME',
          unit: 'mGauss',
          description: 'Magnetic field strength (East component)',
        },
        {
          name: 'MD',
          unit: 'mGauss',
          description: 'Magnetic field strength (Down component)',
        },
        {
          name: 'MX',
          unit: 'mGauss',
          description: 'Magnetic field strength (body X-axis)',
        },
        {
          name: 'MY',
          unit: 'mGauss',
          description: 'Magnetic field strength (body Y-axis)',
        },
        {
          name: 'MZ',
          unit: 'mGauss',
          description: 'Magnetic field strength (body Z-axis)',
        },
        {
          name: 'MI',
          unit: 'Magnetometer used for data',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'nkf3',
    title: 'NKF3',
    description: 'EKF2 innovations',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF2 core this data is for',
        },
        {
          name: 'IVN',
          unit: 'm/s',
          description: 'Innovation in velocity (North component)',
        },
        {
          name: 'IVE',
          unit: 'm/s',
          description: 'Innovation in velocity (East component)',
        },
        {
          name: 'IVD',
          unit: 'm/s',
          description: 'Innovation in velocity (Down component)',
        },
        {
          name: 'IPN',
          unit: 'm',
          description: 'Innovation in position (North component)',
        },
        {
          name: 'IPE',
          unit: 'm',
          description: 'Innovation in position (East component)',
        },
        {
          name: 'IPD',
          unit: 'm',
          description: 'Innovation in position (Down component)',
        },
        {
          name: 'IMX',
          unit: 'mGauss',
          description: 'Innovation in magnetic field strength (X-axis component)',
        },
        {
          name: 'IMY',
          unit: 'mGauss',
          description: 'Innovation in magnetic field strength (Y-axis component)',
        },
        {
          name: 'IMZ',
          unit: 'mGauss',
          description: 'Innovation in magnetic field strength (Z-axis component)',
        },
        {
          name: 'IYAW',
          unit: 'deg',
          description: 'Innovation in vehicle yaw',
        },
        {
          name: 'IVT',
          unit: 'UNKNOWN',
          description: 'Innovation in true-airspeed',
        },
        {
          name: 'RErr',
          unit: 'Accumulated relative error of this core with respect to active primary core',
          description: '',
        },
        {
          name: 'ErSc',
          unit: 'A consolidated error score where higher numbers are less healthy',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'nkf4',
    title: 'NKF4',
    description:
      'EKF2 variances SV, SP, SH and SM are probably best described as ‘Squared Innovation Test Ratios’ where values <1 tells us the measurement was accepted and >1 tells us it was rejected. They represent the square of the (innovation / maximum allowed innovation) where the innovation is the difference between predicted and measured value and the maximum allowed innovation is determined from the uncertainty of the measurement, uncertainty of the prediction and scaled using the number of standard deviations set by the innovation gate parameter for that measurement, eg EK2_MAG_I_GATE, EK2_HGT_I_GATE, etc',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF2 core this data is for',
        },
        {
          name: 'SV',
          unit: 'Square root of the velocity variance',
          description: '',
        },
        {
          name: 'SP',
          unit: 'Square root of the position variance',
          description: '',
        },
        {
          name: 'SH',
          unit: 'Square root of the height variance',
          description: '',
        },
        {
          name: 'SM',
          unit: 'Magnetic field variance',
          description: '',
        },
        {
          name: 'SVT',
          unit: 'tilt error convergence metric',
          description: '',
        },
        {
          name: 'errRP',
          unit: 'Filtered error in roll/pitch estimate',
          description: '',
        },
        {
          name: 'OFN',
          unit: 'm',
          description: 'Most recent position reset (North component)',
        },
        {
          name: 'OFE',
          unit: 'm',
          description: 'Most recent position reset (East component)',
        },
        {
          name: 'FS',
          unit: 'Filter fault status',
          description: '',
        },
        {
          name: 'TS',
          unit:
            'Filter timeout status bitmask (0:position measurement, 1:velocity measurement, 2:height measurement, 3:magnetometer measurement, 4:airspeed measurement)',
          description: '',
        },
        {
          name: 'SS',
          unit: 'bitmask',
          description: 'Filter solution status Bitmask values:',
        },
        {
          name: 'ATTITUDE_VALID',
          unit: '1',
          description: 'attitude estimate valid',
        },
        {
          name: 'HORIZ_VEL',
          unit: '2',
          description: 'horizontal velocity estimate valid',
        },
        {
          name: 'VERT_VEL',
          unit: '4',
          description: 'vertical velocity estimate valid',
        },
        {
          name: 'HORIZ_POS_REL',
          unit: '8',
          description: 'relative horizontal position estimate valid',
        },
        {
          name: 'HORIZ_POS_ABS',
          unit: '16',
          description: 'absolute horizontal position estimate valid',
        },
        {
          name: 'VERT_POS',
          unit: '32',
          description: 'vertical position estimate valid',
        },
        {
          name: 'TERRAIN_ALT',
          unit: '64',
          description: 'terrain height estimate valid',
        },
        {
          name: 'CONST_POS_MODE',
          unit: '128',
          description: 'in constant position mode',
        },
        {
          name: 'PRED_HORIZ_POS_REL',
          unit: '256',
          description: 'expected good relative horizontal position estimate - used before takeoff',
        },
        {
          name: 'PRED_HORIZ_POS_ABS',
          unit: '512',
          description: 'expected good absolute horizontal position estimate - used before takeoff',
        },
        {
          name: 'TAKEOFF_DETECTED',
          unit: '1024',
          description: 'optical flow takeoff has been detected',
        },
        {
          name: 'TAKEOFF_EXPECTED',
          unit: '2048',
          description: 'compensating for baro errors during takeoff',
        },
        {
          name: 'TOUCHDOWN_EXPECTED',
          unit: '4096',
          description: 'compensating for baro errors during touchdown',
        },
        {
          name: 'USING_GPS',
          unit: '8192',
          description: 'using GPS position',
        },
        {
          name: 'GPS_GLITCHING',
          unit: '16384',
          description: 'GPS glitching is affecting navigation accuracy',
        },
        {
          name: 'GPS_QUALITY_GOOD',
          unit: '32768',
          description: 'can use GPS for navigation',
        },
        {
          name: 'INITALIZED',
          unit: '65536',
          description: 'has ever been healthy',
        },
        {
          name: 'REJECTING_AIRSPEED',
          unit: '131072',
          description: 'rejecting airspeed data',
        },
        {
          name: 'DEAD_RECKONING',
          unit: '262144',
          description: 'dead reckoning (e.g. no position or velocity source)',
        },
        {
          name: 'GPS',
          unit: 'Filter GPS status',
          description: '',
        },
        {
          name: 'PI',
          unit: 'Primary core index',
          description: '',
        },
      ],
      [
        {
          name: 'ATTITUDE_VALID',
          unit: '1',
          description: 'attitude estimate valid',
        },
        {
          name: 'HORIZ_VEL',
          unit: '2',
          description: 'horizontal velocity estimate valid',
        },
        {
          name: 'VERT_VEL',
          unit: '4',
          description: 'vertical velocity estimate valid',
        },
        {
          name: 'HORIZ_POS_REL',
          unit: '8',
          description: 'relative horizontal position estimate valid',
        },
        {
          name: 'HORIZ_POS_ABS',
          unit: '16',
          description: 'absolute horizontal position estimate valid',
        },
        {
          name: 'VERT_POS',
          unit: '32',
          description: 'vertical position estimate valid',
        },
        {
          name: 'TERRAIN_ALT',
          unit: '64',
          description: 'terrain height estimate valid',
        },
        {
          name: 'CONST_POS_MODE',
          unit: '128',
          description: 'in constant position mode',
        },
        {
          name: 'PRED_HORIZ_POS_REL',
          unit: '256',
          description: 'expected good relative horizontal position estimate - used before takeoff',
        },
        {
          name: 'PRED_HORIZ_POS_ABS',
          unit: '512',
          description: 'expected good absolute horizontal position estimate - used before takeoff',
        },
        {
          name: 'TAKEOFF_DETECTED',
          unit: '1024',
          description: 'optical flow takeoff has been detected',
        },
        {
          name: 'TAKEOFF_EXPECTED',
          unit: '2048',
          description: 'compensating for baro errors during takeoff',
        },
        {
          name: 'TOUCHDOWN_EXPECTED',
          unit: '4096',
          description: 'compensating for baro errors during touchdown',
        },
        {
          name: 'USING_GPS',
          unit: '8192',
          description: 'using GPS position',
        },
        {
          name: 'GPS_GLITCHING',
          unit: '16384',
          description: 'GPS glitching is affecting navigation accuracy',
        },
        {
          name: 'GPS_QUALITY_GOOD',
          unit: '32768',
          description: 'can use GPS for navigation',
        },
        {
          name: 'INITALIZED',
          unit: '65536',
          description: 'has ever been healthy',
        },
        {
          name: 'REJECTING_AIRSPEED',
          unit: '131072',
          description: 'rejecting airspeed data',
        },
        {
          name: 'DEAD_RECKONING',
          unit: '262144',
          description: 'dead reckoning (e.g. no position or velocity source)',
        },
      ],
    ],
  },
  {
    id: 'nkf5',
    title: 'NKF5',
    description: 'EKF2 Sensor innovations (primary core) and general dumping ground',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF2 core this data is for',
        },
        {
          name: 'NI',
          unit: 'Normalised flow variance',
          description: '',
        },
        {
          name: 'FIX',
          unit: 'Optical flow LOS rate vector innovations from the main nav filter (X-axis)',
          description: '',
        },
        {
          name: 'FIY',
          unit: 'Optical flow LOS rate vector innovations from the main nav filter (Y-axis)',
          description: '',
        },
        {
          name: 'AFI',
          unit: 'Optical flow LOS rate innovation from terrain offset estimator',
          description: '',
        },
        {
          name: 'HAGL',
          unit: 'm',
          description: 'Height above ground level',
        },
        {
          name: 'offset',
          unit: 'UNKNOWN',
          description:
            'Estimated vertical position of the terrain relative to the nav filter zero datum',
        },
        {
          name: 'RI',
          unit: 'UNKNOWN',
          description: 'Range finder innovations',
        },
        {
          name: 'rng',
          unit: 'UNKNOWN',
          description: 'Measured range',
        },
        {
          name: 'Herr',
          unit: 'm',
          description: 'Filter ground offset state error',
        },
        {
          name: 'eAng',
          unit: 'rad',
          description: 'Magnitude of angular error',
        },
        {
          name: 'eVel',
          unit: 'm/s',
          description: 'Magnitude of velocity error',
        },
        {
          name: 'ePos',
          unit: 'm',
          description: 'Magnitude of position error',
        },
      ],
    ],
  },
  {
    id: 'nkq',
    title: 'NKQ',
    description: 'EKF2 quaternion defining the rotation from NED to XYZ (autopilot) axes',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF2 core this data is for',
        },
        {
          name: 'Q1',
          unit: 'Quaternion a term',
          description: '',
        },
        {
          name: 'Q2',
          unit: 'Quaternion b term',
          description: '',
        },
        {
          name: 'Q3',
          unit: 'Quaternion c term',
          description: '',
        },
        {
          name: 'Q4',
          unit: 'Quaternion d term',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'nkt',
    title: 'NKT',
    description: 'EKF2 timing information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF core this message instance applies to',
        },
        {
          name: 'Cnt',
          unit: 's',
          description: 'count of samples used to create this message',
        },
        {
          name: 'IMUMin',
          unit: 's',
          description: 'smallest IMU sample interval',
        },
        {
          name: 'IMUMax',
          unit: 's',
          description: 'largest IMU sample interval',
        },
        {
          name: 'EKFMin',
          unit: 's',
          description: 'low-passed achieved average time step rate for the EKF (minimum)',
        },
        {
          name: 'EKFMax',
          unit: 's',
          description: 'low-passed achieved average time step rate for the EKF (maximum)',
        },
        {
          name: 'AngMin',
          unit: 's',
          description: 'accumulated measurement time interval for the delta angle (minimum)',
        },
        {
          name: 'AngMax',
          unit: 's',
          description: 'accumulated measurement time interval for the delta angle (maximum)',
        },
        {
          name: 'VMin',
          unit: 's',
          description: 'accumulated measurement time interval for the delta velocity (minimum)',
        },
        {
          name: 'VMax',
          unit: 's',
          description: 'accumulated measurement time interval for the delta velocity (maximum)',
        },
      ],
    ],
  },
  {
    id: 'nky0',
    title: 'NKY0',
    description: 'EKF Yaw Estimator States',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF core this data is for',
        },
        {
          name: 'YC',
          unit: 'degheading',
          description: 'GSF yaw estimate (deg)',
        },
        {
          name: 'YCS',
          unit: 'deg',
          description: 'GSF yaw estimate 1-Sigma uncertainty (deg)',
        },
        {
          name: 'Y0',
          unit: 'degheading',
          description: 'Yaw estimate from individual EKF filter 0 (deg)',
        },
        {
          name: 'Y1',
          unit: 'degheading',
          description: 'Yaw estimate from individual EKF filter 1 (deg)',
        },
        {
          name: 'Y2',
          unit: 'degheading',
          description: 'Yaw estimate from individual EKF filter 2 (deg)',
        },
        {
          name: 'Y3',
          unit: 'degheading',
          description: 'Yaw estimate from individual EKF filter 3 (deg)',
        },
        {
          name: 'Y4',
          unit: 'degheading',
          description: 'Yaw estimate from individual EKF filter 4 (deg)',
        },
        {
          name: 'W0',
          unit: 'Weighting applied to yaw estimate from individual EKF filter 0',
          description: '',
        },
        {
          name: 'W1',
          unit: 'Weighting applied to yaw estimate from individual EKF filter 1',
          description: '',
        },
        {
          name: 'W2',
          unit: 'Weighting applied to yaw estimate from individual EKF filter 2',
          description: '',
        },
        {
          name: 'W3',
          unit: 'Weighting applied to yaw estimate from individual EKF filter 3',
          description: '',
        },
        {
          name: 'W4',
          unit: 'Weighting applied to yaw estimate from individual EKF filter 4',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'nky1',
    title: 'NKY1',
    description: 'EKF Yaw Estimator Innovations',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF core this data is for',
        },
        {
          name: 'IVN0',
          unit: 'm/s',
          description: 'North velocity innovation from individual EKF filter 0 (m/s)',
        },
        {
          name: 'IVN1',
          unit: 'm/s',
          description: 'North velocity innovation from individual EKF filter 1 (m/s)',
        },
        {
          name: 'IVN2',
          unit: 'm/s',
          description: 'North velocity innovation from individual EKF filter 2 (m/s)',
        },
        {
          name: 'IVN3',
          unit: 'm/s',
          description: 'North velocity innovation from individual EKF filter 3 (m/s)',
        },
        {
          name: 'IVN4',
          unit: 'm/s',
          description: 'North velocity innovation from individual EKF filter 4 (m/s)',
        },
        {
          name: 'IVE0',
          unit: 'm/s',
          description: 'East velocity innovation from individual EKF filter 0 (m/s)',
        },
        {
          name: 'IVE1',
          unit: 'm/s',
          description: 'East velocity innovation from individual EKF filter 1 (m/s)',
        },
        {
          name: 'IVE2',
          unit: 'm/s',
          description: 'East velocity innovation from individual EKF filter 2 (m/s)',
        },
        {
          name: 'IVE3',
          unit: 'm/s',
          description: 'East velocity innovation from individual EKF filter 3 (m/s)',
        },
        {
          name: 'IVE4',
          unit: 'm/s',
          description: 'East velocity innovation from individual EKF filter 4 (m/s)',
        },
      ],
    ],
  },
  {
    id: 'ntun',
    title: 'NTUN',
    description: 'Navigation Tuning information - e.g. vehicle destination',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Dist',
          unit: 'm',
          description: 'distance to the current navigation waypoint',
        },
        {
          name: 'TBrg',
          unit: 'deg',
          description: 'bearing to the current navigation waypoint',
        },
        {
          name: 'NavBrg',
          unit: 'deg',
          description: 'the vehicle’s desired heading',
        },
        {
          name: 'AltE',
          unit: 'm',
          description: 'difference between current vehicle height and target height',
        },
        {
          name: 'XT',
          unit: 'm',
          description: 'the vehicle’s current distance from the current travel segment',
        },
        {
          name: 'XTi',
          unit: 'cm',
          description: 'integration of the vehicle’s crosstrack error',
        },
        {
          name: 'AsE',
          unit: 'm/s',
          description: 'difference between vehicle’s airspeed and desired airspeed',
        },
        {
          name: 'TLat',
          unit: 'deglatitude',
          description: 'target latitude',
        },
        {
          name: 'TLng',
          unit: 'deglongitude',
          description: 'target longitude',
        },
        {
          name: 'TAW',
          unit: 'm',
          description: 'target altitude WP',
        },
        {
          name: 'TAT',
          unit: 'm',
          description: 'target altitude TECS',
        },
        {
          name: 'TAsp',
          unit: 'm/s',
          description: 'target airspeed',
        },
      ],
    ],
  },
  {
    id: 'nvf',
    title: 'NVF',
    description: 'Named Value Float messages; messages sent to GCS via NAMED_VALUE_FLOAT',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Name',
          unit: 'instance',
          description: 'Name of float',
        },
        {
          name: 'Value',
          unit: 'Value of float',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'nvs',
    title: 'NVS',
    description: 'Named Value String messages; messages sent to GCS via NAMED_VALUE_STRING',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Name',
          unit: 'instance',
          description: 'Name of string',
        },
        {
          name: 'Value',
          unit: 'char[64]',
          description: 'Value of string',
        },
      ],
    ],
  },
  {
    id: 'oabr',
    title: 'OABR',
    description: 'Object avoidance (Bendy Ruler) diagnostics',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Type',
          unit: 'Type of BendyRuler currently active',
          description: '',
        },
        {
          name: 'Act',
          unit: 'True if Bendy Ruler avoidance is being used',
          description: '',
        },
        {
          name: 'DYaw',
          unit: 'deg',
          description: 'Best yaw chosen to avoid obstacle',
        },
        {
          name: 'Yaw',
          unit: 'deg',
          description: 'Current vehicle yaw',
        },
        {
          name: 'DP',
          unit: 'deg',
          description: 'Desired pitch chosen to avoid obstacle',
        },
        {
          name: 'RChg',
          unit:
            'True if BendyRuler resisted changing bearing and continued in last calculated bearing',
          description: '',
        },
        {
          name: 'Mar',
          unit: 'm',
          description: 'Margin from path to obstacle on best yaw chosen',
        },
        {
          name: 'DLt',
          unit: 'deglatitude',
          description: 'Destination latitude',
        },
        {
          name: 'DLg',
          unit: 'deglongitude',
          description: 'Destination longitude',
        },
        {
          name: 'DAlt',
          unit: 'm',
          description: 'Desired alt above EKF Origin',
        },
        {
          name: 'OLt',
          unit: 'deglatitude',
          description: 'Intermediate location chosen for avoidance',
        },
        {
          name: 'OLg',
          unit: 'deglongitude',
          description: 'Intermediate location chosen for avoidance',
        },
        {
          name: 'OAlt',
          unit: 'm',
          description: 'Intermediate alt chosen for avoidance above EKF origin',
        },
      ],
    ],
  },
  {
    id: 'oadj',
    title: 'OADJ',
    description: 'Object avoidance (Dijkstra) diagnostics',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'State',
          unit: 'Dijkstra avoidance library state',
          description: '',
        },
        {
          name: 'Err',
          unit: 'Dijkstra library error condition',
          description: '',
        },
        {
          name: 'CurrPoint',
          unit: 'Destination point in calculated path to destination',
          description: '',
        },
        {
          name: 'TotPoints',
          unit: 'Number of points in path to destination',
          description: '',
        },
        {
          name: 'DLat',
          unit: 'deglatitude',
          description: 'Destination latitude',
        },
        {
          name: 'DLng',
          unit: 'deglongitude',
          description: 'Destination longitude',
        },
        {
          name: 'OALat',
          unit: 'deglatitude',
          description: 'Object Avoidance chosen destination point latitude',
        },
        {
          name: 'OALng',
          unit: 'deglongitude',
          description: 'Object Avoidance chosen destination point longitude',
        },
      ],
    ],
  },
  {
    id: 'oavg',
    title: 'OAVG',
    description: 'Object avoidance path planning visgraph points',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'version',
          unit: 'Visgraph version, increments each time the visgraph is re-generated',
          description: '',
        },
        {
          name: 'point_num',
          unit: 'point number in visgraph',
          description: '',
        },
        {
          name: 'Lat',
          unit: 'deglatitude',
          description: 'Latitude',
        },
        {
          name: 'Lon',
          unit: 'deglongitude',
          description: 'longitude',
        },
      ],
    ],
  },
  {
    id: 'of',
    title: 'OF',
    description: 'Optical flow sensor data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Qual',
          unit: 'Estimated sensor data quality',
          description: '',
        },
        {
          name: 'flowX',
          unit: 'rad/s',
          description: 'Sensor flow rate, X-axis',
        },
        {
          name: 'flowY',
          unit: 'rad/s',
          description: 'Sensor flow rate,Y-axis',
        },
        {
          name: 'bodyX',
          unit: 'rad/s',
          description: 'derived rotational velocity, X-axis',
        },
        {
          name: 'bodyY',
          unit: 'rad/s',
          description: 'derived rotational velocity, Y-axis',
        },
      ],
    ],
  },
  {
    id: 'ofca',
    title: 'OFCA',
    description: 'Optical Flow Calibration sample',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Axis',
          unit: 'instance',
          description: 'Axis (X=0 Y=1)',
        },
        {
          name: 'Num',
          unit: 'Sample number',
          description: '',
        },
        {
          name: 'FRate',
          unit: 'rad/s',
          description: 'Flow rate',
        },
        {
          name: 'BRate',
          unit: 'rad/s',
          description: 'Body rate',
        },
        {
          name: 'LPred',
          unit: 'rad/s',
          description: 'Los pred',
        },
      ],
    ],
  },
  {
    id: 'ofg',
    title: 'OFG',
    description:
      'OFfboard-Guided - an advanced version of GUIDED for companion computers that includes rate/s.',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Arsp',
          unit: 'm/s',
          description: 'target airspeed cm',
        },
        {
          name: 'ArspA',
          unit: 'm/s',
          description: 'target airspeed accel',
        },
        {
          name: 'Alt',
          unit: 'm',
          description: 'target alt',
        },
        {
          name: 'AltA',
          unit: 'm/s/s',
          description: 'target alt velocity (rate of change)',
        },
        {
          name: 'AltF',
          unit: 'target alt frame (MAVLink)',
          description: '',
        },
        {
          name: 'Hdg',
          unit: 'deg',
          description: 'target heading',
        },
        {
          name: 'HdgA',
          unit: 'target heading lim',
          description: '',
        },
        {
          name: 'AltL',
          unit: 'target alt frame (Location)',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'orgn',
    title: 'ORGN',
    description: 'Vehicle navigation origin or other notable position',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Type',
          unit: 'instance',
          description: 'Position type Values:',
        },
        {
          name: 'ekf_origin',
          unit: '0',
          description: '',
        },
        {
          name: 'ahrs_home',
          unit: '1',
          description: '',
        },
        {
          name: 'Lat',
          unit: 'deglatitude',
          description: 'Position latitude',
        },
        {
          name: 'Lng',
          unit: 'deglongitude',
          description: 'Position longitude',
        },
        {
          name: 'Alt',
          unit: 'm',
          description: 'Position altitude',
        },
      ],
      [
        {
          name: 'ekf_origin',
          unit: '0',
          description: '',
        },
        {
          name: 'ahrs_home',
          unit: '1',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'parm',
    title: 'PARM',
    description: 'parameter value',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Name',
          unit: 'char[16]',
          description: 'parameter name',
        },
        {
          name: 'Value',
          unit: 'parameter value',
          description: '',
        },
        {
          name: 'Default',
          unit: 'default parameter value for this board and config',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'pida',
    title: 'PIDA',
    description: 'Proportional/Integral/Derivative gain values for vertical acceleration',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Tar',
          unit: 'desired value',
          description: '',
        },
        {
          name: 'Act',
          unit: 'achieved value',
          description: '',
        },
        {
          name: 'Err',
          unit: 'error between target and achieved',
          description: '',
        },
        {
          name: 'P',
          unit: 'proportional part of PID',
          description: '',
        },
        {
          name: 'I',
          unit: 'integral part of PID',
          description: '',
        },
        {
          name: 'D',
          unit: 'derivative part of PID',
          description: '',
        },
        {
          name: 'FF',
          unit: 'controller feed-forward portion of response',
          description: '',
        },
        {
          name: 'DFF',
          unit: 'controller derivative feed-forward portion of response',
          description: '',
        },
        {
          name: 'Dmod',
          unit: 'scaler applied to D gain to reduce limit cycling',
          description: '',
        },
        {
          name: 'SRate',
          unit: 'slew rate used in slew limiter',
          description: '',
        },
        {
          name: 'Flags',
          unit: 'bitmask',
          description: 'bitmask of PID state flags Bitmask values:',
        },
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
      [
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
    ],
  },
  {
    id: 'pide',
    title: 'PIDE',
    description: 'Proportional/Integral/Derivative gain values for East/West velocity',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Tar',
          unit: 'desired value',
          description: '',
        },
        {
          name: 'Act',
          unit: 'achieved value',
          description: '',
        },
        {
          name: 'Err',
          unit: 'error between target and achieved',
          description: '',
        },
        {
          name: 'P',
          unit: 'proportional part of PID',
          description: '',
        },
        {
          name: 'I',
          unit: 'integral part of PID',
          description: '',
        },
        {
          name: 'D',
          unit: 'derivative part of PID',
          description: '',
        },
        {
          name: 'FF',
          unit: 'controller feed-forward portion of response',
          description: '',
        },
        {
          name: 'DFF',
          unit: 'controller derivative feed-forward portion of response',
          description: '',
        },
        {
          name: 'Dmod',
          unit: 'scaler applied to D gain to reduce limit cycling',
          description: '',
        },
        {
          name: 'SRate',
          unit: 'slew rate used in slew limiter',
          description: '',
        },
        {
          name: 'Flags',
          unit: 'bitmask',
          description: 'bitmask of PID state flags Bitmask values:',
        },
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
      [
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
    ],
  },
  {
    id: 'pidg',
    title: 'PIDG',
    description:
      'Plane Proportional/Integral/Derivative gain values for Heading when using COMMAND_INT control.',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Tar',
          unit: 'desired value',
          description: '',
        },
        {
          name: 'Act',
          unit: 'achieved value',
          description: '',
        },
        {
          name: 'Err',
          unit: 'error between target and achieved',
          description: '',
        },
        {
          name: 'P',
          unit: 'proportional part of PID',
          description: '',
        },
        {
          name: 'I',
          unit: 'integral part of PID',
          description: '',
        },
        {
          name: 'D',
          unit: 'derivative part of PID',
          description: '',
        },
        {
          name: 'FF',
          unit: 'controller feed-forward portion of response',
          description: '',
        },
        {
          name: 'DFF',
          unit: 'controller derivative feed-forward portion of response',
          description: '',
        },
        {
          name: 'Dmod',
          unit: 'scaler applied to D gain to reduce limit cycling',
          description: '',
        },
        {
          name: 'SRate',
          unit: 'slew rate',
          description: '',
        },
        {
          name: 'Flags',
          unit: 'bitmask',
          description: 'bitmask of PID state flags Bitmask values:',
        },
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
      [
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
    ],
  },
  {
    id: 'pidn',
    title: 'PIDN',
    description: 'Proportional/Integral/Derivative gain values for North/South velocity',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Tar',
          unit: 'desired value',
          description: '',
        },
        {
          name: 'Act',
          unit: 'achieved value',
          description: '',
        },
        {
          name: 'Err',
          unit: 'error between target and achieved',
          description: '',
        },
        {
          name: 'P',
          unit: 'proportional part of PID',
          description: '',
        },
        {
          name: 'I',
          unit: 'integral part of PID',
          description: '',
        },
        {
          name: 'D',
          unit: 'derivative part of PID',
          description: '',
        },
        {
          name: 'FF',
          unit: 'controller feed-forward portion of response',
          description: '',
        },
        {
          name: 'DFF',
          unit: 'controller derivative feed-forward portion of response',
          description: '',
        },
        {
          name: 'Dmod',
          unit: 'scaler applied to D gain to reduce limit cycling',
          description: '',
        },
        {
          name: 'SRate',
          unit: 'slew rate used in slew limiter',
          description: '',
        },
        {
          name: 'Flags',
          unit: 'bitmask',
          description: 'bitmask of PID state flags Bitmask values:',
        },
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
      [
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
    ],
  },
  {
    id: 'pidp',
    title: 'PIDP',
    description: 'Proportional/Integral/Derivative gain values for Pitch rate',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Tar',
          unit: 'desired value',
          description: '',
        },
        {
          name: 'Act',
          unit: 'achieved value',
          description: '',
        },
        {
          name: 'Err',
          unit: 'error between target and achieved',
          description: '',
        },
        {
          name: 'P',
          unit: 'proportional part of PID',
          description: '',
        },
        {
          name: 'I',
          unit: 'integral part of PID',
          description: '',
        },
        {
          name: 'D',
          unit: 'derivative part of PID',
          description: '',
        },
        {
          name: 'FF',
          unit: 'controller feed-forward portion of response',
          description: '',
        },
        {
          name: 'DFF',
          unit: 'controller derivative feed-forward portion of response',
          description: '',
        },
        {
          name: 'Dmod',
          unit: 'scaler applied to D gain to reduce limit cycling',
          description: '',
        },
        {
          name: 'SRate',
          unit: 'slew rate used in slew limiter',
          description: '',
        },
        {
          name: 'Flags',
          unit: 'bitmask',
          description: 'bitmask of PID state flags Bitmask values:',
        },
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
      [
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
    ],
  },
  {
    id: 'pidr',
    title: 'PIDR',
    description: 'Proportional/Integral/Derivative gain values for Roll rate',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Tar',
          unit: 'desired value',
          description: '',
        },
        {
          name: 'Act',
          unit: 'achieved value',
          description: '',
        },
        {
          name: 'Err',
          unit: 'error between target and achieved',
          description: '',
        },
        {
          name: 'P',
          unit: 'proportional part of PID',
          description: '',
        },
        {
          name: 'I',
          unit: 'integral part of PID',
          description: '',
        },
        {
          name: 'D',
          unit: 'derivative part of PID',
          description: '',
        },
        {
          name: 'FF',
          unit: 'controller feed-forward portion of response',
          description: '',
        },
        {
          name: 'DFF',
          unit: 'controller derivative feed-forward portion of response',
          description: '',
        },
        {
          name: 'Dmod',
          unit: 'scaler applied to D gain to reduce limit cycling',
          description: '',
        },
        {
          name: 'SRate',
          unit: 'slew rate used in slew limiter',
          description: '',
        },
        {
          name: 'Flags',
          unit: 'bitmask',
          description: 'bitmask of PID state flags Bitmask values:',
        },
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
      [
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
    ],
  },
  {
    id: 'pids',
    title: 'PIDS',
    description: 'Proportional/Integral/Derivative gain values for ground steering yaw rate',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Tar',
          unit: 'desired value',
          description: '',
        },
        {
          name: 'Act',
          unit: 'achieved value',
          description: '',
        },
        {
          name: 'Err',
          unit: 'error between target and achieved',
          description: '',
        },
        {
          name: 'P',
          unit: 'proportional part of PID',
          description: '',
        },
        {
          name: 'I',
          unit: 'integral part of PID',
          description: '',
        },
        {
          name: 'D',
          unit: 'derivative part of PID',
          description: '',
        },
        {
          name: 'FF',
          unit: 'controller feed-forward portion of response',
          description: '',
        },
        {
          name: 'DFF',
          unit: 'controller derivative feed-forward portion of response',
          description: '',
        },
        {
          name: 'Dmod',
          unit: 'scaler applied to D gain to reduce limit cycling',
          description: '',
        },
        {
          name: 'SRate',
          unit: 'slew rate used in slew limiter',
          description: '',
        },
        {
          name: 'Flags',
          unit: 'bitmask',
          description: 'bitmask of PID state flags Bitmask values:',
        },
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
      [
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
    ],
  },
  {
    id: 'pidy',
    title: 'PIDY',
    description: 'Proportional/Integral/Derivative gain values for Yaw rate',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Tar',
          unit: 'desired value',
          description: '',
        },
        {
          name: 'Act',
          unit: 'achieved value',
          description: '',
        },
        {
          name: 'Err',
          unit: 'error between target and achieved',
          description: '',
        },
        {
          name: 'P',
          unit: 'proportional part of PID',
          description: '',
        },
        {
          name: 'I',
          unit: 'integral part of PID',
          description: '',
        },
        {
          name: 'D',
          unit: 'derivative part of PID',
          description: '',
        },
        {
          name: 'FF',
          unit: 'controller feed-forward portion of response',
          description: '',
        },
        {
          name: 'DFF',
          unit: 'controller derivative feed-forward portion of response',
          description: '',
        },
        {
          name: 'Dmod',
          unit: 'scaler applied to D gain to reduce limit cycling',
          description: '',
        },
        {
          name: 'SRate',
          unit: 'slew rate used in slew limiter',
          description: '',
        },
        {
          name: 'Flags',
          unit: 'bitmask',
          description: 'bitmask of PID state flags Bitmask values:',
        },
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
      [
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
    ],
  },
  {
    id: 'piqa',
    title: 'PIQA',
    description: 'QuadPlane Proportional/Integral/Derivative gain values for vertical acceleration',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Tar',
          unit: 'desired value',
          description: '',
        },
        {
          name: 'Act',
          unit: 'achieved value',
          description: '',
        },
        {
          name: 'Err',
          unit: 'error between target and achieved',
          description: '',
        },
        {
          name: 'P',
          unit: 'proportional part of PID',
          description: '',
        },
        {
          name: 'I',
          unit: 'integral part of PID',
          description: '',
        },
        {
          name: 'D',
          unit: 'derivative part of PID',
          description: '',
        },
        {
          name: 'FF',
          unit: 'controller feed-forward portion of response',
          description: '',
        },
        {
          name: 'DFF',
          unit: 'controller derivative feed-forward portion of response',
          description: '',
        },
        {
          name: 'Dmod',
          unit: 'scaler applied to D gain to reduce limit cycling',
          description: '',
        },
        {
          name: 'SRate',
          unit: 'slew rate',
          description: '',
        },
        {
          name: 'Flags',
          unit: 'bitmask',
          description: 'bitmask of PID state flags Bitmask values:',
        },
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
      [
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
    ],
  },
  {
    id: 'piqp',
    title: 'PIQP',
    description: 'QuadPlane Proportional/Integral/Derivative gain values for Pitch rate',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Tar',
          unit: 'desired value',
          description: '',
        },
        {
          name: 'Act',
          unit: 'achieved value',
          description: '',
        },
        {
          name: 'Err',
          unit: 'error between target and achieved',
          description: '',
        },
        {
          name: 'P',
          unit: 'proportional part of PID',
          description: '',
        },
        {
          name: 'I',
          unit: 'integral part of PID',
          description: '',
        },
        {
          name: 'D',
          unit: 'derivative part of PID',
          description: '',
        },
        {
          name: 'FF',
          unit: 'controller feed-forward portion of response',
          description: '',
        },
        {
          name: 'DFF',
          unit: 'controller derivative feed-forward portion of response',
          description: '',
        },
        {
          name: 'Dmod',
          unit: 'scaler applied to D gain to reduce limit cycling',
          description: '',
        },
        {
          name: 'SRate',
          unit: 'slew rate',
          description: '',
        },
        {
          name: 'Flags',
          unit: 'bitmask',
          description: 'bitmask of PID state flags Bitmask values:',
        },
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
      [
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
    ],
  },
  {
    id: 'piqr',
    title: 'PIQR',
    description: 'QuadPlane Proportional/Integral/Derivative gain values for Roll rate',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Tar',
          unit: 'desired value',
          description: '',
        },
        {
          name: 'Act',
          unit: 'achieved value',
          description: '',
        },
        {
          name: 'Err',
          unit: 'error between target and achieved',
          description: '',
        },
        {
          name: 'P',
          unit: 'proportional part of PID',
          description: '',
        },
        {
          name: 'I',
          unit: 'integral part of PID',
          description: '',
        },
        {
          name: 'D',
          unit: 'derivative part of PID',
          description: '',
        },
        {
          name: 'FF',
          unit: 'controller feed-forward portion of response',
          description: '',
        },
        {
          name: 'DFF',
          unit: 'controller derivative feed-forward portion of response',
          description: '',
        },
        {
          name: 'Dmod',
          unit: 'scaler applied to D gain to reduce limit cycling',
          description: '',
        },
        {
          name: 'SRate',
          unit: 'slew rate',
          description: '',
        },
        {
          name: 'Flags',
          unit: 'bitmask',
          description: 'bitmask of PID state flags Bitmask values:',
        },
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
      [
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
    ],
  },
  {
    id: 'piqy',
    title: 'PIQY',
    description: 'QuadPlane Proportional/Integral/Derivative gain values for Yaw rate',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Tar',
          unit: 'desired value',
          description: '',
        },
        {
          name: 'Act',
          unit: 'achieved value',
          description: '',
        },
        {
          name: 'Err',
          unit: 'error between target and achieved',
          description: '',
        },
        {
          name: 'P',
          unit: 'proportional part of PID',
          description: '',
        },
        {
          name: 'I',
          unit: 'integral part of PID',
          description: '',
        },
        {
          name: 'D',
          unit: 'derivative part of PID',
          description: '',
        },
        {
          name: 'FF',
          unit: 'controller feed-forward portion of response',
          description: '',
        },
        {
          name: 'DFF',
          unit: 'controller derivative feed-forward portion of response',
          description: '',
        },
        {
          name: 'Dmod',
          unit: 'scaler applied to D gain to reduce limit cycling',
          description: '',
        },
        {
          name: 'SRate',
          unit: 'slew rate',
          description: '',
        },
        {
          name: 'Flags',
          unit: 'bitmask',
          description: 'bitmask of PID state flags Bitmask values:',
        },
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
      [
        {
          name: 'LIMIT',
          unit: '1',
          description: 'true if the output is saturated, I term anti windup is active',
        },
        {
          name: 'PD_SUM_LIMIT',
          unit: '2',
          description: 'true if the PD sum limit is active',
        },
        {
          name: 'RESET',
          unit: '4',
          description: 'true if the controller was reset',
        },
        {
          name: 'I_TERM_SET',
          unit: '8',
          description: 'true if the I term has been set externally including reseting to 0',
        },
      ],
    ],
  },
  {
    id: 'pl',
    title: 'PL',
    description: 'Precision Landing messages',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Heal',
          unit: 'True if Precision Landing is healthy',
          description: '',
        },
        {
          name: 'TAcq',
          unit: 'True if landing target is detected',
          description: '',
        },
        {
          name: 'pX',
          unit: 'm',
          description: 'Target position relative to vehicle, X-Axis (0 if target not found)',
        },
        {
          name: 'pY',
          unit: 'm',
          description: 'Target position relative to vehicle, Y-Axis (0 if target not found)',
        },
        {
          name: 'vX',
          unit: 'm/s',
          description: 'Target velocity relative to vehicle, X-Axis (0 if target not found)',
        },
        {
          name: 'vY',
          unit: 'm/s',
          description: 'Target velocity relative to vehicle, Y-Axis (0 if target not found)',
        },
        {
          name: 'mX',
          unit: 'm',
          description: 'Target’s relative to origin position as 3-D Vector, X-Axis',
        },
        {
          name: 'mY',
          unit: 'm',
          description: 'Target’s relative to origin position as 3-D Vector, Y-Axis',
        },
        {
          name: 'mZ',
          unit: 'm',
          description: 'Target’s relative to origin position as 3-D Vector, Z-Axis',
        },
        {
          name: 'LastMeasMS',
          unit: 'ms',
          description: 'Time when target was last detected',
        },
        {
          name: 'EKFOutl',
          unit: 'EKF’s outlier count',
          description: '',
        },
        {
          name: 'Est',
          unit: 'Type of estimator used',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'pm',
    title: 'PM',
    description: 'autopilot system performance and general data dumping ground',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'LR',
          unit: 'Hz',
          description: 'Main loop rate',
        },
        {
          name: 'NLon',
          unit: 'Number of long loops detected',
          description: '',
        },
        {
          name: 'NL',
          unit: 'Number of measurement loops for this message',
          description: '',
        },
        {
          name: 'MaxT',
          unit: 'Maximum loop time',
          description: '',
        },
        {
          name: 'Mem',
          unit: 'B',
          description: 'Free memory available',
        },
        {
          name: 'Load',
          unit: 'd%',
          description: 'System processor load',
        },
        {
          name: 'ErrL',
          unit:
            'Internal error line number; last line number on which a internal error was detected',
          description: '',
        },
        {
          name: 'InE',
          unit: 'bitmask',
          description:
            'Internal error mask; which internal errors have been detected Bitmask values:',
        },
        {
          name: 'logger_mapfailure',
          unit: '1',
          description: '',
        },
        {
          name: 'logger_missing_logstructure',
          unit: '2',
          description: '',
        },
        {
          name: 'logger_logwrite_missingfmt',
          unit: '4',
          description: '',
        },
        {
          name: 'logger_too_many_deletions',
          unit: '8',
          description: '',
        },
        {
          name: 'logger_bad_getfilename',
          unit: '16',
          description: '',
        },
        {
          name: 'panic',
          unit: '32',
          description: '',
        },
        {
          name: 'logger_flushing_without_sem',
          unit: '64',
          description: '',
        },
        {
          name: 'logger_bad_current_block',
          unit: '128',
          description: '',
        },
        {
          name: 'logger_blockcount_mismatch',
          unit: '256',
          description: '',
        },
        {
          name: 'logger_dequeue_failure',
          unit: '512',
          description: '',
        },
        {
          name: 'constraining_nan',
          unit: '1024',
          description: '',
        },
        {
          name: 'watchdog_reset',
          unit: '2048',
          description: '',
        },
        {
          name: 'iomcu_reset',
          unit: '4096',
          description: '',
        },
        {
          name: 'iomcu_fail',
          unit: '8192',
          description: '',
        },
        {
          name: 'spi_fail',
          unit: '16384',
          description: '',
        },
        {
          name: 'main_loop_stuck',
          unit: '32768',
          description: '',
        },
        {
          name: 'gcs_bad_missionprotocol_link',
          unit: '65536',
          description: '',
        },
        {
          name: 'bitmask_range',
          unit: '131072',
          description: '',
        },
        {
          name: 'gcs_offset',
          unit: '262144',
          description: '',
        },
        {
          name: 'i2c_isr',
          unit: '524288',
          description: '',
        },
        {
          name: 'flow_of_control',
          unit: '1048576',
          description: '',
        },
        {
          name: 'switch_full_sector_recursion',
          unit: '2097152',
          description: '',
        },
        {
          name: 'bad_rotation',
          unit: '4194304',
          description: '',
        },
        {
          name: 'stack_overflow',
          unit: '8388608',
          description: '',
        },
        {
          name: 'imu_reset',
          unit: '16777216',
          description: '',
        },
        {
          name: 'gpio_isr',
          unit: '33554432',
          description: '',
        },
        {
          name: 'mem_guard',
          unit: '67108864',
          description: '',
        },
        {
          name: 'dma_fail',
          unit: '134217728',
          description: '',
        },
        {
          name: 'params_restored',
          unit: '268435456',
          description: '',
        },
        {
          name: 'invalid_arg_or_result',
          unit: '536870912',
          description: '',
        },
        {
          name: '__LAST__',
          unit: '1073741824',
          description: '',
        },
        {
          name: 'ErC',
          unit: 'Internal error count; how many internal errors have been detected',
          description: '',
        },
        {
          name: 'SPIC',
          unit: 'Number of SPI transactions processed',
          description: '',
        },
        {
          name: 'I2CC',
          unit: 'Number of i2c transactions processed',
          description: '',
        },
        {
          name: 'I2CI',
          unit: 'Number of i2c interrupts serviced',
          description: '',
        },
        {
          name: 'Ex',
          unit: 'μs',
          description:
            'number of microseconds being added to each loop to address scheduler overruns',
        },
        {
          name: 'R',
          unit: 'μs',
          description: 'RTC time, time since Unix epoch',
        },
      ],
      [
        {
          name: 'logger_mapfailure',
          unit: '1',
          description: '',
        },
        {
          name: 'logger_missing_logstructure',
          unit: '2',
          description: '',
        },
        {
          name: 'logger_logwrite_missingfmt',
          unit: '4',
          description: '',
        },
        {
          name: 'logger_too_many_deletions',
          unit: '8',
          description: '',
        },
        {
          name: 'logger_bad_getfilename',
          unit: '16',
          description: '',
        },
        {
          name: 'panic',
          unit: '32',
          description: '',
        },
        {
          name: 'logger_flushing_without_sem',
          unit: '64',
          description: '',
        },
        {
          name: 'logger_bad_current_block',
          unit: '128',
          description: '',
        },
        {
          name: 'logger_blockcount_mismatch',
          unit: '256',
          description: '',
        },
        {
          name: 'logger_dequeue_failure',
          unit: '512',
          description: '',
        },
        {
          name: 'constraining_nan',
          unit: '1024',
          description: '',
        },
        {
          name: 'watchdog_reset',
          unit: '2048',
          description: '',
        },
        {
          name: 'iomcu_reset',
          unit: '4096',
          description: '',
        },
        {
          name: 'iomcu_fail',
          unit: '8192',
          description: '',
        },
        {
          name: 'spi_fail',
          unit: '16384',
          description: '',
        },
        {
          name: 'main_loop_stuck',
          unit: '32768',
          description: '',
        },
        {
          name: 'gcs_bad_missionprotocol_link',
          unit: '65536',
          description: '',
        },
        {
          name: 'bitmask_range',
          unit: '131072',
          description: '',
        },
        {
          name: 'gcs_offset',
          unit: '262144',
          description: '',
        },
        {
          name: 'i2c_isr',
          unit: '524288',
          description: '',
        },
        {
          name: 'flow_of_control',
          unit: '1048576',
          description: '',
        },
        {
          name: 'switch_full_sector_recursion',
          unit: '2097152',
          description: '',
        },
        {
          name: 'bad_rotation',
          unit: '4194304',
          description: '',
        },
        {
          name: 'stack_overflow',
          unit: '8388608',
          description: '',
        },
        {
          name: 'imu_reset',
          unit: '16777216',
          description: '',
        },
        {
          name: 'gpio_isr',
          unit: '33554432',
          description: '',
        },
        {
          name: 'mem_guard',
          unit: '67108864',
          description: '',
        },
        {
          name: 'dma_fail',
          unit: '134217728',
          description: '',
        },
        {
          name: 'params_restored',
          unit: '268435456',
          description: '',
        },
        {
          name: 'invalid_arg_or_result',
          unit: '536870912',
          description: '',
        },
        {
          name: '__LAST__',
          unit: '1073741824',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'pos',
    title: 'POS',
    description: 'Canonical vehicle position',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Lat',
          unit: 'deglatitude',
          description: 'Canonical vehicle latitude',
        },
        {
          name: 'Lng',
          unit: 'deglongitude',
          description: 'Canonical vehicle longitude',
        },
        {
          name: 'Alt',
          unit: 'm',
          description: 'Canonical vehicle altitude',
        },
        {
          name: 'RelHomeAlt',
          unit: 'm',
          description: 'Canonical vehicle altitude relative to home',
        },
        {
          name: 'RelOriginAlt',
          unit: 'm',
          description: 'Canonical vehicle altitude relative to navigation origin',
        },
      ],
    ],
  },
  {
    id: 'powr',
    title: 'POWR',
    description: 'System power information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Vcc',
          unit: 'V',
          description: 'Flight board voltage',
        },
        {
          name: 'VServo',
          unit: 'V',
          description: 'Servo rail voltage',
        },
        {
          name: 'Flags',
          unit: 'bitmask',
          description: 'System power flags Bitmask values:',
        },
        {
          name: 'BRICK_VALID',
          unit: '1',
          description: 'main brick power supply valid',
        },
        {
          name: 'SERVO_VALID',
          unit: '2',
          description: 'main servo power supply valid for FMU',
        },
        {
          name: 'USB_CONNECTED',
          unit: '4',
          description: 'USB power is connected',
        },
        {
          name: 'PERIPH_OVERCURRENT',
          unit: '8',
          description: 'peripheral supply is in over-current state',
        },
        {
          name: 'PERIPH_HIPOWER_OVERCURRENT',
          unit: '16',
          description: 'hi-power peripheral supply is in over-current state',
        },
        {
          name: 'CHANGED',
          unit: '32',
          description: 'Power status has changed since boot',
        },
        {
          name: 'AccFlags',
          unit: 'bitmask',
          description:
            'Accumulated System power flags; all flags which have ever been set Bitmask values:',
        },
        {
          name: 'BRICK_VALID',
          unit: '1',
          description: 'main brick power supply valid',
        },
        {
          name: 'SERVO_VALID',
          unit: '2',
          description: 'main servo power supply valid for FMU',
        },
        {
          name: 'USB_CONNECTED',
          unit: '4',
          description: 'USB power is connected',
        },
        {
          name: 'PERIPH_OVERCURRENT',
          unit: '8',
          description: 'peripheral supply is in over-current state',
        },
        {
          name: 'PERIPH_HIPOWER_OVERCURRENT',
          unit: '16',
          description: 'hi-power peripheral supply is in over-current state',
        },
        {
          name: 'CHANGED',
          unit: '32',
          description: 'Power status has changed since boot',
        },
        {
          name: 'Safety',
          unit: 'Hardware Safety Switch status',
          description: '',
        },
      ],
      [
        {
          name: 'BRICK_VALID',
          unit: '1',
          description: 'main brick power supply valid',
        },
        {
          name: 'SERVO_VALID',
          unit: '2',
          description: 'main servo power supply valid for FMU',
        },
        {
          name: 'USB_CONNECTED',
          unit: '4',
          description: 'USB power is connected',
        },
        {
          name: 'PERIPH_OVERCURRENT',
          unit: '8',
          description: 'peripheral supply is in over-current state',
        },
        {
          name: 'PERIPH_HIPOWER_OVERCURRENT',
          unit: '16',
          description: 'hi-power peripheral supply is in over-current state',
        },
        {
          name: 'CHANGED',
          unit: '32',
          description: 'Power status has changed since boot',
        },
      ],
      [
        {
          name: 'BRICK_VALID',
          unit: '1',
          description: 'main brick power supply valid',
        },
        {
          name: 'SERVO_VALID',
          unit: '2',
          description: 'main servo power supply valid for FMU',
        },
        {
          name: 'USB_CONNECTED',
          unit: '4',
          description: 'USB power is connected',
        },
        {
          name: 'PERIPH_OVERCURRENT',
          unit: '8',
          description: 'peripheral supply is in over-current state',
        },
        {
          name: 'PERIPH_HIPOWER_OVERCURRENT',
          unit: '16',
          description: 'hi-power peripheral supply is in over-current state',
        },
        {
          name: 'CHANGED',
          unit: '32',
          description: 'Power status has changed since boot',
        },
      ],
    ],
  },
  {
    id: 'prtn',
    title: 'PRTN',
    description: 'Plane Parameter Tuning data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'Set',
          unit: 'Parameter set being tuned',
          description: '',
        },
        {
          name: 'Parm',
          unit: 'Parameter being tuned',
          description: '',
        },
        {
          name: 'Value',
          unit: 'Current parameter value',
          description: '',
        },
        {
          name: 'CenterValue',
          unit: 'Center value (startpoint of current modifications) of parameter being tuned',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'prx',
    title: 'PRX',
    description: 'Proximity Filtered sensor data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Layer',
          unit: 'instance',
          description:
            'Pitch(instance) at which the obstacle is at. 0th layer {-75,-45} degrees. 1st layer {-45,-15} degrees. 2nd layer {-15, 15} degrees. 3rd layer {15, 45} degrees. 4th layer {45,75} degrees. Minimum distance in each layer will be logged.',
        },
        {
          name: 'He',
          unit: 'True if proximity sensor is healthy',
          description: '',
        },
        {
          name: 'D0',
          unit: 'm',
          description: 'Nearest object in sector surrounding 0-degrees',
        },
        {
          name: 'D45',
          unit: 'm',
          description: 'Nearest object in sector surrounding 45-degrees',
        },
        {
          name: 'D90',
          unit: 'm',
          description: 'Nearest object in sector surrounding 90-degrees',
        },
        {
          name: 'D135',
          unit: 'm',
          description: 'Nearest object in sector surrounding 135-degrees',
        },
        {
          name: 'D180',
          unit: 'm',
          description: 'Nearest object in sector surrounding 180-degrees',
        },
        {
          name: 'D225',
          unit: 'm',
          description: 'Nearest object in sector surrounding 225-degrees',
        },
        {
          name: 'D270',
          unit: 'm',
          description: 'Nearest object in sector surrounding 270-degrees',
        },
        {
          name: 'D315',
          unit: 'm',
          description: 'Nearest object in sector surrounding 315-degrees',
        },
        {
          name: 'DUp',
          unit: 'm',
          description: 'Nearest object in upwards direction',
        },
        {
          name: 'CAn',
          unit: 'degheading',
          description: 'Angle to closest object',
        },
        {
          name: 'CDis',
          unit: 'm',
          description: 'Distance to closest object',
        },
      ],
    ],
  },
  {
    id: 'prxr',
    title: 'PRXR',
    description: 'Proximity Raw sensor data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Layer',
          unit: 'instance',
          description:
            'Pitch(instance) at which the obstacle is at. 0th layer {-75,-45} degrees. 1st layer {-45,-15} degrees. 2nd layer {-15, 15} degrees. 3rd layer {15, 45} degrees. 4th layer {45,75} degrees. Minimum distance in each layer will be logged.',
        },
        {
          name: 'D0',
          unit: 'm',
          description: 'Nearest object in sector surrounding 0-degrees',
        },
        {
          name: 'D45',
          unit: 'm',
          description: 'Nearest object in sector surrounding 45-degrees',
        },
        {
          name: 'D90',
          unit: 'm',
          description: 'Nearest object in sector surrounding 90-degrees',
        },
        {
          name: 'D135',
          unit: 'm',
          description: 'Nearest object in sector surrounding 135-degrees',
        },
        {
          name: 'D180',
          unit: 'm',
          description: 'Nearest object in sector surrounding 180-degrees',
        },
        {
          name: 'D225',
          unit: 'm',
          description: 'Nearest object in sector surrounding 225-degrees',
        },
        {
          name: 'D270',
          unit: 'm',
          description: 'Nearest object in sector surrounding 270-degrees',
        },
        {
          name: 'D315',
          unit: 'm',
          description: 'Nearest object in sector surrounding 315-degrees',
        },
      ],
    ],
  },
  {
    id: 'pscd',
    title: 'PSCD',
    description: 'Position Control Down',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'DPD',
          unit: 'm',
          description: 'Desired position relative to EKF origin + Offsets',
        },
        {
          name: 'TPD',
          unit: 'm',
          description: 'Target position relative to EKF origin',
        },
        {
          name: 'PD',
          unit: 'm',
          description: 'Position relative to EKF origin',
        },
        {
          name: 'DVD',
          unit: 'm/s',
          description: 'Desired velocity Down',
        },
        {
          name: 'TVD',
          unit: 'm/s',
          description: 'Target velocity Down',
        },
        {
          name: 'VD',
          unit: 'm/s',
          description: 'Velocity Down',
        },
        {
          name: 'DAD',
          unit: 'm/s/s',
          description: 'Desired acceleration Down',
        },
        {
          name: 'TAD',
          unit: 'm/s/s',
          description: 'Target acceleration Down',
        },
        {
          name: 'AD',
          unit: 'm/s/s',
          description: 'Acceleration Down',
        },
      ],
    ],
  },
  {
    id: 'psce',
    title: 'PSCE',
    description: 'Position Control East',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'DPE',
          unit: 'm',
          description: 'Desired position relative to EKF origin + Offsets',
        },
        {
          name: 'TPE',
          unit: 'm',
          description: 'Target position relative to EKF origin',
        },
        {
          name: 'PE',
          unit: 'm',
          description: 'Position relative to EKF origin',
        },
        {
          name: 'DVE',
          unit: 'm/s',
          description: 'Desired velocity East',
        },
        {
          name: 'TVE',
          unit: 'm/s',
          description: 'Target velocity East',
        },
        {
          name: 'VE',
          unit: 'm/s',
          description: 'Velocity East',
        },
        {
          name: 'DAE',
          unit: 'm/s/s',
          description: 'Desired acceleration East',
        },
        {
          name: 'TAE',
          unit: 'm/s/s',
          description: 'Target acceleration East',
        },
        {
          name: 'AE',
          unit: 'm/s/s',
          description: 'Acceleration East',
        },
      ],
    ],
  },
  {
    id: 'pscn',
    title: 'PSCN',
    description: 'Position Control North',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'DPN',
          unit: 'm',
          description: 'Desired position relative to EKF origin',
        },
        {
          name: 'TPN',
          unit: 'm',
          description: 'Target position relative to EKF origin',
        },
        {
          name: 'PN',
          unit: 'm',
          description: 'Position relative to EKF origin',
        },
        {
          name: 'DVN',
          unit: 'm/s',
          description: 'Desired velocity North',
        },
        {
          name: 'TVN',
          unit: 'm/s',
          description: 'Target velocity North',
        },
        {
          name: 'VN',
          unit: 'm/s',
          description: 'Velocity North',
        },
        {
          name: 'DAN',
          unit: 'm/s/s',
          description: 'Desired acceleration North',
        },
        {
          name: 'TAN',
          unit: 'm/s/s',
          description: 'Target acceleration North',
        },
        {
          name: 'AN',
          unit: 'm/s/s',
          description: 'Acceleration North',
        },
      ],
    ],
  },
  {
    id: 'psod',
    title: 'PSOD',
    description: 'Position Control Offsets Down',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'TPOD',
          unit: 'm',
          description: 'Target position offset Down',
        },
        {
          name: 'POD',
          unit: 'm',
          description: 'Position offset Down',
        },
        {
          name: 'TVOD',
          unit: 'm/s',
          description: 'Target velocity offset Down',
        },
        {
          name: 'VOD',
          unit: 'm/s',
          description: 'Velocity offset Down',
        },
        {
          name: 'TAOD',
          unit: 'm/s/s',
          description: 'Target acceleration offset Down',
        },
        {
          name: 'AOD',
          unit: 'm/s/s',
          description: 'Acceleration offset Down',
        },
      ],
    ],
  },
  {
    id: 'psoe',
    title: 'PSOE',
    description: 'Position Control Offsets East',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'TPOE',
          unit: 'm',
          description: 'Target position offset East',
        },
        {
          name: 'POE',
          unit: 'm',
          description: 'Position offset East',
        },
        {
          name: 'TVOE',
          unit: 'm/s',
          description: 'Target velocity offset East',
        },
        {
          name: 'VOE',
          unit: 'm/s',
          description: 'Velocity offset East',
        },
        {
          name: 'TAOE',
          unit: 'm/s/s',
          description: 'Target acceleration offset East',
        },
        {
          name: 'AOE',
          unit: 'm/s/s',
          description: 'Acceleration offset East',
        },
      ],
    ],
  },
  {
    id: 'pson',
    title: 'PSON',
    description: 'Position Control Offsets North',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'TPON',
          unit: 'm',
          description: 'Target position offset North',
        },
        {
          name: 'PON',
          unit: 'm',
          description: 'Position offset North',
        },
        {
          name: 'TVON',
          unit: 'm/s',
          description: 'Target velocity offset North',
        },
        {
          name: 'VON',
          unit: 'm/s',
          description: 'Velocity offset North',
        },
        {
          name: 'TAON',
          unit: 'm/s/s',
          description: 'Target acceleration offset North',
        },
        {
          name: 'AON',
          unit: 'm/s/s',
          description: 'Acceleration offset North',
        },
      ],
    ],
  },
  {
    id: 'psot',
    title: 'PSOT',
    description: 'Position Control Offsets Terrain (Down)',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'TPOT',
          unit: 'm',
          description: 'Target position offset Terrain',
        },
        {
          name: 'POT',
          unit: 'm',
          description: 'Position offset Terrain',
        },
        {
          name: 'TVOT',
          unit: 'm/s',
          description: 'Target velocity offset Terrain',
        },
        {
          name: 'VOT',
          unit: 'm/s',
          description: 'Velocity offset Terrain',
        },
        {
          name: 'TAOT',
          unit: 'm/s/s',
          description: 'Target acceleration offset Terrain',
        },
        {
          name: 'AOT',
          unit: 'm/s/s',
          description: 'Acceleration offset Terrain',
        },
      ],
    ],
  },
  {
    id: 'qbrk',
    title: 'QBRK',
    description: 'Quadplane Braking',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'SpdScaler',
          unit: 'braking speed scaler',
          description: '',
        },
        {
          name: 'NPULCD',
          unit: 'upper limit for navigation pitch',
          description: '',
        },
        {
          name: 'QBPLCD',
          unit: 'upper limit for back transition pitch',
          description: '',
        },
        {
          name: 'NPCD',
          unit: 'demanded navigation pitch',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'qpos',
    title: 'QPOS',
    description: 'Quadplane position data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'State',
          unit: 'enum',
          description: 'Position control state Values:',
        },
        {
          name: 'QPOS_NONE',
          unit: '0',
          description: '',
        },
        {
          name: 'QPOS_APPROACH',
          unit: '1',
          description: '',
        },
        {
          name: 'QPOS_AIRBRAKE',
          unit: '2',
          description: '',
        },
        {
          name: 'QPOS_POSITION1',
          unit: '3',
          description: '',
        },
        {
          name: 'QPOS_POSITION2',
          unit: '4',
          description: '',
        },
        {
          name: 'QPOS_LAND_DESCEND',
          unit: '5',
          description: '',
        },
        {
          name: 'QPOS_LAND_ABORT',
          unit: '6',
          description: '',
        },
        {
          name: 'QPOS_LAND_FINAL',
          unit: '7',
          description: '',
        },
        {
          name: 'QPOS_LAND_COMPLETE',
          unit: '8',
          description: '',
        },
        {
          name: 'Dist',
          unit: 'Distance to next waypoint',
          description: '',
        },
        {
          name: 'TSpd',
          unit: 'Target speed',
          description: '',
        },
        {
          name: 'TAcc',
          unit: 'Target acceleration',
          description: '',
        },
        {
          name: 'OShoot',
          unit: 'True if landing point is overshot or heading off by more than 60 degrees',
          description: '',
        },
      ],
      [
        {
          name: 'QPOS_NONE',
          unit: '0',
          description: '',
        },
        {
          name: 'QPOS_APPROACH',
          unit: '1',
          description: '',
        },
        {
          name: 'QPOS_AIRBRAKE',
          unit: '2',
          description: '',
        },
        {
          name: 'QPOS_POSITION1',
          unit: '3',
          description: '',
        },
        {
          name: 'QPOS_POSITION2',
          unit: '4',
          description: '',
        },
        {
          name: 'QPOS_LAND_DESCEND',
          unit: '5',
          description: '',
        },
        {
          name: 'QPOS_LAND_ABORT',
          unit: '6',
          description: '',
        },
        {
          name: 'QPOS_LAND_FINAL',
          unit: '7',
          description: '',
        },
        {
          name: 'QPOS_LAND_COMPLETE',
          unit: '8',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'qtun',
    title: 'QTUN',
    description: 'QuadPlane vertical tuning message',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'ThI',
          unit: 'throttle input',
          description: '',
        },
        {
          name: 'ABst',
          unit: 'angle boost',
          description: '',
        },
        {
          name: 'ThO',
          unit: 'throttle output',
          description: '',
        },
        {
          name: 'ThH',
          unit: 'calculated hover throttle',
          description: '',
        },
        {
          name: 'DAlt',
          unit: 'm',
          description: 'desired altitude',
        },
        {
          name: 'Alt',
          unit: 'm',
          description: 'achieved altitude',
        },
        {
          name: 'BAlt',
          unit: 'm',
          description: 'barometric altitude',
        },
        {
          name: 'DCRt',
          unit: 'm/s',
          description: 'desired climb rate',
        },
        {
          name: 'CRt',
          unit: 'm/s',
          description: 'climb rate',
        },
        {
          name: 'TMix',
          unit: 'transition throttle mix value',
          description: '',
        },
        {
          name: 'Trn',
          unit:
            'Transition state: 0-AirspeedWait,1-Timer,2-Done / TailSitter: 0-FW Wait,1-VTOL Wait,2-Done',
          description: '',
        },
        {
          name: 'Ast',
          unit: 'bitmask',
          description: 'bitmask of assistance flags Bitmask values:',
        },
        {
          name: 'in_assisted_flight',
          unit: '1',
          description: 'true if VTOL assist is active',
        },
        {
          name: 'forced',
          unit: '2',
          description: 'true if assistance is forced',
        },
        {
          name: 'speed',
          unit: '4',
          description: 'true if assistance due to low airspeed',
        },
        {
          name: 'alt',
          unit: '8',
          description: 'true if assistance due to low altitude',
        },
        {
          name: 'angle',
          unit: '16',
          description: 'true if assistance due to attitude error',
        },
        {
          name: 'fw_force',
          unit: '32',
          description: 'true if forcing use of fixed wing controllers',
        },
        {
          name: 'spin_recovery',
          unit: '64',
          description: 'true if recovering from a spin',
        },
      ],
      [
        {
          name: 'in_assisted_flight',
          unit: '1',
          description: 'true if VTOL assist is active',
        },
        {
          name: 'forced',
          unit: '2',
          description: 'true if assistance is forced',
        },
        {
          name: 'speed',
          unit: '4',
          description: 'true if assistance due to low airspeed',
        },
        {
          name: 'alt',
          unit: '8',
          description: 'true if assistance due to low altitude',
        },
        {
          name: 'angle',
          unit: '16',
          description: 'true if assistance due to attitude error',
        },
        {
          name: 'fw_force',
          unit: '32',
          description: 'true if forcing use of fixed wing controllers',
        },
        {
          name: 'spin_recovery',
          unit: '64',
          description: 'true if recovering from a spin',
        },
      ],
    ],
  },
  {
    id: 'qwik',
    title: 'QWIK',
    description: 'Quicktune',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 's',
          description: 'Time since system startup',
        },
        {
          name: 'ParamNo',
          unit: 'instance',
          description: 'number of parameter being tuned',
        },
        {
          name: 'SRate',
          unit: 'slew rate',
          description: '',
        },
        {
          name: 'Gain',
          unit: 'test gain for current axis and PID element',
          description: '',
        },
        {
          name: 'Param',
          unit: 'char[16]',
          description: 'name of parameter being being tuned',
        },
      ],
    ],
  },
  {
    id: 'rad',
    title: 'RAD',
    description: 'Telemetry radio statistics',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'RSSI',
          unit: 'RSSI',
          description: '',
        },
        {
          name: 'RemRSSI',
          unit: 'RSSI reported from remote radio',
          description: '',
        },
        {
          name: 'TxBuf',
          unit: 'number of bytes in radio ready to be sent',
          description: '',
        },
        {
          name: 'Noise',
          unit: 'local noise floor',
          description: '',
        },
        {
          name: 'RemNoise',
          unit: 'local noise floor reported from remote radio',
          description: '',
        },
        {
          name: 'RxErrors',
          unit: 'damaged packet count',
          description: '',
        },
        {
          name: 'Fixed',
          unit: 'fixed damaged packet count',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'raly',
    title: 'RALY',
    description: 'Rally point information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Tot',
          unit: 'total number of rally points onboard',
          description: '',
        },
        {
          name: 'Seq',
          unit: 'this rally point’s sequence number',
          description: '',
        },
        {
          name: 'Lat',
          unit: 'deglatitude',
          description: 'latitude of rally point',
        },
        {
          name: 'Lng',
          unit: 'deglongitude',
          description: 'longitude of rally point',
        },
        {
          name: 'Alt',
          unit: 'cm',
          description: 'altitude of rally point',
        },
        {
          name: 'Flags',
          unit: 'altitude frame flags',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rash',
    title: 'RASH',
    description: 'Replay Airspeed Sensor Header',
    tables: [
      [
        {
          name: 'Primary',
          unit: 'airspeed instance number',
          description: '',
        },
        {
          name: 'NumInst',
          unit: 'number of airspeed instances',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rasi',
    title: 'RASI',
    description: 'Replay Airspeed Sensor Instance data',
    tables: [
      [
        {
          name: 'pd',
          unit: 'measured airspeed',
          description: '',
        },
        {
          name: 'UpdateMS',
          unit: 'timestamp of measured airspeed',
          description: '',
        },
        {
          name: 'H',
          unit: 'indicator of airspeed sensor health',
          description: '',
        },
        {
          name: 'Use',
          unit: 'true if airspeed is configured to be used',
          description: '',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'airspeed instance number',
        },
      ],
    ],
  },
  {
    id: 'rate',
    title: 'RATE',
    description:
      'Desired and achieved vehicle attitude rates. Not logged in Fixed Wing Plane modes.',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'RDes',
          unit: 'deg/s',
          description: 'vehicle desired roll rate',
        },
        {
          name: 'R',
          unit: 'deg/s',
          description: 'achieved vehicle roll rate',
        },
        {
          name: 'ROut',
          unit: 'normalized output for Roll',
          description: '',
        },
        {
          name: 'PDes',
          unit: 'deg/s',
          description: 'vehicle desired pitch rate',
        },
        {
          name: 'P',
          unit: 'deg/s',
          description: 'vehicle pitch rate',
        },
        {
          name: 'POut',
          unit: 'normalized output for Pitch',
          description: '',
        },
        {
          name: 'YDes',
          unit: 'deg/s',
          description: 'vehicle desired yaw rate',
        },
        {
          name: 'Y',
          unit: 'deg/s',
          description: 'achieved vehicle yaw rate',
        },
        {
          name: 'YOut',
          unit: 'normalized output for Yaw',
          description: '',
        },
        {
          name: 'ADes',
          unit: 'cm/s/s',
          description: 'desired vehicle vertical acceleration',
        },
        {
          name: 'A',
          unit: 'cm/s/s',
          description: 'achieved vehicle vertical acceleration',
        },
        {
          name: 'AOut',
          unit: 'percentage of vertical thrust output current being used',
          description: '',
        },
        {
          name: 'AOutSlew',
          unit: 'vertical thrust output slew rate',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rbch',
    title: 'RBCH',
    description: 'Replay Data Beacon Header',
    tables: [
      [
        {
          name: 'PX',
          unit: 'zero, unused',
          description: '',
        },
        {
          name: 'PY',
          unit: 'zero, unused',
          description: '',
        },
        {
          name: 'PZ',
          unit: 'zero, unused',
          description: '',
        },
        {
          name: 'AE',
          unit: 'zero, unused',
          description: '',
        },
        {
          name: 'OLat',
          unit: 'origin latitude',
          description: '',
        },
        {
          name: 'OLng',
          unit: 'origin longitude',
          description: '',
        },
        {
          name: 'OAlt',
          unit: 'origin altitude',
          description: '',
        },
        {
          name: 'Flags',
          unit: 'vehicle_position_ned_returncode,get_origin_returncode,enabled',
          description: '',
        },
        {
          name: 'NumInst',
          unit: 'number of beacons',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rbci',
    title: 'RBCI',
    description: 'Replay Data Beacon Instance',
    tables: [
      [
        {
          name: 'LU',
          unit: 's',
          description: 'last update from this beacon instance',
        },
        {
          name: 'PX',
          unit: 'm',
          description: 'beacon distance from origin, X-axis',
        },
        {
          name: 'PY',
          unit: 'm',
          description: 'beacon distance from origin, Y-axis',
        },
        {
          name: 'PZ',
          unit: 'm',
          description: 'beacon distance from origin, Z-axis',
        },
        {
          name: 'Dist',
          unit: 'm',
          description: 'distance to beacon',
        },
        {
          name: 'H',
          unit: 'beacon data health',
          description: '',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'beacon instance number',
        },
      ],
    ],
  },
  {
    id: 'rboh',
    title: 'RBOH',
    description: 'Replay body odometry data',
    tables: [
      [
        {
          name: 'Q',
          unit: 'data quality measure',
          description: '',
        },
        {
          name: 'DPX',
          unit: 'delta-position-X',
          description: '',
        },
        {
          name: 'DPY',
          unit: 'delta-position-Y',
          description: '',
        },
        {
          name: 'DPZ',
          unit: 'delta-position-Z',
          description: '',
        },
        {
          name: 'DAX',
          unit: 'delta-angle-X',
          description: '',
        },
        {
          name: 'DAY',
          unit: 'delta-angle-Y',
          description: '',
        },
        {
          name: 'DAZ',
          unit: 'delta-angle-Z',
          description: '',
        },
        {
          name: 'DT',
          unit: 'delta-time',
          description: '',
        },
        {
          name: 'TS',
          unit: 'data timestamp',
          description: '',
        },
        {
          name: 'OX',
          unit: 'zero, unused',
          description: '',
        },
        {
          name: 'OY',
          unit: 'zero, unused',
          description: '',
        },
        {
          name: 'OZ',
          unit: 'zero, unused',
          description: '',
        },
        {
          name: 'D',
          unit: 'zero, unused',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rbrh',
    title: 'RBRH',
    description: 'Replay Data Barometer Header',
    tables: [
      [
        {
          name: 'Primary',
          unit: 'primary barometer instance number',
          description: '',
        },
        {
          name: 'NumInst',
          unit: 'number of barometer sensors',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rbri',
    title: 'RBRI',
    description: 'Replay Data Barometer Instance',
    tables: [
      [
        {
          name: 'LastUpdate',
          unit: 'timestamp of barometer data',
          description: '',
        },
        {
          name: 'Alt',
          unit: 'barometer altitude estimate',
          description: '',
        },
        {
          name: 'H',
          unit: 'barometer sensor health indication',
          description: '',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'barometer instance number',
        },
      ],
    ],
  },
  {
    id: 'rcda',
    title: 'RCDA',
    description: 'Raw RC data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'TS',
          unit: 'data arrival timestamp',
          description: '',
        },
        {
          name: 'Prot',
          unit: 'Protocol currently being decoded',
          description: '',
        },
        {
          name: 'Len',
          unit: 'Number of valid bytes in message',
          description: '',
        },
        {
          name: 'U0',
          unit: 'first quartet of bytes',
          description: '',
        },
        {
          name: 'U1',
          unit: 'second quartet of bytes',
          description: '',
        },
        {
          name: 'U2',
          unit: 'third quartet of bytes',
          description: '',
        },
        {
          name: 'U3',
          unit: 'fourth quartet of bytes',
          description: '',
        },
        {
          name: 'U4',
          unit: 'fifth quartet of bytes',
          description: '',
        },
        {
          name: 'U5',
          unit: 'sixth quartet of bytes',
          description: '',
        },
        {
          name: 'U6',
          unit: 'seventh quartet of bytes',
          description: '',
        },
        {
          name: 'U7',
          unit: 'eight quartet of bytes',
          description: '',
        },
        {
          name: 'U8',
          unit: 'ninth quartet of bytes',
          description: '',
        },
        {
          name: 'U9',
          unit: 'tenth quartet of bytes',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rci2',
    title: 'RCI2',
    description: '(More) RC input channels to vehicle',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C15',
          unit: 'us',
          description: 'channel 15 input',
        },
        {
          name: 'C16',
          unit: 'us',
          description: 'channel 16 input',
        },
        {
          name: 'OMask',
          unit: 'bitmask of RC channels being overridden by mavlink input',
          description: '',
        },
        {
          name: 'Flags',
          unit: 'bitmask',
          description: 'bitmask of RC state flags Bitmask values:',
        },
        {
          name: 'HAS_VALID_INPUT',
          unit: '1',
          description: 'true if the system is receiving good RC values',
        },
        {
          name: 'IN_RC_FAILSAFE',
          unit: '2',
          description: 'true if the system is current in RC failsafe',
        },
      ],
      [
        {
          name: 'HAS_VALID_INPUT',
          unit: '1',
          description: 'true if the system is receiving good RC values',
        },
        {
          name: 'IN_RC_FAILSAFE',
          unit: '2',
          description: 'true if the system is current in RC failsafe',
        },
      ],
    ],
  },
  {
    id: 'rcin',
    title: 'RCIN',
    description: 'RC input channels to vehicle',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C1',
          unit: 'us',
          description: 'channel 1 input',
        },
        {
          name: 'C2',
          unit: 'us',
          description: 'channel 2 input',
        },
        {
          name: 'C3',
          unit: 'us',
          description: 'channel 3 input',
        },
        {
          name: 'C4',
          unit: 'us',
          description: 'channel 4 input',
        },
        {
          name: 'C5',
          unit: 'us',
          description: 'channel 5 input',
        },
        {
          name: 'C6',
          unit: 'us',
          description: 'channel 6 input',
        },
        {
          name: 'C7',
          unit: 'us',
          description: 'channel 7 input',
        },
        {
          name: 'C8',
          unit: 'us',
          description: 'channel 8 input',
        },
        {
          name: 'C9',
          unit: 'us',
          description: 'channel 9 input',
        },
        {
          name: 'C10',
          unit: 'us',
          description: 'channel 10 input',
        },
        {
          name: 'C11',
          unit: 'us',
          description: 'channel 11 input',
        },
        {
          name: 'C12',
          unit: 'us',
          description: 'channel 12 input',
        },
        {
          name: 'C13',
          unit: 'us',
          description: 'channel 13 input',
        },
        {
          name: 'C14',
          unit: 'us',
          description: 'channel 14 input',
        },
      ],
    ],
  },
  {
    id: 'rco2',
    title: 'RCO2',
    description: 'Servo channel output values 15 to 18',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C15',
          unit: 'us',
          description: 'channel 15 output',
        },
        {
          name: 'C16',
          unit: 'us',
          description: 'channel 16 output',
        },
        {
          name: 'C17',
          unit: 'us',
          description: 'channel 17 output',
        },
        {
          name: 'C18',
          unit: 'us',
          description: 'channel 18 output',
        },
      ],
    ],
  },
  {
    id: 'rco3',
    title: 'RCO3',
    description: 'Servo channel output values 19 to 32',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C19',
          unit: 'us',
          description: 'channel 19 output',
        },
        {
          name: 'C20',
          unit: 'us',
          description: 'channel 20 output',
        },
        {
          name: 'C21',
          unit: 'us',
          description: 'channel 21 output',
        },
        {
          name: 'C22',
          unit: 'us',
          description: 'channel 22 output',
        },
        {
          name: 'C23',
          unit: 'us',
          description: 'channel 23 output',
        },
        {
          name: 'C24',
          unit: 'us',
          description: 'channel 24 output',
        },
        {
          name: 'C25',
          unit: 'us',
          description: 'channel 25 output',
        },
        {
          name: 'C26',
          unit: 'us',
          description: 'channel 26 output',
        },
        {
          name: 'C27',
          unit: 'us',
          description: 'channel 27 output',
        },
        {
          name: 'C28',
          unit: 'us',
          description: 'channel 28 output',
        },
        {
          name: 'C29',
          unit: 'us',
          description: 'channel 29 output',
        },
        {
          name: 'C30',
          unit: 'us',
          description: 'channel 30 output',
        },
        {
          name: 'C31',
          unit: 'us',
          description: 'channel 31 output',
        },
        {
          name: 'C32',
          unit: 'us',
          description: 'channel 32 output',
        },
      ],
    ],
  },
  {
    id: 'rcou',
    title: 'RCOU',
    description: 'Servo channel output values 1 to 14',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C1',
          unit: 'us',
          description: 'channel 1 output',
        },
        {
          name: 'C2',
          unit: 'us',
          description: 'channel 2 output',
        },
        {
          name: 'C3',
          unit: 'us',
          description: 'channel 3 output',
        },
        {
          name: 'C4',
          unit: 'us',
          description: 'channel 4 output',
        },
        {
          name: 'C5',
          unit: 'us',
          description: 'channel 5 output',
        },
        {
          name: 'C6',
          unit: 'us',
          description: 'channel 6 output',
        },
        {
          name: 'C7',
          unit: 'us',
          description: 'channel 7 output',
        },
        {
          name: 'C8',
          unit: 'us',
          description: 'channel 8 output',
        },
        {
          name: 'C9',
          unit: 'us',
          description: 'channel 9 output',
        },
        {
          name: 'C10',
          unit: 'us',
          description: 'channel 10 output',
        },
        {
          name: 'C11',
          unit: 'us',
          description: 'channel 11 output',
        },
        {
          name: 'C12',
          unit: 'us',
          description: 'channel 12 output',
        },
        {
          name: 'C13',
          unit: 'us',
          description: 'channel 13 output',
        },
        {
          name: 'C14',
          unit: 'us',
          description: 'channel 14 output',
        },
      ],
    ],
  },
  {
    id: 'rely',
    title: 'RELY',
    description: 'Relay state',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Instance',
          unit: 'instance',
          description: 'relay instance number',
        },
        {
          name: 'State',
          unit: 'current state',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'reph',
    title: 'REPH',
    description: 'Replay external position data',
    tables: [
      [
        {
          name: 'PX',
          unit: 'external position estimate, X-axis',
          description: '',
        },
        {
          name: 'PY',
          unit: 'external position estimate, Y-axis',
          description: '',
        },
        {
          name: 'PZ',
          unit: 'external position estimate, Z-axis',
          description: '',
        },
        {
          name: 'Q1',
          unit: 'external attitude quaternion',
          description: '',
        },
        {
          name: 'Q2',
          unit: 'external attitude quaternion',
          description: '',
        },
        {
          name: 'Q3',
          unit: 'external attitude quaternion',
          description: '',
        },
        {
          name: 'Q4',
          unit: 'external attitude quaternion',
          description: '',
        },
        {
          name: 'PEr',
          unit: 'external position error estimate',
          description: '',
        },
        {
          name: 'AEr',
          unit: 'external attitude error estimate',
          description: '',
        },
        {
          name: 'TS',
          unit: 'timestamp on external error estimate',
          description: '',
        },
        {
          name: 'RT',
          unit: 'timestamp of last external reset',
          description: '',
        },
        {
          name: 'D',
          unit: 'delay on external data',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rev2',
    title: 'REV2',
    description: 'Replay Event (EKF2)',
    tables: [
      [
        {
          name: 'Event',
          unit: 'enum',
          description: 'external event injected into EKF Values:',
        },
        {
          name: 'resetGyroBias',
          unit: '0',
          description: '',
        },
        {
          name: 'resetHeightDatum',
          unit: '1',
          description: '',
        },
        {
          name: 'setTerrainHgtStable',
          unit: '9',
          description: '',
        },
        {
          name: 'unsetTerrainHgtStable',
          unit: '10',
          description: '',
        },
        {
          name: 'requestYawReset',
          unit: '11',
          description: '',
        },
        {
          name: 'checkLaneSwitch',
          unit: '12',
          description: '',
        },
        {
          name: 'setSourceSet0',
          unit: '13',
          description: '',
        },
        {
          name: 'setSourceSet1',
          unit: '14',
          description: '',
        },
        {
          name: 'setSourceSet2',
          unit: '15',
          description: '',
        },
      ],
      [
        {
          name: 'resetGyroBias',
          unit: '0',
          description: '',
        },
        {
          name: 'resetHeightDatum',
          unit: '1',
          description: '',
        },
        {
          name: 'setTerrainHgtStable',
          unit: '9',
          description: '',
        },
        {
          name: 'unsetTerrainHgtStable',
          unit: '10',
          description: '',
        },
        {
          name: 'requestYawReset',
          unit: '11',
          description: '',
        },
        {
          name: 'checkLaneSwitch',
          unit: '12',
          description: '',
        },
        {
          name: 'setSourceSet0',
          unit: '13',
          description: '',
        },
        {
          name: 'setSourceSet1',
          unit: '14',
          description: '',
        },
        {
          name: 'setSourceSet2',
          unit: '15',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rev3',
    title: 'REV3',
    description: 'Replay Event (EKF3)',
    tables: [
      [
        {
          name: 'Event',
          unit: 'enum',
          description: 'external event injected into EKF Values:',
        },
        {
          name: 'resetGyroBias',
          unit: '0',
          description: '',
        },
        {
          name: 'resetHeightDatum',
          unit: '1',
          description: '',
        },
        {
          name: 'setTerrainHgtStable',
          unit: '9',
          description: '',
        },
        {
          name: 'unsetTerrainHgtStable',
          unit: '10',
          description: '',
        },
        {
          name: 'requestYawReset',
          unit: '11',
          description: '',
        },
        {
          name: 'checkLaneSwitch',
          unit: '12',
          description: '',
        },
        {
          name: 'setSourceSet0',
          unit: '13',
          description: '',
        },
        {
          name: 'setSourceSet1',
          unit: '14',
          description: '',
        },
        {
          name: 'setSourceSet2',
          unit: '15',
          description: '',
        },
      ],
      [
        {
          name: 'resetGyroBias',
          unit: '0',
          description: '',
        },
        {
          name: 'resetHeightDatum',
          unit: '1',
          description: '',
        },
        {
          name: 'setTerrainHgtStable',
          unit: '9',
          description: '',
        },
        {
          name: 'unsetTerrainHgtStable',
          unit: '10',
          description: '',
        },
        {
          name: 'requestYawReset',
          unit: '11',
          description: '',
        },
        {
          name: 'checkLaneSwitch',
          unit: '12',
          description: '',
        },
        {
          name: 'setSourceSet0',
          unit: '13',
          description: '',
        },
        {
          name: 'setSourceSet1',
          unit: '14',
          description: '',
        },
        {
          name: 'setSourceSet2',
          unit: '15',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'revh',
    title: 'REVH',
    description: 'Replay external velocity data',
    tables: [
      [
        {
          name: 'VX',
          unit: 'external velocity estimate, X-axis',
          description: '',
        },
        {
          name: 'VY',
          unit: 'external velocity estimate, Y-axis',
          description: '',
        },
        {
          name: 'VZ',
          unit: 'external velocity estimate, Z-axis',
          description: '',
        },
        {
          name: 'Er',
          unit: 'error in velocity estimate',
          description: '',
        },
        {
          name: 'TS',
          unit: 'timestamp of velocity estimate',
          description: '',
        },
        {
          name: 'D',
          unit: 'delay in external velocity data',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rey3',
    title: 'REY3',
    description: 'Replay Euler Yaw event',
    tables: [
      [
        {
          name: 'yawangle',
          unit: 'UNKNOWN',
          description: 'externally supplied yaw angle',
        },
        {
          name: 'yawangleerr',
          unit: 'UNKNOWN',
          description: 'error in externally supplied yaw angle',
        },
        {
          name: 'timestamp_ms',
          unit: 'UNKNOWN',
          description: 'timestamp associated with yaw angle and yaw angle error',
        },
        {
          name: 'type',
          unit: 'number that needs documenting',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rfnd',
    title: 'RFND',
    description: 'Rangefinder sensor information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Instance',
          unit: 'instance',
          description: 'rangefinder instance number this data is from',
        },
        {
          name: 'Dist',
          unit: 'm',
          description: 'Reported distance from sensor',
        },
        {
          name: 'Stat',
          unit: 'enum',
          description: 'Sensor state Values:',
        },
        {
          name: 'NotConnected',
          unit: '0',
          description: '',
        },
        {
          name: 'NoData',
          unit: '1',
          description: '',
        },
        {
          name: 'OutOfRangeLow',
          unit: '2',
          description: '',
        },
        {
          name: 'OutOfRangeHigh',
          unit: '3',
          description: '',
        },
        {
          name: 'Good',
          unit: '4',
          description: '',
        },
        {
          name: 'Orient',
          unit: 'enum',
          description: 'Sensor orientation Values:',
        },
        {
          name: 'ROTATION_NONE',
          unit: '0',
          description: '',
        },
        {
          name: 'ROTATION_YAW_45',
          unit: '1',
          description: '',
        },
        {
          name: 'ROTATION_YAW_90',
          unit: '2',
          description: '',
        },
        {
          name: 'ROTATION_YAW_135',
          unit: '3',
          description: '',
        },
        {
          name: 'ROTATION_YAW_180',
          unit: '4',
          description: '',
        },
        {
          name: 'ROTATION_YAW_225',
          unit: '5',
          description: '',
        },
        {
          name: 'ROTATION_YAW_270',
          unit: '6',
          description: '',
        },
        {
          name: 'ROTATION_YAW_315',
          unit: '7',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_180',
          unit: '8',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_180_YAW_45',
          unit: '9',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_180_YAW_90',
          unit: '10',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_180_YAW_135',
          unit: '11',
          description: '',
        },
        {
          name: 'ROTATION_PITCH_180',
          unit: '12',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_180_YAW_225',
          unit: '13',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_180_YAW_270',
          unit: '14',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_180_YAW_315',
          unit: '15',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_90',
          unit: '16',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_90_YAW_45',
          unit: '17',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_90_YAW_90',
          unit: '18',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_90_YAW_135',
          unit: '19',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_270',
          unit: '20',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_270_YAW_45',
          unit: '21',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_270_YAW_90',
          unit: '22',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_270_YAW_135',
          unit: '23',
          description: '',
        },
        {
          name: 'ROTATION_PITCH_90',
          unit: '24',
          description: '',
        },
        {
          name: 'ROTATION_PITCH_270',
          unit: '25',
          description: '',
        },
        {
          name: 'ROTATION_PITCH_180_YAW_90',
          unit: '26',
          description: 'same as ROTATION_ROLL_180_YAW_270',
        },
        {
          name: 'ROTATION_PITCH_180_YAW_270',
          unit: '27',
          description: 'same as ROTATION_ROLL_180_YAW_90',
        },
        {
          name: 'ROTATION_ROLL_90_PITCH_90',
          unit: '28',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_180_PITCH_90',
          unit: '29',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_270_PITCH_90',
          unit: '30',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_90_PITCH_180',
          unit: '31',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_270_PITCH_180',
          unit: '32',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_90_PITCH_270',
          unit: '33',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_180_PITCH_270',
          unit: '34',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_270_PITCH_270',
          unit: '35',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_90_PITCH_180_YAW_90',
          unit: '36',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_90_YAW_270',
          unit: '37',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_90_PITCH_68_YAW_293',
          unit: '38',
          description: 'this is actually, roll 90, pitch 68.8, yaw 293.3',
        },
        {
          name: 'ROTATION_PITCH_315',
          unit: '39',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_90_PITCH_315',
          unit: '40',
          description: '',
        },
        {
          name: 'ROTATION_PITCH_7',
          unit: '41',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_45',
          unit: '42',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_315',
          unit: '43',
          description: '',
        },
        {
          name: 'ROTATION_MAX',
          unit: '44',
          description: '',
        },
        {
          name: 'ROTATION_CUSTOM_OLD',
          unit: '100',
          description: '',
        },
        {
          name: 'ROTATION_CUSTOM_1',
          unit: '101',
          description: '',
        },
        {
          name: 'ROTATION_CUSTOM_2',
          unit: '102',
          description: '',
        },
        {
          name: 'ROTATION_CUSTOM_END',
          unit: '103',
          description: '',
        },
        {
          name: 'Quality',
          unit: '%',
          description: 'Signal quality. -1 means invalid, 0 is no signal, 100 is perfect signal',
        },
      ],
      [
        {
          name: 'NotConnected',
          unit: '0',
          description: '',
        },
        {
          name: 'NoData',
          unit: '1',
          description: '',
        },
        {
          name: 'OutOfRangeLow',
          unit: '2',
          description: '',
        },
        {
          name: 'OutOfRangeHigh',
          unit: '3',
          description: '',
        },
        {
          name: 'Good',
          unit: '4',
          description: '',
        },
      ],
      [
        {
          name: 'ROTATION_NONE',
          unit: '0',
          description: '',
        },
        {
          name: 'ROTATION_YAW_45',
          unit: '1',
          description: '',
        },
        {
          name: 'ROTATION_YAW_90',
          unit: '2',
          description: '',
        },
        {
          name: 'ROTATION_YAW_135',
          unit: '3',
          description: '',
        },
        {
          name: 'ROTATION_YAW_180',
          unit: '4',
          description: '',
        },
        {
          name: 'ROTATION_YAW_225',
          unit: '5',
          description: '',
        },
        {
          name: 'ROTATION_YAW_270',
          unit: '6',
          description: '',
        },
        {
          name: 'ROTATION_YAW_315',
          unit: '7',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_180',
          unit: '8',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_180_YAW_45',
          unit: '9',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_180_YAW_90',
          unit: '10',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_180_YAW_135',
          unit: '11',
          description: '',
        },
        {
          name: 'ROTATION_PITCH_180',
          unit: '12',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_180_YAW_225',
          unit: '13',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_180_YAW_270',
          unit: '14',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_180_YAW_315',
          unit: '15',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_90',
          unit: '16',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_90_YAW_45',
          unit: '17',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_90_YAW_90',
          unit: '18',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_90_YAW_135',
          unit: '19',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_270',
          unit: '20',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_270_YAW_45',
          unit: '21',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_270_YAW_90',
          unit: '22',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_270_YAW_135',
          unit: '23',
          description: '',
        },
        {
          name: 'ROTATION_PITCH_90',
          unit: '24',
          description: '',
        },
        {
          name: 'ROTATION_PITCH_270',
          unit: '25',
          description: '',
        },
        {
          name: 'ROTATION_PITCH_180_YAW_90',
          unit: '26',
          description: 'same as ROTATION_ROLL_180_YAW_270',
        },
        {
          name: 'ROTATION_PITCH_180_YAW_270',
          unit: '27',
          description: 'same as ROTATION_ROLL_180_YAW_90',
        },
        {
          name: 'ROTATION_ROLL_90_PITCH_90',
          unit: '28',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_180_PITCH_90',
          unit: '29',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_270_PITCH_90',
          unit: '30',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_90_PITCH_180',
          unit: '31',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_270_PITCH_180',
          unit: '32',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_90_PITCH_270',
          unit: '33',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_180_PITCH_270',
          unit: '34',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_270_PITCH_270',
          unit: '35',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_90_PITCH_180_YAW_90',
          unit: '36',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_90_YAW_270',
          unit: '37',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_90_PITCH_68_YAW_293',
          unit: '38',
          description: 'this is actually, roll 90, pitch 68.8, yaw 293.3',
        },
        {
          name: 'ROTATION_PITCH_315',
          unit: '39',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_90_PITCH_315',
          unit: '40',
          description: '',
        },
        {
          name: 'ROTATION_PITCH_7',
          unit: '41',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_45',
          unit: '42',
          description: '',
        },
        {
          name: 'ROTATION_ROLL_315',
          unit: '43',
          description: '',
        },
        {
          name: 'ROTATION_MAX',
          unit: '44',
          description: '',
        },
        {
          name: 'ROTATION_CUSTOM_OLD',
          unit: '100',
          description: '',
        },
        {
          name: 'ROTATION_CUSTOM_1',
          unit: '101',
          description: '',
        },
        {
          name: 'ROTATION_CUSTOM_2',
          unit: '102',
          description: '',
        },
        {
          name: 'ROTATION_CUSTOM_END',
          unit: '103',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rfrf',
    title: 'RFRF',
    description: 'Replay FRame data - Finished frame',
    tables: [
      [
        {
          name: 'FTypes',
          unit: 'bitmask',
          description: 'accumulated method calls made during frame Bitmask values:',
        },
        {
          name: 'InitialiseFilterEKF2',
          unit: '1',
          description: '',
        },
        {
          name: 'UpdateFilterEKF2',
          unit: '2',
          description: '',
        },
        {
          name: 'InitialiseFilterEKF3',
          unit: '4',
          description: '',
        },
        {
          name: 'UpdateFilterEKF3',
          unit: '8',
          description: '',
        },
        {
          name: 'LogWriteEKF2',
          unit: '16',
          description: '',
        },
        {
          name: 'LogWriteEKF3',
          unit: '32',
          description: '',
        },
        {
          name: 'Slow',
          unit: 'true if we are not keeping up with IMU loop rate',
          description: '',
        },
      ],
      [
        {
          name: 'InitialiseFilterEKF2',
          unit: '1',
          description: '',
        },
        {
          name: 'UpdateFilterEKF2',
          unit: '2',
          description: '',
        },
        {
          name: 'InitialiseFilterEKF3',
          unit: '4',
          description: '',
        },
        {
          name: 'UpdateFilterEKF3',
          unit: '8',
          description: '',
        },
        {
          name: 'LogWriteEKF2',
          unit: '16',
          description: '',
        },
        {
          name: 'LogWriteEKF3',
          unit: '32',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rfrh',
    title: 'RFRH',
    description: 'Replay FRame Header',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'TF',
          unit: 'Time flying',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rfrn',
    title: 'RFRN',
    description: 'Replay FRame - aNother frame header',
    tables: [
      [
        {
          name: 'HLat',
          unit: '1e-7 deglatitude',
          description: 'home latitude',
        },
        {
          name: 'HLon',
          unit: '1e-7 deglongitude',
          description: 'home latitude',
        },
        {
          name: 'HAlt',
          unit: 'cm',
          description: 'home altitude AMSL',
        },
        {
          name: 'E2T',
          unit: 'EAS to TAS factor',
          description: '',
        },
        {
          name: 'AM',
          unit: 'B',
          description: 'available memory',
        },
        {
          name: 'TX',
          unit: 'deg',
          description: 'AHRS trim X',
        },
        {
          name: 'TY',
          unit: 'deg',
          description: 'AHRS trim Y',
        },
        {
          name: 'TZ',
          unit: 'deg',
          description: 'AHRS trim Z',
        },
        {
          name: 'VC',
          unit: 'AHRS Vehicle Class',
          description: '',
        },
        {
          name: 'EKT',
          unit: 'enum',
          description: 'configured EKF type Values:',
        },
        {
          name: 'EKF2',
          unit: '0',
          description: '',
        },
        {
          name: 'EKF3',
          unit: '1',
          description: '',
        },
        {
          name: 'Flags',
          unit: 'bitmask',
          description: 'bitmask of boolean state Bitmask values:',
        },
        {
          name: 'ARMED',
          unit: '1',
          description: '',
        },
        {
          name: 'UNUSED',
          unit: '2',
          description: '',
        },
        {
          name: 'FLY_FORWARD',
          unit: '4',
          description: '',
        },
        {
          name: 'AHRS_AIRSPEED_SENSOR_ENABLED',
          unit: '8',
          description: '',
        },
        {
          name: 'OPTICALFLOW_ENABLED',
          unit: '16',
          description: '',
        },
        {
          name: 'WHEELENCODER_ENABLED',
          unit: '32',
          description: '',
        },
        {
          name: 'TAKEOFF_EXPECTED',
          unit: '64',
          description: '',
        },
        {
          name: 'TOUCHDOWN_EXPECTED',
          unit: '128',
          description: '',
        },
      ],
      [
        {
          name: 'EKF2',
          unit: '0',
          description: '',
        },
        {
          name: 'EKF3',
          unit: '1',
          description: '',
        },
      ],
      [
        {
          name: 'ARMED',
          unit: '1',
          description: '',
        },
        {
          name: 'UNUSED',
          unit: '2',
          description: '',
        },
        {
          name: 'FLY_FORWARD',
          unit: '4',
          description: '',
        },
        {
          name: 'AHRS_AIRSPEED_SENSOR_ENABLED',
          unit: '8',
          description: '',
        },
        {
          name: 'OPTICALFLOW_ENABLED',
          unit: '16',
          description: '',
        },
        {
          name: 'WHEELENCODER_ENABLED',
          unit: '32',
          description: '',
        },
        {
          name: 'TAKEOFF_EXPECTED',
          unit: '64',
          description: '',
        },
        {
          name: 'TOUCHDOWN_EXPECTED',
          unit: '128',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rgph',
    title: 'RGPH',
    description: 'Replay Data GPS Header',
    tables: [
      [
        {
          name: 'NumInst',
          unit: 'number of GPS sensors',
          description: '',
        },
        {
          name: 'Primary',
          unit: 'instance number of primary sensor',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rgpi',
    title: 'RGPI',
    description: 'Replay Data GPS Instance, infrequently changing data',
    tables: [
      [
        {
          name: 'OX',
          unit: 'antenna body-frame offset, X-axis',
          description: '',
        },
        {
          name: 'OY',
          unit: 'antenna body-frame offset, Y-axis',
          description: '',
        },
        {
          name: 'OZ',
          unit: 'antenna body-frame offset, Z-axis',
          description: '',
        },
        {
          name: 'Lg',
          unit: 'GPS time lag',
          description: '',
        },
        {
          name: 'Flags',
          unit: 'bitmask',
          description: 'various GPS flags Bitmask values:',
        },
        {
          name: 'have_vertical_velocity',
          unit: '1',
          description: '',
        },
        {
          name: 'horizontal_accuracy_returncode',
          unit: '2',
          description: '',
        },
        {
          name: 'vertical_accuracy_returncode',
          unit: '4',
          description: '',
        },
        {
          name: 'get_lag_returncode',
          unit: '8',
          description: '',
        },
        {
          name: 'speed_accuracy_returncode',
          unit: '16',
          description: '',
        },
        {
          name: 'gps_yaw_deg_returncode',
          unit: '32',
          description: '',
        },
        {
          name: 'Stat',
          unit: 'GPS fix status',
          description: '',
        },
        {
          name: 'NSats',
          unit: 'number of satellites GPS is using',
          description: '',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'GPS sensor instance number',
        },
      ],
      [
        {
          name: 'have_vertical_velocity',
          unit: '1',
          description: '',
        },
        {
          name: 'horizontal_accuracy_returncode',
          unit: '2',
          description: '',
        },
        {
          name: 'vertical_accuracy_returncode',
          unit: '4',
          description: '',
        },
        {
          name: 'get_lag_returncode',
          unit: '8',
          description: '',
        },
        {
          name: 'speed_accuracy_returncode',
          unit: '16',
          description: '',
        },
        {
          name: 'gps_yaw_deg_returncode',
          unit: '32',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rgpj',
    title: 'RGPJ',
    description: 'Replay Data GPS Instance - rapidly changing data',
    tables: [
      [
        {
          name: 'TS',
          unit: 'GPS data timestamp',
          description: '',
        },
        {
          name: 'VX',
          unit: 'GPS velocity, North',
          description: '',
        },
        {
          name: 'VY',
          unit: 'GPS velocity, East',
          description: '',
        },
        {
          name: 'VZ',
          unit: 'GPS velocity, Down',
          description: '',
        },
        {
          name: 'SA',
          unit: 'speed accuracy',
          description: '',
        },
        {
          name: 'Y',
          unit: 'GPS yaw',
          description: '',
        },
        {
          name: 'YA',
          unit: 'GPS yaw accuracy',
          description: '',
        },
        {
          name: 'YT',
          unit: 'timestamp of GPS yaw estimate',
          description: '',
        },
        {
          name: 'Lat',
          unit: 'latitude',
          description: '',
        },
        {
          name: 'Lon',
          unit: 'longitude',
          description: '',
        },
        {
          name: 'Alt',
          unit: 'altitude',
          description: '',
        },
        {
          name: 'HA',
          unit: 'horizontal accuracy',
          description: '',
        },
        {
          name: 'VA',
          unit: 'vertical accuracy',
          description: '',
        },
        {
          name: 'HD',
          unit: 'HDOP',
          description: '',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'GPS sensor instance number',
        },
      ],
    ],
  },
  {
    id: 'rich',
    title: 'RICH',
    description: 'Richenpower generator telemetry',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'runTime',
          unit: 'total generator runtime',
          description: '',
        },
        {
          name: 'maintTime',
          unit: 'time until generator requires maintenance',
          description: '',
        },
        {
          name: 'errors',
          unit: 'bitmask of error received from generator',
          description: '',
        },
        {
          name: 'rpm',
          unit: 'current generator RPM',
          description: '',
        },
        {
          name: 'ovolt',
          unit: 'output voltage',
          description: '',
        },
        {
          name: 'ocurr',
          unit: 'output current',
          description: '',
        },
        {
          name: 'mode',
          unit: 'generator mode; idle/run/charge/balance',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rish',
    title: 'RISH',
    description: 'Replay Inertial Sensor header',
    tables: [
      [
        {
          name: 'LR',
          unit: 'INS loop rate',
          description: '',
        },
        {
          name: 'PG',
          unit: 'primary gyro index',
          description: '',
        },
        {
          name: 'PA',
          unit: 'primary accel index',
          description: '',
        },
        {
          name: 'LD',
          unit: 'INS loop-delta-t',
          description: '',
        },
        {
          name: 'AC',
          unit: 'accel count',
          description: '',
        },
        {
          name: 'GC',
          unit: 'gyro count',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'risi',
    title: 'RISI',
    description: 'Replay Inertial Sensor instance data',
    tables: [
      [
        {
          name: 'DVX',
          unit: 'x-axis delta-velocity',
          description: '',
        },
        {
          name: 'DVY',
          unit: 'y-axis delta-velocity',
          description: '',
        },
        {
          name: 'DVZ',
          unit: 'z-axis delta-velocity',
          description: '',
        },
        {
          name: 'DAX',
          unit: 'x-axis delta-angle',
          description: '',
        },
        {
          name: 'DAY',
          unit: 'y-axis delta-angle',
          description: '',
        },
        {
          name: 'DAZ',
          unit: 'z-axis delta-angle',
          description: '',
        },
        {
          name: 'DVDT',
          unit: 'delta-velocity-delta-time',
          description: '',
        },
        {
          name: 'DADT',
          unit: 'delta-angle-delta-time',
          description: '',
        },
        {
          name: 'Flags',
          unit: 'use-accel, use-gyro, delta-vel-valid, delta-accel-valid',
          description: '',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'IMU instance',
        },
      ],
    ],
  },
  {
    id: 'rmgh',
    title: 'RMGH',
    description: 'Replay Data Magnetometer Header',
    tables: [
      [
        {
          name: 'Dec',
          unit: 'vehicle declination',
          description: '',
        },
        {
          name: 'Avail',
          unit: 'true if the compass library is marking itself as available',
          description: '',
        },
        {
          name: 'NumInst',
          unit: 'number of compass instances',
          description: '',
        },
        {
          name: 'AutoDec',
          unit: 'true if compass autodeclination is enabled',
          description: '',
        },
        {
          name: 'NumEna',
          unit: 'number of enabled compass instances',
          description: '',
        },
        {
          name: 'LOE',
          unit: 'true if compass learning of offsets is enabled',
          description: '',
        },
        {
          name: 'C',
          unit: 'true if compasses are consistent',
          description: '',
        },
        {
          name: 'FUsable',
          unit: 'index of first usable compass',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rmgi',
    title: 'RMGI',
    description: 'Replay Data Magnetometer Instance',
    tables: [
      [
        {
          name: 'LU',
          unit: 'last update time for magnetometer data',
          description: '',
        },
        {
          name: 'OX',
          unit: 'mag sensor offset, X-axis',
          description: '',
        },
        {
          name: 'OY',
          unit: 'mag sensor offset, Y-axis',
          description: '',
        },
        {
          name: 'OZ',
          unit: 'mag sensor offset, Z-axis',
          description: '',
        },
        {
          name: 'FX',
          unit: 'field strength, X-axis',
          description: '',
        },
        {
          name: 'FY',
          unit: 'field strength, Y-axis',
          description: '',
        },
        {
          name: 'FZ',
          unit: 'field strength, Z-axis',
          description: '',
        },
        {
          name: 'UFY',
          unit: 'true if compass is being used for yaw',
          description: '',
        },
        {
          name: 'H',
          unit: 'sensor health',
          description: '',
        },
        {
          name: 'HSF',
          unit: 'compass has scale factor',
          description: '',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'magnetometer instance number',
        },
      ],
    ],
  },
  {
    id: 'rofh',
    title: 'ROFH',
    description: 'Replay optical flow data',
    tables: [
      [
        {
          name: 'FX',
          unit: 'raw flow rate, X-axis',
          description: '',
        },
        {
          name: 'FY',
          unit: 'raw flow rate, Y-axis',
          description: '',
        },
        {
          name: 'GX',
          unit: 'gyro rate, X-axis',
          description: '',
        },
        {
          name: 'GY',
          unit: 'gyro rate, Y-axis',
          description: '',
        },
        {
          name: 'Tms',
          unit: 'measurement timestamp',
          description: '',
        },
        {
          name: 'PX',
          unit: 'gyro rate, X-axis',
          description: '',
        },
        {
          name: 'PY',
          unit: 'body-frame offset, Y-axis',
          description: '',
        },
        {
          name: 'PZ',
          unit: 'body-frame offset, Z-axis',
          description: '',
        },
        {
          name: 'HgtOvr',
          unit: 'sensor height override',
          description: '',
        },
        {
          name: 'Qual',
          unit: 'flow quality measurement',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rpm',
    title: 'RPM',
    description: 'Data from RPM sensors',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'Instance',
        },
        {
          name: 'RPM',
          unit: 'rpm',
          description: 'Sensor’s rpm measurement',
        },
        {
          name: 'Qual',
          unit: 'Signal quality',
          description: '',
        },
        {
          name: 'H',
          unit: 'Sensor Health (Bool)',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rrnh',
    title: 'RRNH',
    description: 'Replay Data Rangefinder Header',
    tables: [
      [
        {
          name: 'GCl',
          unit: 'm',
          description: 'rangefinder ground clearance for downward-facing rangefinders',
        },
        {
          name: 'MaxD',
          unit: 'm',
          description: 'rangefinder maximum distance for downward-facing rangefinders',
        },
        {
          name: 'NumSensors',
          unit: 'number of rangefinder instances',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rrni',
    title: 'RRNI',
    description: 'Replay Data Rangefinder Instance',
    tables: [
      [
        {
          name: 'PX',
          unit: 'rangefinder body-frame offset, X-axis',
          description: '',
        },
        {
          name: 'PY',
          unit: 'rangefinder body-frame offset, Y-axis',
          description: '',
        },
        {
          name: 'PZ',
          unit: 'rangefinder body-frame offset, Z-axis',
          description: '',
        },
        {
          name: 'Dist',
          unit: 'm',
          description: 'Measured rangefinder distance',
        },
        {
          name: 'Orient',
          unit: 'orientation',
          description: '',
        },
        {
          name: 'Status',
          unit: 'status',
          description: '',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'rangefinder instance number',
        },
      ],
    ],
  },
  {
    id: 'rsll',
    title: 'RSLL',
    description: 'Replay Set Lat Lng event',
    tables: [
      [
        {
          name: 'Lat',
          unit: '1e-7 deglatitude',
          description: 'latitude',
        },
        {
          name: 'Lng',
          unit: '1e-7 deglongitude',
          description: 'longitude',
        },
        {
          name: 'PosAccSD',
          unit: 'position accuracy, 1-StD',
          description: '',
        },
        {
          name: 'TS',
          unit: 'timestamp of latitude/longitude',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rso2',
    title: 'RSO2',
    description: 'Replay Set Origin event (EKF2)',
    tables: [
      [
        {
          name: 'Lat',
          unit: '1e-7 deglatitude',
          description: 'origin latitude',
        },
        {
          name: 'Lon',
          unit: '1e-7 deglongitude',
          description: 'origin longitude',
        },
        {
          name: 'Alt',
          unit: 'cm',
          description: 'origin altitude',
        },
      ],
    ],
  },
  {
    id: 'rso3',
    title: 'RSO3',
    description: 'Replay Set Origin event (EKF3)',
    tables: [
      [
        {
          name: 'Lat',
          unit: '1e-7 deglatitude',
          description: 'origin latitude',
        },
        {
          name: 'Lon',
          unit: '1e-7 deglongitude',
          description: 'origin longitude',
        },
        {
          name: 'Alt',
          unit: 'cm',
          description: 'origin altitude',
        },
      ],
    ],
  },
  {
    id: 'rssi',
    title: 'RSSI',
    description: 'Received Signal Strength Indicator for RC receiver',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'RXRSSI',
          unit: 'RSSI',
          description: '',
        },
        {
          name: 'RXLQ',
          unit: '%',
          description: 'RX Link Quality',
        },
      ],
    ],
  },
  {
    id: 'rtc',
    title: 'RTC',
    description: 'Information about RTC clock resets',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'old_utc',
          unit: 'μs',
          description: 'old time',
        },
        {
          name: 'new_utc',
          unit: 'μs',
          description: 'new time',
        },
      ],
    ],
  },
  {
    id: 'rtcm',
    title: 'RTCM',
    description: 'GPS atmospheric perturbation data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Chan',
          unit: 'instance',
          description: 'mavlink channel number this data was received on',
        },
        {
          name: 'RTCMId',
          unit: 'ID field from RTCM packet',
          description: '',
        },
        {
          name: 'Len',
          unit: 'RTCM packet length',
          description: '',
        },
        {
          name: 'CRC',
          unit: 'calculated crc32 for the packet',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rter',
    title: 'RTER',
    description: 'Replay Terrain SRTM Altitude',
    tables: [
      [
        {
          name: 'Alt',
          unit: 'm',
          description: 'altitude above origin in meters',
        },
      ],
    ],
  },
  {
    id: 'rvoh',
    title: 'RVOH',
    description: 'Replay Data Visual Odometry data',
    tables: [
      [
        {
          name: 'OX',
          unit: 'offset, x-axis',
          description: '',
        },
        {
          name: 'OY',
          unit: 'offset, y-axis',
          description: '',
        },
        {
          name: 'OZ',
          unit: 'offset, z-axis',
          description: '',
        },
        {
          name: 'Del',
          unit: 'data delay',
          description: '',
        },
        {
          name: 'H',
          unit: 'sensor health',
          description: '',
        },
        {
          name: 'Ena',
          unit: 'sensor enabled',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'rwa2',
    title: 'RWA2',
    description: 'Replay set-default-airspeed event (EKF2)',
    tables: [
      [
        {
          name: 'Airspeed',
          unit: 'm/s',
          description: 'default airspeed',
        },
        {
          name: 'uncertainty',
          unit: 'm/s',
          description: 'uncertainty in default airspeed',
        },
      ],
    ],
  },
  {
    id: 'rwa3',
    title: 'RWA3',
    description: 'Replay set-default-airspeed event (EKF3)',
    tables: [
      [
        {
          name: 'Airspeed',
          unit: 'm/s',
          description: 'default airspeed',
        },
        {
          name: 'Uncertainty',
          unit: 'm/s',
          description: 'uncertainty in default airspeed',
        },
      ],
    ],
  },
  {
    id: 'rwoh',
    title: 'RWOH',
    description: 'Replay wheel odometry data',
    tables: [
      [
        {
          name: 'DA',
          unit: 'delta-angle',
          description: '',
        },
        {
          name: 'DT',
          unit: 'delta-time',
          description: '',
        },
        {
          name: 'TS',
          unit: 'data timestamp',
          description: '',
        },
        {
          name: 'PX',
          unit: 'sensor body-frame offset, x-axis',
          description: '',
        },
        {
          name: 'PY',
          unit: 'sensor body-frame offset, y-axis',
          description: '',
        },
        {
          name: 'PZ',
          unit: 'sensor body-frame offset, z-axis',
          description: '',
        },
        {
          name: 'R',
          unit: 'wheel radius',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'sa',
    title: 'SA',
    description: 'Simple Avoidance messages',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'State',
          unit: 'True if Simple Avoidance is active',
          description: '',
        },
        {
          name: 'DVelX',
          unit: 'm/s',
          description: 'Desired velocity, X-Axis (Velocity before Avoidance)',
        },
        {
          name: 'DVelY',
          unit: 'm/s',
          description: 'Desired velocity, Y-Axis (Velocity before Avoidance)',
        },
        {
          name: 'DVelZ',
          unit: 'm/s',
          description: 'Desired velocity, Z-Axis (Velocity before Avoidance)',
        },
        {
          name: 'MVelX',
          unit: 'm/s',
          description: 'Modified velocity, X-Axis (Velocity after Avoidance)',
        },
        {
          name: 'MVelY',
          unit: 'm/s',
          description: 'Modified velocity, Y-Axis (Velocity after Avoidance)',
        },
        {
          name: 'MVelZ',
          unit: 'm/s',
          description: 'Modified velocity, Z-Axis (Velocity after Avoidance)',
        },
        {
          name: 'Back',
          unit: 'True if vehicle is backing away',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'saf1',
    title: 'SAF1',
    description: 'Simulated Blimp Sin-Angles',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'afx',
          unit: 'sin(x angle)',
          description: '',
        },
        {
          name: 'afy',
          unit: 'sin(y angle)',
          description: '',
        },
        {
          name: 'afz',
          unit: 'sin(z angle)',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'san1',
    title: 'SAN1',
    description: 'Simulated Blimp Angles',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'anx',
          unit: 'x angle',
          description: '',
        },
        {
          name: 'any',
          unit: 'y angle',
          description: '',
        },
        {
          name: 'anz',
          unit: 'z angle',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'san2',
    title: 'SAN2',
    description: 'Simulated Blimp Angles',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'anx',
          unit: 'x earth-frame angle',
          description: '',
        },
        {
          name: 'any',
          unit: 'y earth-frame angle',
          description: '',
        },
        {
          name: 'anz',
          unit: 'z earth-frame angle',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'sba1',
    title: 'SBA1',
    description: 'Simulated Blimp Body-Frame accelerations',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'ax',
          unit: 'x-axis acceleration',
          description: '',
        },
        {
          name: 'ay',
          unit: 'y-axis acceleration',
          description: '',
        },
        {
          name: 'az',
          unit: 'z-axis acceleration',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'sblm',
    title: 'SBLM',
    description: 'Simulated Blimp Rotational Accelerations',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'RAx',
          unit: 'acceleration around X axis',
          description: '',
        },
        {
          name: 'RAy',
          unit: 'acceleration around Y axis',
          description: '',
        },
        {
          name: 'RAz',
          unit: 'acceleration around Z axis',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'sbph',
    title: 'SBPH',
    description: 'Swift Health Data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'CrcError',
          unit: 'Number of packet CRC errors on serial connection',
          description: '',
        },
        {
          name: 'LastInject',
          unit: 'Timestamp of last raw data injection to GPS',
          description: '',
        },
        {
          name: 'IARhyp',
          unit: 'Current number of integer ambiguity hypotheses',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'sbre',
    title: 'SBRE',
    description: 'Swift Time Data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'GWk',
          unit: 'UNKNOWN',
          description: 'GPS week number',
        },
        {
          name: 'GMS',
          unit: 'UNKNOWN',
          description: 'Milliseconds through GPS week',
        },
        {
          name: 'ns_residual',
          unit: 'UNKNOWN',
          description: 'residual of milliseconds rounding in ns',
        },
        {
          name: 'level',
          unit: 'UNKNOWN',
          description: 'GPIO pin levels',
        },
        {
          name: 'quality',
          unit: 'UNKNOWN',
          description: 'time quality',
        },
      ],
    ],
  },
  {
    id: 'scr',
    title: 'SCR',
    description: 'Scripting runtime stats',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Name',
          unit: 'instance',
          description: 'script name',
        },
        {
          name: 'Runtime',
          unit: 'μs',
          description: 'run time',
        },
        {
          name: 'Total_mem',
          unit: 'B',
          description: 'total memory usage of all scripts',
        },
        {
          name: 'Run_mem',
          unit: 'B',
          description: 'run memory usage',
        },
      ],
    ],
  },
  {
    id: 'sctl',
    title: 'SCTL',
    description: 'Simulated Glider Drop control outputs',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'Ail',
          unit: 'aileron output',
          description: '',
        },
        {
          name: 'Elev',
          unit: 'elevator output',
          description: '',
        },
        {
          name: 'Rudd',
          unit: 'rudder output',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'scve',
    title: 'SCVE',
    description: 'Debug message for SCurve internal error',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Sm',
          unit: 'duration of the raised cosine jerk profile',
          description: '',
        },
        {
          name: 'Jm',
          unit: 'maximum value of the raised cosine jerk profile',
          description: '',
        },
        {
          name: 'V0',
          unit: 'initial velocity magnitude',
          description: '',
        },
        {
          name: 'Am',
          unit: 'maximum constant acceleration',
          description: '',
        },
        {
          name: 'Vm',
          unit: 'maximum constant velocity',
          description: '',
        },
        {
          name: 'L',
          unit: 'Length of the path',
          description: '',
        },
        {
          name: 'Jm_out',
          unit: 'maximum value of the raised cosine jerk profile',
          description: '',
        },
        {
          name: 'tj_out',
          unit: 'segment duration',
          description: '',
        },
        {
          name: 't2_out',
          unit: 'segment duration',
          description: '',
        },
        {
          name: 't4_out',
          unit: 'segment duration',
          description: '',
        },
        {
          name: 't6_out',
          unit: 'segment duration',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'sfa1',
    title: 'SFA1',
    description: 'Simulated Blimp Fin Angles',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'f0',
          unit: 'fin 0 angle',
          description: '',
        },
        {
          name: 'f1',
          unit: 'fin 1 angle',
          description: '',
        },
        {
          name: 'f2',
          unit: 'fin 2 angle',
          description: '',
        },
        {
          name: 'f3',
          unit: 'fin 3 angle',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'sfan',
    title: 'SFAN',
    description: 'Simulated Blimp Servo Angles',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'f0',
          unit: 'fin 0 servo angle',
          description: '',
        },
        {
          name: 'f1',
          unit: 'fin 1 servo angle',
          description: '',
        },
        {
          name: 'f2',
          unit: 'fin 2 servo angle',
          description: '',
        },
        {
          name: 'f3',
          unit: 'fin 3 servo angle',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'sfn',
    title: 'SFN',
    description: 'Simulated Blimp Fin Forces',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'n0',
          unit: 'Fin 0 normal force',
          description: '',
        },
        {
          name: 'n1',
          unit: 'Fin 1 normal force',
          description: '',
        },
        {
          name: 'n2',
          unit: 'Fin 2 normal force',
          description: '',
        },
        {
          name: 'n3',
          unit: 'Fin 3 normal force',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'sft',
    title: 'SFT',
    description: 'Simulated Blimp Fin Thrust',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'f0',
          unit: 'Fin 0 tangential thrust',
          description: '',
        },
        {
          name: 'f1',
          unit: 'Fin 1 tangential thrust',
          description: '',
        },
        {
          name: 'f2',
          unit: 'Fin 2 tangential thrust',
          description: '',
        },
        {
          name: 'f3',
          unit: 'Fin 3 tangential thrust',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'sfv1',
    title: 'SFV1',
    description: 'Simulated Blimp Fin Velocities',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'f0',
          unit: 'fin 0 velocity',
          description: '',
        },
        {
          name: 'f1',
          unit: 'fin 1 velocity',
          description: '',
        },
        {
          name: 'f2',
          unit: 'fin 2 velocity',
          description: '',
        },
        {
          name: 'f3',
          unit: 'fin 3 velocity',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'sidd',
    title: 'SIDD',
    description: 'System ID data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Time',
          unit: 's',
          description: 'Time reference for waveform',
        },
        {
          name: 'Targ',
          unit: 'Current waveform sample',
          description: '',
        },
        {
          name: 'F',
          unit: 'Hz',
          description: 'Instantaneous waveform frequency',
        },
        {
          name: 'Gx',
          unit: 'deg/s',
          description: 'Delta angle, X-Axis',
        },
        {
          name: 'Gy',
          unit: 'deg/s',
          description: 'Delta angle, Y-Axis',
        },
        {
          name: 'Gz',
          unit: 'deg/s',
          description: 'Delta angle, Z-Axis',
        },
        {
          name: 'Ax',
          unit: 'm/s/s',
          description: 'Delta velocity, X-Axis',
        },
        {
          name: 'Ay',
          unit: 'm/s/s',
          description: 'Delta velocity, Y-Axis',
        },
        {
          name: 'Az',
          unit: 'm/s/s',
          description: 'Delta velocity, Z-Axis',
        },
      ],
    ],
  },
  {
    id: 'sids',
    title: 'SIDS',
    description: 'System ID settings',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Ax',
          unit: 'The axis which is being excited',
          description: '',
        },
        {
          name: 'Mag',
          unit: 'Magnitude of the chirp waveform',
          description: '',
        },
        {
          name: 'FSt',
          unit: 's',
          description: 'Frequency at the start of chirp',
        },
        {
          name: 'FSp',
          unit: 's',
          description: 'Frequency at the end of chirp',
        },
        {
          name: 'TFin',
          unit: 's',
          description: 'Time to reach maximum amplitude of chirp',
        },
        {
          name: 'TC',
          unit: 's',
          description: 'Time at constant frequency before chirp starts',
        },
        {
          name: 'TR',
          unit: 's',
          description: 'Time taken to complete chirp waveform',
        },
        {
          name: 'TFout',
          unit: 's',
          description: 'Time to reach zero amplitude after chirp finishes',
        },
      ],
    ],
  },
  {
    id: 'sim',
    title: 'SIM',
    description: 'SITL simulator state',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Roll',
          unit: 'deg',
          description: 'Simulated roll',
        },
        {
          name: 'Pitch',
          unit: 'deg',
          description: 'Simulated pitch',
        },
        {
          name: 'Yaw',
          unit: 'degheading',
          description: 'Simulated yaw',
        },
        {
          name: 'Alt',
          unit: 'm',
          description: 'Simulated altitude',
        },
        {
          name: 'Lat',
          unit: 'deglatitude',
          description: 'Simulated latitude',
        },
        {
          name: 'Lng',
          unit: 'deglongitude',
          description: 'Simulated longitude',
        },
        {
          name: 'Q1',
          unit: 'Attitude quaternion component 1',
          description: '',
        },
        {
          name: 'Q2',
          unit: 'Attitude quaternion component 2',
          description: '',
        },
        {
          name: 'Q3',
          unit: 'Attitude quaternion component 3',
          description: '',
        },
        {
          name: 'Q4',
          unit: 'Attitude quaternion component 4',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'sim2',
    title: 'SIM2',
    description: 'Additional simulator state',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'PN',
          unit: 'North position from home',
          description: '',
        },
        {
          name: 'PE',
          unit: 'East position from home',
          description: '',
        },
        {
          name: 'PD',
          unit: 'Down position from home',
          description: '',
        },
        {
          name: 'VN',
          unit: 'Velocity north',
          description: '',
        },
        {
          name: 'VE',
          unit: 'Velocity east',
          description: '',
        },
        {
          name: 'VD',
          unit: 'Velocity down',
          description: '',
        },
        {
          name: 'As',
          unit: 'Airspeed',
          description: '',
        },
        {
          name: 'ASpdU',
          unit: 'Achieved simulation speedup value',
          description: '',
        },
        {
          name: 'UFC',
          unit: 'Number of times simulation paused for serial0 output',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'sitl',
    title: 'SITL',
    description: 'Simulation data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'VN',
          unit: 'Velocity - North component',
          description: '',
        },
        {
          name: 'VE',
          unit: 'Velocity - East component',
          description: '',
        },
        {
          name: 'VD',
          unit: 'Velocity - Down component',
          description: '',
        },
        {
          name: 'AN',
          unit: 'Acceleration - North component',
          description: '',
        },
        {
          name: 'AE',
          unit: 'Acceleration - East component',
          description: '',
        },
        {
          name: 'AD',
          unit: 'Acceleration - Down component',
          description: '',
        },
        {
          name: 'PN',
          unit: 'Position - North component',
          description: '',
        },
        {
          name: 'PE',
          unit: 'Position - East component',
          description: '',
        },
        {
          name: 'PD',
          unit: 'Position - Down component',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'sl2',
    title: 'SL2',
    description: 'More Simulated Glider Dropped Calculations',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'AltFt',
          unit: 'altitude in feet',
          description: '',
        },
        {
          name: 'KEAS',
          unit: 'equivalent airspeed in knots',
          description: '',
        },
        {
          name: 'KTAS',
          unit: 'true airspeed in knots',
          description: '',
        },
        {
          name: 'AD',
          unit: 'air density',
          description: '',
        },
        {
          name: 'Fl',
          unit: 'lift',
          description: '',
        },
        {
          name: 'Fd',
          unit: 'drag',
          description: '',
        },
        {
          name: 'LD',
          unit: 'lift/drag ratio',
          description: '',
        },
        {
          name: 'Elev',
          unit: 'elevator output',
          description: '',
        },
        {
          name: 'Ail',
          unit: 'aileron output',
          description: '',
        },
        {
          name: 'Rud',
          unit: 'rudder output',
          description: '',
        },
        {
          name: 'AoA',
          unit: 'Angle of Attack',
          description: '',
        },
        {
          name: 'SSA',
          unit: 'Side Slip Angle',
          description: '',
        },
        {
          name: 'q',
          unit: 'air pressire',
          description: '',
        },
        {
          name: 'Az',
          unit: 'z-axis body-frame acceleration',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'sld',
    title: 'SLD',
    description: 'Simulated Glider Dropped Calculations',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'AltFt',
          unit: 'altitude in feet',
          description: '',
        },
        {
          name: 'AltM',
          unit: 'altitude in metres',
          description: '',
        },
        {
          name: 'EAS',
          unit: 'equivalent airspeed',
          description: '',
        },
        {
          name: 'TAS',
          unit: 'true airspeed',
          description: '',
        },
        {
          name: 'AD',
          unit: 'air density',
          description: '',
        },
        {
          name: 'Fl',
          unit: 'lift',
          description: '',
        },
        {
          name: 'Fd',
          unit: 'drag',
          description: '',
        },
        {
          name: 'LD',
          unit: 'lift/drag ratio',
          description: '',
        },
        {
          name: 'Elev',
          unit: 'elevator output',
          description: '',
        },
        {
          name: 'AoA',
          unit: 'angle of attack',
          description: '',
        },
        {
          name: 'Fx',
          unit: 'X-axis force',
          description: '',
        },
        {
          name: 'Fy',
          unit: 'Y-axis force',
          description: '',
        },
        {
          name: 'Fz',
          unit: 'Z-axis force',
          description: '',
        },
        {
          name: 'q',
          unit: 'air pressure',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'slup',
    title: 'SLUP',
    description: 'Slung payload',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Land',
          unit: '1 if payload is landed, 0 otherwise',
          description: '',
        },
        {
          name: 'Tens',
          unit: '1e2 %',
          description: 'Tension ratio, 1 if line is taut, 0 if slack',
        },
        {
          name: 'Len',
          unit: 'm',
          description: 'Line length',
        },
        {
          name: 'PN',
          unit: 'm',
          description: 'Payload position as offset from vehicle in North direction',
        },
        {
          name: 'PE',
          unit: 'm',
          description: 'Payload position as offset from vehicle in East direction',
        },
        {
          name: 'PD',
          unit: 'm',
          description: 'Payload position as offset from vehicle in Down direction',
        },
        {
          name: 'VN',
          unit: 'm/s',
          description: 'Payload velocity in North direction',
        },
        {
          name: 'VE',
          unit: 'm/s',
          description: 'Payload velocity in East direction',
        },
        {
          name: 'VD',
          unit: 'm/s',
          description: 'Payload velocity in Down direction',
        },
        {
          name: 'AN',
          unit: 'm/s/s',
          description: 'Payload acceleration in North direction',
        },
        {
          name: 'AE',
          unit: 'm/s/s',
          description: 'Payload acceleration in East direction',
        },
        {
          name: 'AD',
          unit: 'm/s/s',
          description: 'Payload acceleration in Down direction',
        },
        {
          name: 'VFN',
          unit: 'Force on vehicle in North direction',
          description: '',
        },
        {
          name: 'VFE',
          unit: 'Force on vehicle in East direction',
          description: '',
        },
        {
          name: 'VFD',
          unit: 'Force on vehicle in Down direction',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'slv1',
    title: 'SLV1',
    description: 'Log data received from JSON simulator 1',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup (us)',
        },
        {
          name: 'Instance',
          unit: 'instance',
          description: 'Slave instance',
        },
        {
          name: 'magic',
          unit: 'magic JSON protocol key',
          description: '',
        },
        {
          name: 'frame_rate',
          unit: 'Slave instance’s desired frame rate',
          description: '',
        },
        {
          name: 'frame_count',
          unit: 'Slave instance’s current frame count',
          description: '',
        },
        {
          name: 'active',
          unit: '1 if the servo outputs are being used from this instance',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'slv2',
    title: 'SLV2',
    description: 'Log data received from JSON simulator 2',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Instance',
          unit: 'instance',
          description: 'Slave instance',
        },
        {
          name: 'C1',
          unit: 'us',
          description: 'channel 1 output',
        },
        {
          name: 'C2',
          unit: 'us',
          description: 'channel 2 output',
        },
        {
          name: 'C3',
          unit: 'us',
          description: 'channel 3 output',
        },
        {
          name: 'C4',
          unit: 'us',
          description: 'channel 4 output',
        },
        {
          name: 'C5',
          unit: 'us',
          description: 'channel 5 output',
        },
        {
          name: 'C6',
          unit: 'us',
          description: 'channel 6 output',
        },
        {
          name: 'C7',
          unit: 'us',
          description: 'channel 7 output',
        },
        {
          name: 'C8',
          unit: 'us',
          description: 'channel 8 output',
        },
        {
          name: 'C9',
          unit: 'us',
          description: 'channel 9 output',
        },
        {
          name: 'C10',
          unit: 'us',
          description: 'channel 10 output',
        },
        {
          name: 'C11',
          unit: 'us',
          description: 'channel 11 output',
        },
        {
          name: 'C12',
          unit: 'us',
          description: 'channel 12 output',
        },
        {
          name: 'C13',
          unit: 'us',
          description: 'channel 13 output',
        },
        {
          name: 'C14',
          unit: 'us',
          description: 'channel 14 output',
        },
      ],
    ],
  },
  {
    id: 'smgc',
    title: 'SMGC',
    description: 'Simulated Blimp Mass and COG',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'm',
          unit: 'mass',
          description: '',
        },
        {
          name: 'g',
          unit: 'gravity',
          description: '',
        },
        {
          name: 'cz',
          unit: 'centre-of-gravity, z-axis',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'smoo',
    title: 'SMOO',
    description: 'Smoothed sensor data fed to EKF to avoid inconsistencies',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'AEx',
          unit: 'Angular Velocity (around x-axis)',
          description: '',
        },
        {
          name: 'AEy',
          unit: 'Angular Velocity (around y-axis)',
          description: '',
        },
        {
          name: 'AEz',
          unit: 'Angular Velocity (around z-axis)',
          description: '',
        },
        {
          name: 'DPx',
          unit: 'Velocity (along x-axis)',
          description: '',
        },
        {
          name: 'DPy',
          unit: 'Velocity (along y-axis)',
          description: '',
        },
        {
          name: 'DPz',
          unit: 'Velocity (along z-axis)',
          description: '',
        },
        {
          name: 'R',
          unit: 'Roll',
          description: '',
        },
        {
          name: 'P',
          unit: 'Pitch',
          description: '',
        },
        {
          name: 'Y',
          unit: 'Yaw',
          description: '',
        },
        {
          name: 'R2',
          unit: 'DCM Roll',
          description: '',
        },
        {
          name: 'P2',
          unit: 'DCM Pitch',
          description: '',
        },
        {
          name: 'Y2',
          unit: 'DCM Yaw',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'smvz',
    title: 'SMVZ',
    description: 'Simulated Volz servo information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Id',
          unit: 'instance',
          description: 'Volz servo ID',
        },
        {
          name: 'Pos',
          unit: '%',
          description: 'Current Simulated Position',
        },
        {
          name: 'DesPos',
          unit: '%',
          description: 'Desired Simulated Position',
        },
        {
          name: 'V',
          unit: 'V',
          description: 'simulated servo voltage',
        },
        {
          name: 'A',
          unit: 'A',
          description: 'simulated servo current',
        },
        {
          name: 'PCBT',
          unit: 'degC',
          description: 'simulated PCB Temperature',
        },
        {
          name: 'MotT',
          unit: 'degC',
          description: 'simulated motor Temperature',
        },
      ],
    ],
  },
  {
    id: 'soar',
    title: 'SOAR',
    description: 'Logged data from soaring feature',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'microseconds since system startup',
          description: '',
        },
        {
          name: 'nettorate',
          unit: 'Estimate of vertical speed of surrounding airmass',
          description: '',
        },
        {
          name: 'x0',
          unit: 'Thermal strength estimate',
          description: '',
        },
        {
          name: 'x1',
          unit: 'Thermal radius estimate',
          description: '',
        },
        {
          name: 'x2',
          unit: 'Thermal position estimate north from home',
          description: '',
        },
        {
          name: 'x3',
          unit: 'Thermal position estimate east from home',
          description: '',
        },
        {
          name: 'north',
          unit: 'Aircraft position north from home',
          description: '',
        },
        {
          name: 'east',
          unit: 'Aircraft position east from home',
          description: '',
        },
        {
          name: 'alt',
          unit: 'Aircraft altitude',
          description: '',
        },
        {
          name: 'dx_w',
          unit: 'Wind speed north',
          description: '',
        },
        {
          name: 'dy_w',
          unit: 'Wind speed east',
          description: '',
        },
        {
          name: 'th',
          unit: 'Estimate of achievable climbrate in thermal',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'sorc',
    title: 'SORC',
    description: 'Soaring Cruise-phase data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'wx',
          unit: 'body-frame wind estimate, x-axis',
          description: '',
        },
        {
          name: 'wz',
          unit: 'body-frame wind estimate, z-axis',
          description: '',
        },
        {
          name: 'wexp',
          unit: 'estimated thermal vertical speed',
          description: '',
        },
        {
          name: 'CLmin',
          unit: 'expected climb-rate lower-limit',
          description: '',
        },
        {
          name: 'CLmax',
          unit: 'expected climb-rate upper-limit',
          description: '',
        },
        {
          name: 'Vopt',
          unit: 'calculated optimal speed to fly',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'srt1',
    title: 'SRT1',
    description: 'Simulated Blimp Rotational forces',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'rtx',
          unit: 'zero',
          description: '',
        },
        {
          name: 'rty',
          unit: 'zero',
          description: '',
        },
        {
          name: 'rtz',
          unit: 'zero',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'srt2',
    title: 'SRT2',
    description: 'Transformed Simulated Blimp Rotational forces',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'rtx',
          unit: 'x-axis wobble rotational force',
          description: '',
        },
        {
          name: 'rty',
          unit: 'y-axis wobble rotational force',
          description: '',
        },
        {
          name: 'rtz',
          unit: 'z-axis wobble rotational force',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'srt3',
    title: 'SRT3',
    description: 'Simulated Blimp Torques',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'rtx',
          unit: 'torque around x axis',
          description: '',
        },
        {
          name: 'rty',
          unit: 'torque around y axis',
          description: '',
        },
        {
          name: 'rtz',
          unit: 'torque around z axis',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'srtl',
    title: 'SRTL',
    description: 'SmartRTL statistics',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Active',
          unit: 'true if SmartRTL could be used right now',
          description: '',
        },
        {
          name: 'NumPts',
          unit: 'number of points currently in use',
          description: '',
        },
        {
          name: 'MaxPts',
          unit: 'maximum number of points that could be used',
          description: '',
        },
        {
          name: 'Action',
          unit: 'enum',
          description: 'most recent internal action taken by SRTL library Values:',
        },
        {
          name: 'POINT_ADD',
          unit: '0',
          description: '',
        },
        {
          name: 'POINT_PRUNE',
          unit: '1',
          description: '',
        },
        {
          name: 'POINT_SIMPLIFY',
          unit: '2',
          description: '',
        },
        {
          name: 'ADD_FAILED_NO_SEMAPHORE',
          unit: '3',
          description: '',
        },
        {
          name: 'ADD_FAILED_PATH_FULL',
          unit: '4',
          description: '',
        },
        {
          name: 'POP_FAILED_NO_SEMAPHORE',
          unit: '5',
          description: '',
        },
        {
          name: 'PEEK_FAILED_NO_SEMAPHORE',
          unit: '6',
          description: '',
        },
        {
          name: 'DEACTIVATED_INIT_FAILED',
          unit: '7',
          description: '',
        },
        {
          name: 'DEACTIVATED_BAD_POSITION_TIMEOUT',
          unit: '9',
          description: '',
        },
        {
          name: 'DEACTIVATED_PATH_FULL_TIMEOUT',
          unit: '10',
          description: '',
        },
        {
          name: 'DEACTIVATED_PROGRAM_ERROR',
          unit: '11',
          description: '',
        },
        {
          name: 'N',
          unit: 'm',
          description: 'point associated with most recent action (North component)',
        },
        {
          name: 'E',
          unit: 'm',
          description: 'point associated with most recent action (East component)',
        },
        {
          name: 'D',
          unit: 'm',
          description: 'point associated with most recent action (Down component)',
        },
      ],
      [
        {
          name: 'POINT_ADD',
          unit: '0',
          description: '',
        },
        {
          name: 'POINT_PRUNE',
          unit: '1',
          description: '',
        },
        {
          name: 'POINT_SIMPLIFY',
          unit: '2',
          description: '',
        },
        {
          name: 'ADD_FAILED_NO_SEMAPHORE',
          unit: '3',
          description: '',
        },
        {
          name: 'ADD_FAILED_PATH_FULL',
          unit: '4',
          description: '',
        },
        {
          name: 'POP_FAILED_NO_SEMAPHORE',
          unit: '5',
          description: '',
        },
        {
          name: 'PEEK_FAILED_NO_SEMAPHORE',
          unit: '6',
          description: '',
        },
        {
          name: 'DEACTIVATED_INIT_FAILED',
          unit: '7',
          description: '',
        },
        {
          name: 'DEACTIVATED_BAD_POSITION_TIMEOUT',
          unit: '9',
          description: '',
        },
        {
          name: 'DEACTIVATED_PATH_FULL_TIMEOUT',
          unit: '10',
          description: '',
        },
        {
          name: 'DEACTIVATED_PROGRAM_ERROR',
          unit: '11',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'ssan',
    title: 'SSAN',
    description: 'Simulated Blimp Servo Inputs',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'f0',
          unit: 'fin 0 servo angle input',
          description: '',
        },
        {
          name: 'f1',
          unit: 'fin 1 servo angle input',
          description: '',
        },
        {
          name: 'f2',
          unit: 'fin 2 servo angle input',
          description: '',
        },
        {
          name: 'f3',
          unit: 'fin 3 servo angle input',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'stak',
    title: 'STAK',
    description: 'Stack information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Id',
          unit: 'instance',
          description: 'thread ID',
        },
        {
          name: 'Pri',
          unit: 'thread priority',
          description: '',
        },
        {
          name: 'Total',
          unit: 'total stack',
          description: '',
        },
        {
          name: 'Free',
          unit: 'free stack',
          description: '',
        },
        {
          name: 'Name',
          unit: 'char[16]',
          description: 'thread name',
        },
      ],
    ],
  },
  {
    id: 'stat',
    title: 'STAT',
    description: 'Current status of the aircraft',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'isFlying',
          unit: 'True if aircraft is probably flying',
          description: '',
        },
        {
          name: 'isFlyProb',
          unit: 'Probability that the aircraft is flying',
          description: '',
        },
        {
          name: 'Armed',
          unit: 'Arm status of the aircraft',
          description: '',
        },
        {
          name: 'Safety',
          unit: 'State of the safety switch',
          description: '',
        },
        {
          name: 'Crash',
          unit: 'True if crash is detected',
          description: '',
        },
        {
          name: 'Still',
          unit: 'True when vehicle is not moving in any axis',
          description: '',
        },
        {
          name: 'Stage',
          unit: 'Current stage of the flight',
          description: '',
        },
        {
          name: 'Hit',
          unit: 'True if impact is detected',
          description: '',
        },
        {
          name: 'Sup',
          unit: 'True if throttle is suppressed',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'swsh',
    title: 'SWSH',
    description: 'Helicopter swashplate logging',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'Swashplate instance',
        },
        {
          name: 'Col',
          unit: 'deg',
          description: 'Blade pitch angle contribution from collective',
        },
        {
          name: 'TCyc',
          unit: 'deg',
          description: 'Total blade pitch angle contribution from cyclic',
        },
        {
          name: 'PCyc',
          unit: 'deg',
          description: 'Blade pitch angle contribution from pitch cyclic',
        },
        {
          name: 'RCyc',
          unit: 'deg',
          description: 'Blade pitch angle contribution from roll cyclic',
        },
      ],
    ],
  },
  {
    id: 'tclr',
    title: 'TCLR',
    description: 'Temperature Calibration Information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'temperature calibration instance number',
        },
        {
          name: 'SType',
          unit: 'sensor type (0==accel, 1==gyro)',
          description: '',
        },
        {
          name: 'Temp',
          unit: 'current temperature',
          description: '',
        },
        {
          name: 'X',
          unit: 'x-axis sample sum',
          description: '',
        },
        {
          name: 'Y',
          unit: 'y-axis sample sum',
          description: '',
        },
        {
          name: 'Z',
          unit: 'z-axis sample sum',
          description: '',
        },
        {
          name: 'NSamp',
          unit: 'sample count',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'tec2',
    title: 'TEC2',
    description: 'Additional information about the Total Energy Control System',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'PEW',
          unit: 'Potential energy weighting',
          description: '',
        },
        {
          name: 'KEW',
          unit: 'Kinetic energy weighting',
          description: '',
        },
        {
          name: 'EBD',
          unit: 'Energy balance demand',
          description: '',
        },
        {
          name: 'EBE',
          unit: 'Energy balance estimate',
          description: '',
        },
        {
          name: 'EBDD',
          unit: 'Energy balance rate demand',
          description: '',
        },
        {
          name: 'EBDE',
          unit: 'Energy balance rate estimate',
          description: '',
        },
        {
          name: 'EBDDT',
          unit: 'Energy balance rate demand + Energy balance rate error*pitch_damping',
          description: '',
        },
        {
          name: 'Imin',
          unit: 'Minimum integrator value',
          description: '',
        },
        {
          name: 'Imax',
          unit: 'Maximum integrator value',
          description: '',
        },
        {
          name: 'I',
          unit: 'Energy balance error integral',
          description: '',
        },
        {
          name: 'KI',
          unit: 'Pitch demand kinetic energy integral',
          description: '',
        },
        {
          name: 'tmin',
          unit: 'Throttle min',
          description: '',
        },
        {
          name: 'tmax',
          unit: 'Throttle max',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'tec3',
    title: 'TEC3',
    description: 'Additional additional information about the Total Energy Control System',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'KED',
          unit: 'Kinetic Energy Dot (1st derivative of KE)',
          description: '',
        },
        {
          name: 'PED',
          unit: 'Potential Energy Dot (1st derivative of PE)',
          description: '',
        },
        {
          name: 'KEDD',
          unit: 'Kinetic Energy Dot Demand',
          description: '',
        },
        {
          name: 'PEDD',
          unit: 'Potential Energy Dot Demand',
          description: '',
        },
        {
          name: 'TEE',
          unit: 'Total energy error',
          description: '',
        },
        {
          name: 'TEDE',
          unit: 'Total energy dot error (1st derivative of total energy error)',
          description: '',
        },
        {
          name: 'FFT',
          unit: 'feed-forward throttle',
          description: '',
        },
        {
          name: 'Imin',
          unit: 'integrator limit based on throttle values',
          description: '',
        },
        {
          name: 'Imax',
          unit: 'integrator limit based on throttle values',
          description: '',
        },
        {
          name: 'I',
          unit: 'integrator state for throttle',
          description: '',
        },
        {
          name: 'Emin',
          unit: 'lower limit for potential energy error',
          description: '',
        },
        {
          name: 'Emax',
          unit: 'upper limit for potential energy error',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'tec4',
    title: 'TEC4',
    description:
      'Additional additional additional information about the Total Energy Control System',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'P',
          unit: 'estimate of potential energy',
          description: '',
        },
        {
          name: 'K',
          unit: 'estimate of kinetic energy',
          description: '',
        },
        {
          name: 'Pdem',
          unit: 'demanded potential energy',
          description: '',
        },
        {
          name: 'Kdem',
          unit: 'demanded kinetic energy',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'tecs',
    title: 'TECS',
    description: 'Information about the Total Energy Control System',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'h',
          unit: 'm',
          description: 'height estimate (UP) currently in use by TECS',
        },
        {
          name: 'dh',
          unit: 'm/s',
          description: 'current climb rate (“delta-height”)',
        },
        {
          name: 'hin',
          unit: 'm',
          description: 'height demand received by TECS',
        },
        {
          name: 'hdem',
          unit: 'm',
          description:
            'height demand after rate limiting and filtering that TECS is currently trying to achieve',
        },
        {
          name: 'dhdem',
          unit: 'm/s',
          description: 'climb rate TECS is currently trying to achieve',
        },
        {
          name: 'spdem',
          unit: 'm/s',
          description: 'True AirSpeed TECS is currently trying to achieve',
        },
        {
          name: 'sp',
          unit: 'm/s',
          description: 'current estimated True AirSpeed',
        },
        {
          name: 'dsp',
          unit: 'm/s',
          description: 'x-axis acceleration estimate (“delta-speed”)',
        },
        {
          name: 'th',
          unit: 'throttle output',
          description: '',
        },
        {
          name: 'ph',
          unit: 'pitch output',
          description: '',
        },
        {
          name: 'pmin',
          unit: 'pitch lower limit',
          description: '',
        },
        {
          name: 'pmax',
          unit: 'pitch upper limit',
          description: '',
        },
        {
          name: 'dspdem',
          unit: 'demanded acceleration output (“delta-speed demand”)',
          description: '',
        },
        {
          name: 'f',
          unit: 'bitmask',
          description: 'flags Bitmask values:',
        },
        {
          name: 'Underspeed',
          unit: '1',
          description: '',
        },
        {
          name: 'UnachievableDescent',
          unit: '2',
          description: '',
        },
        {
          name: 'AutoLanding',
          unit: '4',
          description: '',
        },
        {
          name: 'ReachedTakeoffSpd',
          unit: '8',
          description: '',
        },
      ],
      [
        {
          name: 'Underspeed',
          unit: '1',
          description: '',
        },
        {
          name: 'UnachievableDescent',
          unit: '2',
          description: '',
        },
        {
          name: 'AutoLanding',
          unit: '4',
          description: '',
        },
        {
          name: 'ReachedTakeoffSpd',
          unit: '8',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'temp',
    title: 'TEMP',
    description: 'Temperature Sensor Data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Instance',
          unit: 'instance',
          description: 'temperature sensor instance',
        },
        {
          name: 'Temp',
          unit: 'degC',
          description: 'temperature',
        },
      ],
    ],
  },
  {
    id: 'terr',
    title: 'TERR',
    description: 'Terrain database information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Status',
          unit: 'enum',
          description: 'Terrain database status Values:',
        },
        {
          name: 'TerrainStatusDisabled',
          unit: '0',
          description: 'not enabled',
        },
        {
          name: 'TerrainStatusUnhealthy',
          unit: '1',
          description: 'no terrain data for current location',
        },
        {
          name: 'TerrainStatusOK',
          unit: '2',
          description: 'terrain data available',
        },
        {
          name: 'Lat',
          unit: 'deglatitude',
          description: 'Current vehicle latitude',
        },
        {
          name: 'Lng',
          unit: 'deglongitude',
          description: 'Current vehicle longitude',
        },
        {
          name: 'Spacing',
          unit: 'terrain Tile spacing',
          description: '',
        },
        {
          name: 'TerrH',
          unit: 'm',
          description: 'current Terrain height',
        },
        {
          name: 'CHeight',
          unit: 'm',
          description: 'Vehicle height above terrain',
        },
        {
          name: 'Pending',
          unit: 'Number of tile requests outstanding',
          description: '',
        },
        {
          name: 'Loaded',
          unit: 'Number of tiles in memory',
          description: '',
        },
        {
          name: 'ROfs',
          unit: 'm',
          description: 'terrain reference offset for arming altitude',
        },
      ],
      [
        {
          name: 'TerrainStatusDisabled',
          unit: '0',
          description: 'not enabled',
        },
        {
          name: 'TerrainStatusUnhealthy',
          unit: '1',
          description: 'no terrain data for current location',
        },
        {
          name: 'TerrainStatusOK',
          unit: '2',
          description: 'terrain data available',
        },
      ],
    ],
  },
  {
    id: 'teth',
    title: 'TETH',
    description: 'Tether state',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Len',
          unit: 'Tether length',
          description: '',
        },
        {
          name: 'VFN',
          unit: 'Force on vehicle in North direction',
          description: '',
        },
        {
          name: 'VFE',
          unit: 'Force on vehicle in East direction',
          description: '',
        },
        {
          name: 'VFD',
          unit: 'Force on vehicle in Down direction',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'tilt',
    title: 'TILT',
    description: 'Tiltrotor tilt values',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Tilt',
          unit: 'deg',
          description: 'Current tilt angle, 0 deg vertical, 90 deg horizontal',
        },
        {
          name: 'FL',
          unit: 'deg',
          description: 'Front left tilt angle, 0 deg vertical, 90 deg horizontal',
        },
        {
          name: 'FR',
          unit: 'deg',
          description: 'Front right tilt angle, 0 deg vertical, 90 deg horizontal',
        },
      ],
    ],
  },
  {
    id: 'trig',
    title: 'TRIG',
    description: 'Camera shutter information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'Instance number',
        },
        {
          name: 'Img',
          unit: 'Image number',
          description: '',
        },
        {
          name: 'GPSTime',
          unit: 'milliseconds since start of GPS week',
          description: '',
        },
        {
          name: 'GPSWeek',
          unit: 'weeks since 5 Jan 1980',
          description: '',
        },
        {
          name: 'Lat',
          unit: 'deglatitude',
          description: 'current latitude',
        },
        {
          name: 'Lng',
          unit: 'deglongitude',
          description: 'current longitude',
        },
        {
          name: 'Alt',
          unit: 'm',
          description: 'current altitude',
        },
        {
          name: 'RelAlt',
          unit: 'm',
          description: 'current altitude relative to home',
        },
        {
          name: 'GPSAlt',
          unit: 'm',
          description: 'altitude as reported by GPS',
        },
        {
          name: 'R',
          unit: 'deg',
          description: 'current vehicle roll',
        },
        {
          name: 'P',
          unit: 'deg',
          description: 'current vehicle pitch',
        },
        {
          name: 'Y',
          unit: 'deg',
          description: 'current vehicle yaw',
        },
      ],
    ],
  },
  {
    id: 'trmp',
    title: 'TRMP',
    description: 'Torqeedo Motor Param',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'instance',
        },
        {
          name: 'RPM',
          unit: 'rpm',
          description: 'Motor RPM',
        },
        {
          name: 'Pow',
          unit: 'Watt',
          description: 'Motor power',
        },
        {
          name: 'Volt',
          unit: 'V',
          description: 'Motor voltage',
        },
        {
          name: 'Cur',
          unit: 'A',
          description: 'Motor current',
        },
        {
          name: 'ETemp',
          unit: 'degC',
          description: 'ESC Temp',
        },
        {
          name: 'MTemp',
          unit: 'degC',
          description: 'Motor Temp',
        },
      ],
    ],
  },
  {
    id: 'trms',
    title: 'TRMS',
    description: 'Torqeedo Motor Status',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'instance',
        },
        {
          name: 'State',
          unit: 'Motor status flags',
          description: '',
        },
        {
          name: 'Err',
          unit: 'Motor error flags',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'trqd',
    title: 'TRQD',
    description: 'Torqeedo Status',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'instance',
        },
        {
          name: 'Health',
          unit: 'Health',
          description: '',
        },
        {
          name: 'DesMotSpeed',
          unit: 'Desired Motor Speed (-1000 to 1000)',
          description: '',
        },
        {
          name: 'MotSpeed',
          unit: 'Motor Speed (-1000 to 1000)',
          description: '',
        },
        {
          name: 'SuccCnt',
          unit: 'Success Count',
          description: '',
        },
        {
          name: 'ErrCnt',
          unit: 'Error Count',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'trse',
    title: 'TRSE',
    description: 'Torqeedo System Setup',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'instance',
        },
        {
          name: 'Flag',
          unit: 'Flags',
          description: '',
        },
        {
          name: 'MotType',
          unit: 'Motor type',
          description: '',
        },
        {
          name: 'MotVer',
          unit: 'Motor software version',
          description: '',
        },
        {
          name: 'BattCap',
          unit: 'Ah',
          description: 'Battery capacity',
        },
        {
          name: 'BattPct',
          unit: '%',
          description: 'Battery charge percentage',
        },
        {
          name: 'BattType',
          unit: 'Battery type',
          description: '',
        },
        {
          name: 'SwVer',
          unit: 'Master software version',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'trst',
    title: 'TRST',
    description: 'Torqeedo System State',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'instance',
        },
        {
          name: 'F',
          unit: 'Flags bitmask',
          description: '',
        },
        {
          name: 'Err',
          unit: 'Master error code',
          description: '',
        },
        {
          name: 'MVolt',
          unit: 'V',
          description: 'Motor voltage',
        },
        {
          name: 'MCur',
          unit: 'A',
          description: 'Motor current',
        },
        {
          name: 'Pow',
          unit: 'Watt',
          description: 'Motor power',
        },
        {
          name: 'RPM',
          unit: 'rpm',
          description: 'Motor RPM',
        },
        {
          name: 'MTemp',
          unit: 'degC',
          description: 'Motor Temp (higher of pcb or stator)',
        },
        {
          name: 'BPct',
          unit: '%',
          description: 'Battery charge percentage',
        },
        {
          name: 'BVolt',
          unit: 'V',
          description: 'Battery voltage',
        },
        {
          name: 'BCur',
          unit: 'A',
          description: 'Battery current',
        },
      ],
    ],
  },
  {
    id: 'tsit',
    title: 'TSIT',
    description: 'tailsitter speed scailing values',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Ts',
          unit: 'throttle scaling used for tilt motors',
          description: '',
        },
        {
          name: 'Ss',
          unit: 'speed scailing used for control surfaces method from Q_TAILSIT_GSCMSK',
          description: '',
        },
        {
          name: 'Tmin',
          unit:
            'minimum output throttle calculated from disk thoery gain scale with Q_TAILSIT_MIN_VO',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'tsyn',
    title: 'TSYN',
    description: 'Time synchronisation response information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'SysID',
          unit: 'system ID this data is for',
          description: '',
        },
        {
          name: 'RTT',
          unit: 'μs',
          description: 'round trip time for this system',
        },
      ],
    ],
  },
  {
    id: 'uart',
    title: 'UART',
    description: 'UART stats',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'instance',
        },
        {
          name: 'Tx',
          unit: 'B/s',
          description: 'transmitted data rate bytes per second',
        },
        {
          name: 'Rx',
          unit: 'B/s',
          description:
            'received data rate bytes per second, this is all incoming data, it may not all be processed by the driver using this port.',
        },
        {
          name: 'RxDp',
          unit: 'B/s',
          description:
            'Data rate of dropped received bytes, ideally should be 0. This is the difference between the received data rate and the processed data rate.',
        },
      ],
    ],
  },
  {
    id: 'ubx1',
    title: 'UBX1',
    description: 'uBlox-specific GPS information (part 1)',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Instance',
          unit: 'instance',
          description: 'GPS instance number',
        },
        {
          name: 'noisePerMS',
          unit: 'noise level as measured by GPS',
          description: '',
        },
        {
          name: 'jamInd',
          unit: 'jamming indicator; higher is more likely jammed',
          description: '',
        },
        {
          name: 'aPower',
          unit: 'antenna power indicator; 2 is don’t know',
          description: '',
        },
        {
          name: 'agcCnt',
          unit: 'automatic gain control monitor',
          description: '',
        },
        {
          name: 'config',
          unit: 'bitmask for messages which haven’t been seen',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'ubx2',
    title: 'UBX2',
    description: 'uBlox-specific GPS information (part 2)',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Instance',
          unit: 'instance',
          description: 'GPS instance number',
        },
        {
          name: 'ofsI',
          unit: 'imbalance of I part of complex signal',
          description: '',
        },
        {
          name: 'magI',
          unit: 'magnitude of I part of complex signal',
          description: '',
        },
        {
          name: 'ofsQ',
          unit: 'imbalance of Q part of complex signal',
          description: '',
        },
        {
          name: 'magQ',
          unit: 'magnitude of Q part of complex signal',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'ubxt',
    title: 'UBXT',
    description: 'uBlox specific UBX-TIM-TM2 logging, see uBlox interface description',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'I',
          unit: 'instance',
          description: 'GPS instance number',
        },
        {
          name: 'ch',
          unit: 'Channel (i.e. EXTINT) upon which the pulse was measured',
          description: '',
        },
        {
          name: 'flags',
          unit: 'Bitmask',
          description: '',
        },
        {
          name: 'count',
          unit: 'Rising edge counter',
          description: '',
        },
        {
          name: 'wnR',
          unit: 'Week number of last rising edge',
          description: '',
        },
        {
          name: 'MsR',
          unit: 'ms',
          description: 'Tow of rising edge',
        },
        {
          name: 'SubMsR',
          unit: 'ns',
          description: 'Millisecond fraction of tow of rising edge in nanoseconds',
        },
        {
          name: 'wnF',
          unit: 'Week number of last falling edge',
          description: '',
        },
        {
          name: 'MsF',
          unit: 'ms',
          description: 'Tow of falling edge',
        },
        {
          name: 'SubMsF',
          unit: 'ns',
          description: 'Millisecond fraction of tow of falling edge in nanoseconds',
        },
        {
          name: 'accEst',
          unit: 'ns',
          description: 'Accuracy estimate',
        },
      ],
    ],
  },
  {
    id: 'unit',
    title: 'UNIT',
    description: 'Message mapping from single character to SI unit',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Id',
          unit: 'character referenced by FMTU',
          description: '',
        },
        {
          name: 'Label',
          unit: 'char[64]',
          description: 'Unit - SI where available',
        },
      ],
    ],
  },
  {
    id: 'var',
    title: 'VAR',
    description: 'Variometer data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'Time since system startup',
          description: '',
        },
        {
          name: 'aspd_raw',
          unit: 'always zero',
          description: '',
        },
        {
          name: 'aspd_filt',
          unit: 'filtered and constrained airspeed',
          description: '',
        },
        {
          name: 'alt',
          unit: 'AHRS altitude',
          description: '',
        },
        {
          name: 'roll',
          unit: 'AHRS roll',
          description: '',
        },
        {
          name: 'raw',
          unit: 'estimated air vertical speed',
          description: '',
        },
        {
          name: 'filt',
          unit: 'low-pass filtered air vertical speed',
          description: '',
        },
        {
          name: 'cl',
          unit: 'raw climb rate',
          description: '',
        },
        {
          name: 'fc',
          unit: 'filtered climb rate',
          description: '',
        },
        {
          name: 'exs',
          unit: 'expected sink rate relative to air in thermalling turn',
          description: '',
        },
        {
          name: 'dsp',
          unit: 'average acceleration along X axis',
          description: '',
        },
        {
          name: 'dspb',
          unit: 'detected bias in average acceleration along X axis',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'ver',
    title: 'VER',
    description: 'Ardupilot version',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'BT',
          unit: 'enum',
          description: 'Board type Values:',
        },
        {
          name: 'HAL_BOARD_SITL',
          unit: '3',
          description: '',
        },
        {
          name: 'HAL_BOARD_LINUX',
          unit: '7',
          description: '',
        },
        {
          name: 'HAL_BOARD_CHIBIOS',
          unit: '10',
          description: '',
        },
        {
          name: 'HAL_BOARD_ESP32',
          unit: '12',
          description: '',
        },
        {
          name: 'HAL_BOARD_QURT',
          unit: '13',
          description: '',
        },
        {
          name: 'HAL_BOARD_EMPTY',
          unit: '99',
          description: '',
        },
        {
          name: 'BST',
          unit: 'enum',
          description: 'Board subtype Values:',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_NONE',
          unit: '-1',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_NONE',
          unit: '1000',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD',
          unit: '1001',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_PXF',
          unit: '1002',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_NAVIO',
          unit: '1003',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_ZYNQ',
          unit: '1004',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_BBBMINI',
          unit: '1005',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_BEBOP',
          unit: '1006',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2',
          unit: '1009',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_BH',
          unit: '1010',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_PXFMINI',
          unit: '1012',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_NAVIO2',
          unit: '1013',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_DISCO',
          unit: '1014',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_AERO',
          unit: '1015',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_DARK',
          unit: '1016',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_BLUE',
          unit: '1018',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_OCPOC_ZYNQ',
          unit: '1019',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_EDGE',
          unit: '1020',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_POCKET',
          unit: '1022',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_NAVIGATOR',
          unit: '1023',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_VNAV',
          unit: '1024',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_OBAL_V1',
          unit: '1025',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_CANZERO',
          unit: '1026',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_PILOTPI',
          unit: '1027',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_POCKET2',
          unit: '1028',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_T3_GEM_O1',
          unit: '1029',
          description: '',
        },
        {
          name: 'Maj',
          unit: 'Major version number',
          description: '',
        },
        {
          name: 'Min',
          unit: 'Minor version number',
          description: '',
        },
        {
          name: 'Pat',
          unit: 'Patch number',
          description: '',
        },
        {
          name: 'FWT',
          unit: 'Firmware type',
          description: '',
        },
        {
          name: 'GH',
          unit: 'Github commit',
          description: '',
        },
        {
          name: 'FWS',
          unit: 'char[64]',
          description: 'Firmware version string',
        },
        {
          name: 'APJ',
          unit: 'Board ID',
          description: '',
        },
        {
          name: 'BU',
          unit: 'enum',
          description: 'Build vehicle type Values:',
        },
        {
          name: 'APM_BUILD_Rover',
          unit: '1',
          description: '',
        },
        {
          name: 'APM_BUILD_ArduCopter',
          unit: '2',
          description: '',
        },
        {
          name: 'APM_BUILD_ArduPlane',
          unit: '3',
          description: '',
        },
        {
          name: 'APM_BUILD_AntennaTracker',
          unit: '4',
          description: '',
        },
        {
          name: 'APM_BUILD_UNKNOWN',
          unit: '5',
          description: '',
        },
        {
          name: 'APM_BUILD_Replay',
          unit: '6',
          description: '',
        },
        {
          name: 'APM_BUILD_ArduSub',
          unit: '7',
          description: '',
        },
        {
          name: 'APM_BUILD_iofirmware',
          unit: '8',
          description: '',
        },
        {
          name: 'APM_BUILD_AP_Periph',
          unit: '9',
          description: '',
        },
        {
          name: 'APM_BUILD_AP_DAL_Standalone',
          unit: '10',
          description: '',
        },
        {
          name: 'APM_BUILD_AP_Bootloader',
          unit: '11',
          description: '',
        },
        {
          name: 'APM_BUILD_Blimp',
          unit: '12',
          description: '',
        },
        {
          name: 'APM_BUILD_Heli',
          unit: '13',
          description: '',
        },
        {
          name: 'FV',
          unit: 'Filter version',
          description: '',
        },
        {
          name: 'IMI',
          unit: 'IOMCU MCU ID',
          description: '',
        },
        {
          name: 'ICI',
          unit: 'IOMCU CPU ID',
          description: '',
        },
      ],
      [
        {
          name: 'HAL_BOARD_SITL',
          unit: '3',
          description: '',
        },
        {
          name: 'HAL_BOARD_LINUX',
          unit: '7',
          description: '',
        },
        {
          name: 'HAL_BOARD_CHIBIOS',
          unit: '10',
          description: '',
        },
        {
          name: 'HAL_BOARD_ESP32',
          unit: '12',
          description: '',
        },
        {
          name: 'HAL_BOARD_QURT',
          unit: '13',
          description: '',
        },
        {
          name: 'HAL_BOARD_EMPTY',
          unit: '99',
          description: '',
        },
      ],
      [
        {
          name: 'HAL_BOARD_SUBTYPE_NONE',
          unit: '-1',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_NONE',
          unit: '1000',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD',
          unit: '1001',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_PXF',
          unit: '1002',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_NAVIO',
          unit: '1003',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_ZYNQ',
          unit: '1004',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_BBBMINI',
          unit: '1005',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_BEBOP',
          unit: '1006',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2',
          unit: '1009',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_BH',
          unit: '1010',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_PXFMINI',
          unit: '1012',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_NAVIO2',
          unit: '1013',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_DISCO',
          unit: '1014',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_AERO',
          unit: '1015',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_DARK',
          unit: '1016',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_BLUE',
          unit: '1018',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_OCPOC_ZYNQ',
          unit: '1019',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_EDGE',
          unit: '1020',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_POCKET',
          unit: '1022',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_NAVIGATOR',
          unit: '1023',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_VNAV',
          unit: '1024',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_OBAL_V1',
          unit: '1025',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_CANZERO',
          unit: '1026',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_PILOTPI',
          unit: '1027',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_POCKET2',
          unit: '1028',
          description: '',
        },
        {
          name: 'HAL_BOARD_SUBTYPE_LINUX_T3_GEM_O1',
          unit: '1029',
          description: '',
        },
      ],
      [
        {
          name: 'APM_BUILD_Rover',
          unit: '1',
          description: '',
        },
        {
          name: 'APM_BUILD_ArduCopter',
          unit: '2',
          description: '',
        },
        {
          name: 'APM_BUILD_ArduPlane',
          unit: '3',
          description: '',
        },
        {
          name: 'APM_BUILD_AntennaTracker',
          unit: '4',
          description: '',
        },
        {
          name: 'APM_BUILD_UNKNOWN',
          unit: '5',
          description: '',
        },
        {
          name: 'APM_BUILD_Replay',
          unit: '6',
          description: '',
        },
        {
          name: 'APM_BUILD_ArduSub',
          unit: '7',
          description: '',
        },
        {
          name: 'APM_BUILD_iofirmware',
          unit: '8',
          description: '',
        },
        {
          name: 'APM_BUILD_AP_Periph',
          unit: '9',
          description: '',
        },
        {
          name: 'APM_BUILD_AP_DAL_Standalone',
          unit: '10',
          description: '',
        },
        {
          name: 'APM_BUILD_AP_Bootloader',
          unit: '11',
          description: '',
        },
        {
          name: 'APM_BUILD_Blimp',
          unit: '12',
          description: '',
        },
        {
          name: 'APM_BUILD_Heli',
          unit: '13',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'vibe',
    title: 'VIBE',
    description: 'Processed (acceleration) vibration information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'IMU',
          unit: 'instance',
          description: 'Vibration instance number',
        },
        {
          name: 'VibeX',
          unit: 'm/s/s',
          description: 'Primary accelerometer filtered vibration, x-axis',
        },
        {
          name: 'VibeY',
          unit: 'm/s/s',
          description: 'Primary accelerometer filtered vibration, y-axis',
        },
        {
          name: 'VibeZ',
          unit: 'm/s/s',
          description: 'Primary accelerometer filtered vibration, z-axis',
        },
        {
          name: 'Clip',
          unit: 'Number of clipping events on 1st accelerometer',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'viso',
    title: 'VISO',
    description: 'Visual Odometry',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'System time',
        },
        {
          name: 'dt',
          unit: 'μs',
          description: 'Time period this data covers',
        },
        {
          name: 'AngDX',
          unit: 'rad',
          description: 'Angular change for body-frame roll axis',
        },
        {
          name: 'AngDY',
          unit: 'rad',
          description: 'Angular change for body-frame pitch axis',
        },
        {
          name: 'AngDZ',
          unit: 'rad',
          description: 'Angular change for body-frame z axis',
        },
        {
          name: 'PosDX',
          unit: 'm',
          description: 'Position change for body-frame X axis (Forward-Back)',
        },
        {
          name: 'PosDY',
          unit: 'm',
          description: 'Position change for body-frame Y axis (Right-Left)',
        },
        {
          name: 'PosDZ',
          unit: 'm',
          description: 'Position change for body-frame Z axis (Down-Up)',
        },
        {
          name: 'conf',
          unit: 'Confidence',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'visp',
    title: 'VISP',
    description: 'Vision Position',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'System time',
        },
        {
          name: 'RTimeUS',
          unit: 'μs',
          description: 'Remote system time',
        },
        {
          name: 'CTimeMS',
          unit: 'ms',
          description: 'Corrected system time',
        },
        {
          name: 'PX',
          unit: 'm',
          description: 'Position X-axis (North-South)',
        },
        {
          name: 'PY',
          unit: 'm',
          description: 'Position Y-axis (East-West)',
        },
        {
          name: 'PZ',
          unit: 'm',
          description: 'Position Z-axis (Down-Up)',
        },
        {
          name: 'R',
          unit: 'deg',
          description: 'Roll lean angle',
        },
        {
          name: 'P',
          unit: 'deg',
          description: 'Pitch lean angle',
        },
        {
          name: 'Y',
          unit: 'degheading',
          description: 'Yaw angle',
        },
        {
          name: 'PErr',
          unit: 'm',
          description: 'Position estimate error',
        },
        {
          name: 'AErr',
          unit: 'deg',
          description: 'Attitude estimate error',
        },
        {
          name: 'Rst',
          unit: 'Position reset counter',
          description: '',
        },
        {
          name: 'Ign',
          unit: 'Ignored',
          description: '',
        },
        {
          name: 'Q',
          unit: '%',
          description: 'Quality',
        },
      ],
    ],
  },
  {
    id: 'visv',
    title: 'VISV',
    description: 'Vision Velocity',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'System time',
        },
        {
          name: 'RTimeUS',
          unit: 'μs',
          description: 'Remote system time',
        },
        {
          name: 'CTimeMS',
          unit: 'ms',
          description: 'Corrected system time',
        },
        {
          name: 'VX',
          unit: 'm/s',
          description: 'Velocity X-axis (North-South)',
        },
        {
          name: 'VY',
          unit: 'm/s',
          description: 'Velocity Y-axis (East-West)',
        },
        {
          name: 'VZ',
          unit: 'm/s',
          description: 'Velocity Z-axis (Down-Up)',
        },
        {
          name: 'VErr',
          unit: 'm/s',
          description: 'Velocity estimate error',
        },
        {
          name: 'Rst',
          unit: 'Velocity reset counter',
          description: '',
        },
        {
          name: 'Ign',
          unit: 'Ignored',
          description: '',
        },
        {
          name: 'Q',
          unit: '%',
          description: 'Quality',
        },
      ],
    ],
  },
  {
    id: 'vnat',
    title: 'VNAT',
    description: 'VectorNav Attitude data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Q1',
          unit: 'Attitude quaternion 1',
          description: '',
        },
        {
          name: 'Q2',
          unit: 'Attitude quaternion 2',
          description: '',
        },
        {
          name: 'Q3',
          unit: 'Attitude quaternion 3',
          description: '',
        },
        {
          name: 'Q4',
          unit: 'Attitude quaternion 4',
          description: '',
        },
        {
          name: 'Yaw',
          unit: 'deg',
          description: 'Yaw',
        },
        {
          name: 'Pitch',
          unit: 'deg',
          description: 'Pitch',
        },
        {
          name: 'Roll',
          unit: 'deg',
          description: 'Roll',
        },
        {
          name: 'YU',
          unit: 'deg',
          description: 'Yaw unceratainty',
        },
        {
          name: 'PU',
          unit: 'deg',
          description: 'Pitch uncertainty',
        },
        {
          name: 'RU',
          unit: 'deg',
          description: 'Roll uncertainty',
        },
      ],
    ],
  },
  {
    id: 'vnim',
    title: 'VNIM',
    description: 'VectorNav IMU data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Temp',
          unit: 'deg',
          description: 'Temprature',
        },
        {
          name: 'Pres',
          unit: 'Pa',
          description: 'Pressure',
        },
        {
          name: 'MX',
          unit: 'Gauss',
          description: 'Magnetic feild X-axis',
        },
        {
          name: 'MY',
          unit: 'Gauss',
          description: 'Magnetic feild Y-axis',
        },
        {
          name: 'MZ',
          unit: 'Gauss',
          description: 'Magnetic feild Z-axis',
        },
        {
          name: 'AX',
          unit: 'm/s/s',
          description: 'Acceleration X-axis',
        },
        {
          name: 'AY',
          unit: 'm/s/s',
          description: 'Acceleration Y-axis',
        },
        {
          name: 'AZ',
          unit: 'm/s/s',
          description: 'Acceleration Z-axis',
        },
        {
          name: 'GX',
          unit: 'rad/s',
          description: 'Rotation rate X-axis',
        },
        {
          name: 'GY',
          unit: 'rad/s',
          description: 'Rotation rate Y-axis',
        },
        {
          name: 'GZ',
          unit: 'rad/s',
          description: 'Rotation rate Z-axis',
        },
      ],
    ],
  },
  {
    id: 'vnkf',
    title: 'VNKF',
    description: 'VectorNav INS Kalman Filter data',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'InsStatus',
          unit: 'VectorNav INS health status',
          description: '',
        },
        {
          name: 'Lat',
          unit: 'deg',
          description: 'Latitude',
        },
        {
          name: 'Lon',
          unit: 'deg',
          description: 'Longitude',
        },
        {
          name: 'Alt',
          unit: 'm',
          description: 'Altitude',
        },
        {
          name: 'VelN',
          unit: 'm/s',
          description: 'Velocity Northing',
        },
        {
          name: 'VelE',
          unit: 'm/s',
          description: 'Velocity Easting',
        },
        {
          name: 'VelD',
          unit: 'm/s',
          description: 'Velocity Downing',
        },
        {
          name: 'PosU',
          unit: 'deg',
          description: 'Filter estimated position uncertainty',
        },
        {
          name: 'VelU',
          unit: 'm/s',
          description: 'Filter estimated Velocity uncertainty',
        },
      ],
    ],
  },
  {
    id: 'vstb',
    title: 'VSTB',
    description: 'Log message for video stabilisation software such as Gyroflow',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'GyrX',
          unit: 'rad/s',
          description: 'measured rotation rate about X axis',
        },
        {
          name: 'GyrY',
          unit: 'rad/s',
          description: 'measured rotation rate about Y axis',
        },
        {
          name: 'GyrZ',
          unit: 'rad/s',
          description: 'measured rotation rate about Z axis',
        },
        {
          name: 'AccX',
          unit: 'm/s/s',
          description: 'acceleration along X axis',
        },
        {
          name: 'AccY',
          unit: 'm/s/s',
          description: 'acceleration along Y axis',
        },
        {
          name: 'AccZ',
          unit: 'm/s/s',
          description: 'acceleration along Z axis',
        },
        {
          name: 'Q1',
          unit: 'Estimated attitude quaternion component 1',
          description: '',
        },
        {
          name: 'Q2',
          unit: 'Estimated attitude quaternion component 2',
          description: '',
        },
        {
          name: 'Q3',
          unit: 'Estimated attitude quaternion component 3',
          description: '',
        },
        {
          name: 'Q4',
          unit: 'Estimated attitude quaternion component 4',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'wdog',
    title: 'WDOG',
    description: 'Watchdog diagnostics',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Tsk',
          unit: 'current task number',
          description: '',
        },
        {
          name: 'IE',
          unit: 'internal error mast',
          description: '',
        },
        {
          name: 'IEC',
          unit: 'internal error count',
          description: '',
        },
        {
          name: 'IEL',
          unit: 'line internal error was raised on',
          description: '',
        },
        {
          name: 'MvMsg',
          unit: 'mavlink message being acted on',
          description: '',
        },
        {
          name: 'MvCmd',
          unit: 'mavlink command being acted on',
          description: '',
        },
        {
          name: 'SmLn',
          unit: 'line semaphore was taken on',
          description: '',
        },
        {
          name: 'FL',
          unit: 'fault_line',
          description: '',
        },
        {
          name: 'FT',
          unit: 'fault_type',
          description: '',
        },
        {
          name: 'FA',
          unit: 'fault address',
          description: '',
        },
        {
          name: 'FP',
          unit: 'fault thread priority',
          description: '',
        },
        {
          name: 'ICSR',
          unit: 'ICS regiuster',
          description: '',
        },
        {
          name: 'LR',
          unit: 'long return address',
          description: '',
        },
        {
          name: 'TN',
          unit: 'char[4]',
          description: 'Thread name',
        },
      ],
    ],
  },
  {
    id: 'wenc',
    title: 'WENC',
    description: 'Wheel encoder measurements',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Dist0',
          unit: 'm',
          description: 'First wheel distance travelled',
        },
        {
          name: 'Qual0',
          unit: 'Quality measurement of Dist0',
          description: '',
        },
        {
          name: 'Dist1',
          unit: 'm',
          description: 'Second wheel distance travelled',
        },
        {
          name: 'Qual1',
          unit: 'Quality measurement of Dist1',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'winc',
    title: 'WINC',
    description: 'Winch',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'Heal',
          unit: 'Healthy',
          description: '',
        },
        {
          name: 'ThEnd',
          unit: 'Reached end of thread',
          description: '',
        },
        {
          name: 'Mov',
          unit: 'Motor is moving',
          description: '',
        },
        {
          name: 'Clut',
          unit: 'Clutch is engaged (motor can move freely)',
          description: '',
        },
        {
          name: 'Mode',
          unit: '0 is Relaxed, 1 is Position Control, 2 is Rate Control',
          description: '',
        },
        {
          name: 'DLen',
          unit: 'm',
          description: 'Desired Length',
        },
        {
          name: 'Len',
          unit: 'm',
          description: 'Estimated Length',
        },
        {
          name: 'DRate',
          unit: 'm/s',
          description: 'Desired Rate',
        },
        {
          name: 'Tens',
          unit: 'UNKNOWN',
          description: 'Tension on line',
        },
        {
          name: 'Vcc',
          unit: 'V',
          description: 'Voltage to Motor',
        },
        {
          name: 'Temp',
          unit: 'degC',
          description: 'Motor temperature',
        },
      ],
    ],
  },
  {
    id: 'wind',
    title: 'WIND',
    description: 'Windvane readings',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'DrRaw',
          unit: 'deg',
          description: 'raw apparent wind direction direct from sensor, in body-frame',
        },
        {
          name: 'DrApp',
          unit: 'deg',
          description: 'Apparent wind direction, in body-frame',
        },
        {
          name: 'DrTru',
          unit: 'degheading',
          description: 'True wind direction',
        },
        {
          name: 'SpdRaw',
          unit: 'm/s',
          description: 'raw wind speed direct from sensor',
        },
        {
          name: 'SpdApp',
          unit: 'm/s',
          description: 'Apparent wind Speed',
        },
        {
          name: 'SpdTru',
          unit: 'm/s',
          description: 'True wind speed',
        },
      ],
    ],
  },
  {
    id: 'xkf0',
    title: 'XKF0',
    description: 'EKF3 beacon sensor diagnostics',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF3 core this data is for',
        },
        {
          name: 'ID',
          unit: 'Beacon sensor ID',
          description: '',
        },
        {
          name: 'rng',
          unit: 'm',
          description: 'Beacon range',
        },
        {
          name: 'innov',
          unit: 'Beacon range innovation',
          description: '',
        },
        {
          name: 'SIV',
          unit: 'sqrt of beacon range innovation variance',
          description: '',
        },
        {
          name: 'TR',
          unit: 'Beacon range innovation consistency test ratio',
          description: '',
        },
        {
          name: 'BPN',
          unit: 'm',
          description: 'Beacon north position',
        },
        {
          name: 'BPE',
          unit: 'm',
          description: 'Beacon east position',
        },
        {
          name: 'BPD',
          unit: 'm',
          description: 'Beacon down position',
        },
        {
          name: 'OFH',
          unit: 'm',
          description: 'High estimate of vertical position offset of beacons rel to EKF origin',
        },
        {
          name: 'OFL',
          unit: 'm',
          description: 'Low estimate of vertical position offset of beacons rel to EKF origin',
        },
        {
          name: 'OFN',
          unit: 'm',
          description: 'North position of receiver rel to EKF origin',
        },
        {
          name: 'OFE',
          unit: 'm',
          description: 'East position of receiver rel to EKF origin',
        },
        {
          name: 'OFD',
          unit: 'm',
          description: 'Down position of receiver rel to EKF origin',
        },
      ],
    ],
  },
  {
    id: 'xkf1',
    title: 'XKF1',
    description: 'EKF3 estimator outputs',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF3 core this data is for',
        },
        {
          name: 'Roll',
          unit: 'deg',
          description: 'Estimated roll',
        },
        {
          name: 'Pitch',
          unit: 'deg',
          description: 'Estimated pitch',
        },
        {
          name: 'Yaw',
          unit: 'degheading',
          description: 'Estimated yaw',
        },
        {
          name: 'VN',
          unit: 'm/s',
          description: 'Estimated velocity (North component)',
        },
        {
          name: 'VE',
          unit: 'm/s',
          description: 'Estimated velocity (East component)',
        },
        {
          name: 'VD',
          unit: 'm/s',
          description: 'Estimated velocity (Down component)',
        },
        {
          name: 'dPD',
          unit: 'm/s',
          description: 'Filtered derivative of vertical position (down)',
        },
        {
          name: 'PN',
          unit: 'm',
          description: 'Estimated distance from origin (North component)',
        },
        {
          name: 'PE',
          unit: 'm',
          description: 'Estimated distance from origin (East component)',
        },
        {
          name: 'PD',
          unit: 'm',
          description: 'Estimated distance from origin (Down component)',
        },
        {
          name: 'GX',
          unit: 'deg/s',
          description: 'Estimated gyro bias, X axis',
        },
        {
          name: 'GY',
          unit: 'deg/s',
          description: 'Estimated gyro bias, Y axis',
        },
        {
          name: 'GZ',
          unit: 'deg/s',
          description: 'Estimated gyro bias, Z axis',
        },
        {
          name: 'OH',
          unit: 'm',
          description: 'Height of origin above WGS-84',
        },
      ],
    ],
  },
  {
    id: 'xkf2',
    title: 'XKF2',
    description: 'EKF3 estimator secondary outputs',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF3 core this data is for',
        },
        {
          name: 'AX',
          unit: 'Estimated accelerometer X bias',
          description: '',
        },
        {
          name: 'AY',
          unit: 'Estimated accelerometer Y bias',
          description: '',
        },
        {
          name: 'AZ',
          unit: 'Estimated accelerometer Z bias',
          description: '',
        },
        {
          name: 'VWN',
          unit: 'm/s',
          description: 'Estimated wind velocity (moving-to-North component)',
        },
        {
          name: 'VWE',
          unit: 'm/s',
          description: 'Estimated wind velocity (moving-to-East component)',
        },
        {
          name: 'MN',
          unit: 'mGauss',
          description: 'Magnetic field strength (North component)',
        },
        {
          name: 'ME',
          unit: 'mGauss',
          description: 'Magnetic field strength (East component)',
        },
        {
          name: 'MD',
          unit: 'mGauss',
          description: 'Magnetic field strength (Down component)',
        },
        {
          name: 'MX',
          unit: 'mGauss',
          description: 'Magnetic field strength (body X-axis)',
        },
        {
          name: 'MY',
          unit: 'mGauss',
          description: 'Magnetic field strength (body Y-axis)',
        },
        {
          name: 'MZ',
          unit: 'mGauss',
          description: 'Magnetic field strength (body Z-axis)',
        },
        {
          name: 'IDX',
          unit: 'm/s/s',
          description: 'Innovation in vehicle drag acceleration (X-axis component)',
        },
        {
          name: 'IDY',
          unit: 'm/s/s',
          description: 'Innovation in vehicle drag acceleration (Y-axis component)',
        },
        {
          name: 'IS',
          unit: 'rad',
          description: 'Innovation in vehicle sideslip',
        },
      ],
    ],
  },
  {
    id: 'xkf3',
    title: 'XKF3',
    description: 'EKF3 innovations',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF3 core this data is for',
        },
        {
          name: 'IVN',
          unit: 'm/s',
          description: 'Innovation in velocity (North component)',
        },
        {
          name: 'IVE',
          unit: 'm/s',
          description: 'Innovation in velocity (East component)',
        },
        {
          name: 'IVD',
          unit: 'm/s',
          description: 'Innovation in velocity (Down component)',
        },
        {
          name: 'IPN',
          unit: 'm',
          description: 'Innovation in position (North component)',
        },
        {
          name: 'IPE',
          unit: 'm',
          description: 'Innovation in position (East component)',
        },
        {
          name: 'IPD',
          unit: 'm',
          description: 'Innovation in position (Down component)',
        },
        {
          name: 'IMX',
          unit: 'mGauss',
          description: 'Innovation in magnetic field strength (X-axis component)',
        },
        {
          name: 'IMY',
          unit: 'mGauss',
          description: 'Innovation in magnetic field strength (Y-axis component)',
        },
        {
          name: 'IMZ',
          unit: 'mGauss',
          description: 'Innovation in magnetic field strength (Z-axis component)',
        },
        {
          name: 'IYAW',
          unit: 'deg',
          description: 'Innovation in vehicle yaw',
        },
        {
          name: 'IVT',
          unit: 'UNKNOWN',
          description: 'Innovation in true-airspeed',
        },
        {
          name: 'RErr',
          unit: 'Accumulated relative error of this core with respect to active primary core',
          description: '',
        },
        {
          name: 'ErSc',
          unit: 'A consolidated error score where higher numbers are less healthy',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'xkf4',
    title: 'XKF4',
    description:
      'EKF3 variances. SV, SP, SH and SM are probably best described as ‘Squared Innovation Test Ratios’ where values <1 tells us the measurement was accepted and >1 tells us it was rejected. They represent the square of the (innovation / maximum allowed innovation) where the innovation is the difference between predicted and measured value and the maximum allowed innovation is determined from the uncertainty of the measurement, uncertainty of the prediction and scaled using the number of standard deviations set by the innovation gate parameter for that measurement, eg EK3_MAG_I_GATE, EK3_HGT_I_GATE, etc',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF3 core this data is for',
        },
        {
          name: 'SV',
          unit: 'Square root of the velocity variance',
          description: '',
        },
        {
          name: 'SP',
          unit: 'Square root of the position variance',
          description: '',
        },
        {
          name: 'SH',
          unit: 'Square root of the height variance',
          description: '',
        },
        {
          name: 'SM',
          unit: 'Magnetic field variance',
          description: '',
        },
        {
          name: 'SVT',
          unit: 'Square root of the total airspeed variance',
          description: '',
        },
        {
          name: 'errRP',
          unit: 'Filtered error in roll/pitch estimate',
          description: '',
        },
        {
          name: 'OFN',
          unit: 'm',
          description: 'Most recent position reset (North component)',
        },
        {
          name: 'OFE',
          unit: 'm',
          description: 'Most recent position reset (East component)',
        },
        {
          name: 'FS',
          unit: 'Filter fault status',
          description: '',
        },
        {
          name: 'TS',
          unit:
            'Filter timeout status bitmask (0:position measurement, 1:velocity measurement, 2:height measurement, 3:magnetometer measurement, 4:airspeed measurement, 5:drag measurement)',
          description: '',
        },
        {
          name: 'SS',
          unit: 'bitmask',
          description: 'Filter solution status Bitmask values:',
        },
        {
          name: 'ATTITUDE_VALID',
          unit: '1',
          description: 'attitude estimate valid',
        },
        {
          name: 'HORIZ_VEL',
          unit: '2',
          description: 'horizontal velocity estimate valid',
        },
        {
          name: 'VERT_VEL',
          unit: '4',
          description: 'vertical velocity estimate valid',
        },
        {
          name: 'HORIZ_POS_REL',
          unit: '8',
          description: 'relative horizontal position estimate valid',
        },
        {
          name: 'HORIZ_POS_ABS',
          unit: '16',
          description: 'absolute horizontal position estimate valid',
        },
        {
          name: 'VERT_POS',
          unit: '32',
          description: 'vertical position estimate valid',
        },
        {
          name: 'TERRAIN_ALT',
          unit: '64',
          description: 'terrain height estimate valid',
        },
        {
          name: 'CONST_POS_MODE',
          unit: '128',
          description: 'in constant position mode',
        },
        {
          name: 'PRED_HORIZ_POS_REL',
          unit: '256',
          description: 'expected good relative horizontal position estimate - used before takeoff',
        },
        {
          name: 'PRED_HORIZ_POS_ABS',
          unit: '512',
          description: 'expected good absolute horizontal position estimate - used before takeoff',
        },
        {
          name: 'TAKEOFF_DETECTED',
          unit: '1024',
          description: 'optical flow takeoff has been detected',
        },
        {
          name: 'TAKEOFF_EXPECTED',
          unit: '2048',
          description: 'compensating for baro errors during takeoff',
        },
        {
          name: 'TOUCHDOWN_EXPECTED',
          unit: '4096',
          description: 'compensating for baro errors during touchdown',
        },
        {
          name: 'USING_GPS',
          unit: '8192',
          description: 'using GPS position',
        },
        {
          name: 'GPS_GLITCHING',
          unit: '16384',
          description: 'GPS glitching is affecting navigation accuracy',
        },
        {
          name: 'GPS_QUALITY_GOOD',
          unit: '32768',
          description: 'can use GPS for navigation',
        },
        {
          name: 'INITALIZED',
          unit: '65536',
          description: 'has ever been healthy',
        },
        {
          name: 'REJECTING_AIRSPEED',
          unit: '131072',
          description: 'rejecting airspeed data',
        },
        {
          name: 'DEAD_RECKONING',
          unit: '262144',
          description: 'dead reckoning (e.g. no position or velocity source)',
        },
        {
          name: 'GPS',
          unit: 'Filter GPS status',
          description: '',
        },
        {
          name: 'PI',
          unit: 'Primary core index',
          description: '',
        },
      ],
      [
        {
          name: 'ATTITUDE_VALID',
          unit: '1',
          description: 'attitude estimate valid',
        },
        {
          name: 'HORIZ_VEL',
          unit: '2',
          description: 'horizontal velocity estimate valid',
        },
        {
          name: 'VERT_VEL',
          unit: '4',
          description: 'vertical velocity estimate valid',
        },
        {
          name: 'HORIZ_POS_REL',
          unit: '8',
          description: 'relative horizontal position estimate valid',
        },
        {
          name: 'HORIZ_POS_ABS',
          unit: '16',
          description: 'absolute horizontal position estimate valid',
        },
        {
          name: 'VERT_POS',
          unit: '32',
          description: 'vertical position estimate valid',
        },
        {
          name: 'TERRAIN_ALT',
          unit: '64',
          description: 'terrain height estimate valid',
        },
        {
          name: 'CONST_POS_MODE',
          unit: '128',
          description: 'in constant position mode',
        },
        {
          name: 'PRED_HORIZ_POS_REL',
          unit: '256',
          description: 'expected good relative horizontal position estimate - used before takeoff',
        },
        {
          name: 'PRED_HORIZ_POS_ABS',
          unit: '512',
          description: 'expected good absolute horizontal position estimate - used before takeoff',
        },
        {
          name: 'TAKEOFF_DETECTED',
          unit: '1024',
          description: 'optical flow takeoff has been detected',
        },
        {
          name: 'TAKEOFF_EXPECTED',
          unit: '2048',
          description: 'compensating for baro errors during takeoff',
        },
        {
          name: 'TOUCHDOWN_EXPECTED',
          unit: '4096',
          description: 'compensating for baro errors during touchdown',
        },
        {
          name: 'USING_GPS',
          unit: '8192',
          description: 'using GPS position',
        },
        {
          name: 'GPS_GLITCHING',
          unit: '16384',
          description: 'GPS glitching is affecting navigation accuracy',
        },
        {
          name: 'GPS_QUALITY_GOOD',
          unit: '32768',
          description: 'can use GPS for navigation',
        },
        {
          name: 'INITALIZED',
          unit: '65536',
          description: 'has ever been healthy',
        },
        {
          name: 'REJECTING_AIRSPEED',
          unit: '131072',
          description: 'rejecting airspeed data',
        },
        {
          name: 'DEAD_RECKONING',
          unit: '262144',
          description: 'dead reckoning (e.g. no position or velocity source)',
        },
      ],
    ],
  },
  {
    id: 'xkf5',
    title: 'XKF5',
    description: 'EKF3 Sensor innovations (primary core) and general dumping ground',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF3 core this data is for',
        },
        {
          name: 'NI',
          unit: 'Normalised flow variance',
          description: '',
        },
        {
          name: 'FIX',
          unit: 'Optical flow LOS rate vector innovations from the main nav filter (X-axis)',
          description: '',
        },
        {
          name: 'FIY',
          unit: 'Optical flow LOS rate vector innovations from the main nav filter (Y-axis)',
          description: '',
        },
        {
          name: 'AFI',
          unit: 'Optical flow LOS rate innovation from terrain offset estimator',
          description: '',
        },
        {
          name: 'HAGL',
          unit: 'm',
          description: 'Height above ground level',
        },
        {
          name: 'offset',
          unit: 'UNKNOWN',
          description:
            'Estimated vertical position of the terrain relative to the nav filter zero datum',
        },
        {
          name: 'RI',
          unit: 'UNKNOWN',
          description: 'Range finder innovations',
        },
        {
          name: 'rng',
          unit: 'UNKNOWN',
          description: 'Measured range',
        },
        {
          name: 'Herr',
          unit: 'm',
          description: 'Filter ground offset state error',
        },
        {
          name: 'eAng',
          unit: 'rad',
          description: 'Magnitude of angular error',
        },
        {
          name: 'eVel',
          unit: 'm/s',
          description: 'Magnitude of velocity error',
        },
        {
          name: 'ePos',
          unit: 'm',
          description: 'Magnitude of position error',
        },
      ],
    ],
  },
  {
    id: 'xkfd',
    title: 'XKFD',
    description: 'EKF3 Body Frame Odometry errors',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF3 core this data is for',
        },
        {
          name: 'IX',
          unit: 'Innovation in velocity (X-axis)',
          description: '',
        },
        {
          name: 'IY',
          unit: 'Innovation in velocity (Y-axis)',
          description: '',
        },
        {
          name: 'IZ',
          unit: 'Innovation in velocity (Z-axis)',
          description: '',
        },
        {
          name: 'IVX',
          unit: 'Variance in velocity (X-axis)',
          description: '',
        },
        {
          name: 'IVY',
          unit: 'Variance in velocity (Y-axis)',
          description: '',
        },
        {
          name: 'IVZ',
          unit: 'Variance in velocity (Z-axis)',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'xkfm',
    title: 'XKFM',
    description: 'EKF3 diagnostic data for on-ground-and-not-moving check',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF core this message instance applies to',
        },
        {
          name: 'OGNM',
          unit: 'True of on ground and not moving',
          description: '',
        },
        {
          name: 'GLR',
          unit: 'Gyroscope length ratio',
          description: '',
        },
        {
          name: 'ALR',
          unit: 'Accelerometer length ratio',
          description: '',
        },
        {
          name: 'GDR',
          unit: 'Gyroscope rate of change ratio',
          description: '',
        },
        {
          name: 'ADR',
          unit: 'Accelerometer rate of change ratio',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'xkfs',
    title: 'XKFS',
    description: 'EKF3 sensor selection',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF3 core this data is for',
        },
        {
          name: 'MI',
          unit: 'compass selection index',
          description: '',
        },
        {
          name: 'BI',
          unit: 'barometer selection index',
          description: '',
        },
        {
          name: 'GI',
          unit: 'GPS selection index',
          description: '',
        },
        {
          name: 'AI',
          unit: 'airspeed selection index',
          description: '',
        },
        {
          name: 'SS',
          unit: 'Source Set (primary=0/secondary=1/tertiary=2)',
          description: '',
        },
        {
          name: 'GPS_GTA',
          unit: 'GPS good to align',
          description: '',
        },
        {
          name: 'GPS_CHK_WAIT',
          unit: 'Waiting for GPS checks to pass',
          description: '',
        },
        {
          name: 'MAG_FUSION',
          unit: 'Magnetometer fusion (0=not fusing/1=fuse yaw/2=fuse mag)',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'xkq',
    title: 'XKQ',
    description: 'EKF3 quaternion defining the rotation from NED to XYZ (autopilot) axes',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF3 core this data is for',
        },
        {
          name: 'Q1',
          unit: 'UNKNOWN',
          description: 'Quaternion a term',
        },
        {
          name: 'Q2',
          unit: 'UNKNOWN',
          description: 'Quaternion b term',
        },
        {
          name: 'Q3',
          unit: 'UNKNOWN',
          description: 'Quaternion c term',
        },
        {
          name: 'Q4',
          unit: 'UNKNOWN',
          description: 'Quaternion d term',
        },
      ],
    ],
  },
  {
    id: 'xkt',
    title: 'XKT',
    description: 'EKF3 timing information',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF core this message instance applies to',
        },
        {
          name: 'Cnt',
          unit: 's',
          description: 'count of samples used to create this message',
        },
        {
          name: 'IMUMin',
          unit: 's',
          description: 'smallest IMU sample interval',
        },
        {
          name: 'IMUMax',
          unit: 's',
          description: 'largest IMU sample interval',
        },
        {
          name: 'EKFMin',
          unit: 's',
          description: 'low-passed achieved average time step rate for the EKF (minimum)',
        },
        {
          name: 'EKFMax',
          unit: 's',
          description: 'low-passed achieved average time step rate for the EKF (maximum)',
        },
        {
          name: 'AngMin',
          unit: 's',
          description: 'accumulated measurement time interval for the delta angle (minimum)',
        },
        {
          name: 'AngMax',
          unit: 's',
          description: 'accumulated measurement time interval for the delta angle (maximum)',
        },
        {
          name: 'VMin',
          unit: 's',
          description: 'accumulated measurement time interval for the delta velocity (minimum)',
        },
        {
          name: 'VMax',
          unit: 's',
          description: 'accumulated measurement time interval for the delta velocity (maximum)',
        },
      ],
    ],
  },
  {
    id: 'xktv',
    title: 'XKTV',
    description: 'EKF3 Yaw Estimator States',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF3 core this data is for',
        },
        {
          name: 'TVS',
          unit: 'rad',
          description: 'Tilt Error Variance from symbolic equations (rad^2)',
        },
        {
          name: 'TVD',
          unit: 'rad',
          description: 'Tilt Error Variance from difference method (rad^2)',
        },
      ],
    ],
  },
  {
    id: 'xkv1',
    title: 'XKV1',
    description: 'EKF3 State variances (primary core)',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF3 core this data is for',
        },
        {
          name: 'V00',
          unit: 'Variance for state 0 (attitude quaternion)',
          description: '',
        },
        {
          name: 'V01',
          unit: 'Variance for state 1 (attitude quaternion)',
          description: '',
        },
        {
          name: 'V02',
          unit: 'Variance for state 2 (attitude quaternion)',
          description: '',
        },
        {
          name: 'V03',
          unit: 'Variance for state 3 (attitude quaternion)',
          description: '',
        },
        {
          name: 'V04',
          unit: 'Variance for state 4 (velocity-north)',
          description: '',
        },
        {
          name: 'V05',
          unit: 'Variance for state 5 (velocity-east)',
          description: '',
        },
        {
          name: 'V06',
          unit: 'Variance for state 6 (velocity-down)',
          description: '',
        },
        {
          name: 'V07',
          unit: 'Variance for state 7 (position-north)',
          description: '',
        },
        {
          name: 'V08',
          unit: 'Variance for state 8 (position-east)',
          description: '',
        },
        {
          name: 'V09',
          unit: 'Variance for state 9 (position-down)',
          description: '',
        },
        {
          name: 'V10',
          unit: 'Variance for state 10 (delta-angle-bias-x)',
          description: '',
        },
        {
          name: 'V11',
          unit: 'Variance for state 11 (delta-angle-bias-y)',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'xkv2',
    title: 'XKV2',
    description: 'more EKF3 State Variances (primary core)',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF3 core this data is for',
        },
        {
          name: 'V12',
          unit: 'Variance for state 12 (delta-angle-bias-z)',
          description: '',
        },
        {
          name: 'V13',
          unit: 'Variance for state 13 (delta-velocity-bias-x)',
          description: '',
        },
        {
          name: 'V14',
          unit: 'Variance for state 14 (delta-velocity-bias-y)',
          description: '',
        },
        {
          name: 'V15',
          unit: 'Variance for state 15 (delta-velocity-bias-z)',
          description: '',
        },
        {
          name: 'V16',
          unit: 'Variance for state 16 (Earth-frame mag-field-bias-x)',
          description: '',
        },
        {
          name: 'V17',
          unit: 'Variance for state 17 (Earth-frame mag-field-bias-y)',
          description: '',
        },
        {
          name: 'V18',
          unit: 'Variance for state 18 (Earth-frame mag-field-bias-z)',
          description: '',
        },
        {
          name: 'V19',
          unit: 'Variance for state 19 (body-frame mag-field-bias-x)',
          description: '',
        },
        {
          name: 'V20',
          unit: 'Variance for state 20 (body-frame mag-field-bias-y)',
          description: '',
        },
        {
          name: 'V21',
          unit: 'Variance for state 21 (body-frame mag-field-bias-z)',
          description: '',
        },
        {
          name: 'V22',
          unit: 'Variance for state 22 (wind-north)',
          description: '',
        },
        {
          name: 'V23',
          unit: 'Variance for state 23 (wind-east)',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'xky0',
    title: 'XKY0',
    description: 'EKF Yaw Estimator States',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF core this data is for',
        },
        {
          name: 'YC',
          unit: 'degheading',
          description: 'GSF yaw estimate (deg)',
        },
        {
          name: 'YCS',
          unit: 'deg',
          description: 'GSF yaw estimate 1-Sigma uncertainty (deg)',
        },
        {
          name: 'Y0',
          unit: 'degheading',
          description: 'Yaw estimate from individual EKF filter 0 (deg)',
        },
        {
          name: 'Y1',
          unit: 'degheading',
          description: 'Yaw estimate from individual EKF filter 1 (deg)',
        },
        {
          name: 'Y2',
          unit: 'degheading',
          description: 'Yaw estimate from individual EKF filter 2 (deg)',
        },
        {
          name: 'Y3',
          unit: 'degheading',
          description: 'Yaw estimate from individual EKF filter 3 (deg)',
        },
        {
          name: 'Y4',
          unit: 'degheading',
          description: 'Yaw estimate from individual EKF filter 4 (deg)',
        },
        {
          name: 'W0',
          unit: 'Weighting applied to yaw estimate from individual EKF filter 0',
          description: '',
        },
        {
          name: 'W1',
          unit: 'Weighting applied to yaw estimate from individual EKF filter 1',
          description: '',
        },
        {
          name: 'W2',
          unit: 'Weighting applied to yaw estimate from individual EKF filter 2',
          description: '',
        },
        {
          name: 'W3',
          unit: 'Weighting applied to yaw estimate from individual EKF filter 3',
          description: '',
        },
        {
          name: 'W4',
          unit: 'Weighting applied to yaw estimate from individual EKF filter 4',
          description: '',
        },
      ],
    ],
  },
  {
    id: 'xky1',
    title: 'XKY1',
    description: 'EKF Yaw Estimator Innovations',
    tables: [
      [
        {
          name: 'TimeUS',
          unit: 'μs',
          description: 'Time since system startup',
        },
        {
          name: 'C',
          unit: 'instance',
          description: 'EKF core this data is for',
        },
        {
          name: 'IVN0',
          unit: 'm/s',
          description: 'North velocity innovation from individual EKF filter 0 (m/s)',
        },
        {
          name: 'IVN1',
          unit: 'm/s',
          description: 'North velocity innovation from individual EKF filter 1 (m/s)',
        },
        {
          name: 'IVN2',
          unit: 'm/s',
          description: 'North velocity innovation from individual EKF filter 2 (m/s)',
        },
        {
          name: 'IVN3',
          unit: 'm/s',
          description: 'North velocity innovation from individual EKF filter 3 (m/s)',
        },
        {
          name: 'IVN4',
          unit: 'm/s',
          description: 'North velocity innovation from individual EKF filter 4 (m/s)',
        },
        {
          name: 'IVE0',
          unit: 'm/s',
          description: 'East velocity innovation from individual EKF filter 0 (m/s)',
        },
        {
          name: 'IVE1',
          unit: 'm/s',
          description: 'East velocity innovation from individual EKF filter 1 (m/s)',
        },
        {
          name: 'IVE2',
          unit: 'm/s',
          description: 'East velocity innovation from individual EKF filter 2 (m/s)',
        },
        {
          name: 'IVE3',
          unit: 'm/s',
          description: 'East velocity innovation from individual EKF filter 3 (m/s)',
        },
        {
          name: 'IVE4',
          unit: 'm/s',
          description: 'East velocity innovation from individual EKF filter 4 (m/s)',
        },
      ],
    ],
  },
];

export const getAllLogDocumentation = () => `
# 📄 UAV Log Message Documentation

This document provides an overview of the key UAV log message types analyzed by the UAV Log Viewer tool. Each section describes the fields, purpose, and common use cases for the respective log messages.

---
${messageDataDocs_tlog
  .map(
    doc => `## ${doc.title}
${doc.description}
### Fields
${doc.tables
  .map(
    table => `| Field Name | Description |
|------------|-------------|
${table.map(field => `| ${field.name} | ${field.description} |`).join('\n')}
`,
  )
  .join('\n')}
`,
  )
  .join('\n---\n')}

${messageDataDocs_bin
  .map(
    doc => `## ${doc.title}
${doc.description}
### Fields
${doc.tables
  .map(
    table => `| Field Name | Description |
|------------|-------------|
${table.map(field => `| ${field.name} | ${field.description} |`).join('\n')}
`,
  )
  .join('\n')}
`,
  )
  .join('\n---\n')}
`;
