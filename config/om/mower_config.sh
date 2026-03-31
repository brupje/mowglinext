################################
## Yardforce Classic 500
## GUI configuration for mowgli-docker v3
################################

################################
##   Hardware / Robot Model   ##
################################

# Mower model: YardForce500, YardForceSA650, CUSTOM
export OM_MOWER="YardForce500"

# ESC type: xesc_mini (STM32 VESC) or xesc_2040 (RP2040)
export OM_MOWER_ESC_TYPE="xesc_mini"

# Gamepad: ps3, xbox360, shield, steam_stick, steam_touch, switch_pro
export OM_MOWER_GAMEPAD="xbox360"

# Wheel geometry
export OM_WHEEL_DISTANCE_M=0.40       # wheel track centre-to-centre (m)
export OM_WHEEL_TICKS_PER_M=1000.0    # encoder ticks per metre

# GPS antenna offset from base_link centre (metres)
# On YF500 the antenna sits ~0.3m forward on the top cover
export OM_ANTENNA_OFFSET_X=0.0
export OM_ANTENNA_OFFSET_Y=0.0

################################
##        GPS Settings        ##
################################

# False = use datum point as map origin (recommended)
# True  = use ublox NAVRELPOSNED (base station = origin)
export OM_USE_RELATIVE_POSITION=False

# Map origin — set to coordinates near your docking station
export OM_DATUM_LAT=48.8831951
export OM_DATUM_LONG=2.1661984

# GPS protocol: UBX for u-blox, NMEA for others
export OM_GPS_PROTOCOL=UBX

# GPS serial port and baudrate
export OM_GPS_PORT=/dev/gps
export OM_GPS_BAUDRATE=921600

# Uncomment for GPS over TCP (e.g. via ser2net)
# export OM_GPS_DEVICE_TYPE="tcp"
# export OM_GPS_TCP_HOSTNAME="10.0.0.161"
# export OM_GPS_TCP_PORT="4002"

# NTRIP RTK correction — required for cm-level accuracy
# Set OM_USE_NTRIP=False if using external radio on Ardusimple board
export OM_USE_NTRIP=True
export OM_NTRIP_HOSTNAME=caster.centipede.fr
export OM_NTRIP_PORT=2101
export OM_NTRIP_USER=centipede
export OM_NTRIP_PASSWORD=centipede
export OM_NTRIP_ENDPOINT=OUIL

# u-blox F9R dead-reckoning sensor fusion (experimental, leave False)
export OM_USE_F9R_SENSOR_FUSION=False

# GPS timing
export OM_GPS_WAIT_TIME_SEC=10.0       # wait for fix after undock
export OM_GPS_TIMEOUT_SEC=5.0          # pause mowing if no fix

################################
##    Mower Logic Settings    ##
################################

# Docking / undocking
export OM_DOCKING_DISTANCE=1.0         # forward into dock (m)
export OM_UNDOCK_DISTANCE=1.5          # reverse out of dock (m)
# export OM_UNDOCK_ANGLED_DISTANCE=0.0 # extra angled undock distance
# export OM_UNDOCK_ANGLE=0.0           # undock angle (neg=left, pos=right)
# export OM_DOCKING_APPROACH_DISTANCE=1.5
# export OM_DOCKING_RETRY_COUNT=4
# export OM_DOCKING_EXTRA_TIME=0.0

# Mowing pattern
export OM_OUTLINE_COUNT=1              # perimeter passes before fill
export OM_OUTLINE_OVERLAP_COUNT=0      # overlap between outline passes
export OM_OUTLINE_OFFSET=0.05          # inward offset from boundary (m)
export OM_TOOL_WIDTH=0.13              # path width — gives overlap with 0.18m blade

# Mowing angle
export OM_MOWING_ANGLE_OFFSET=0        # deg: 0=east, -90=north, or -1=auto
export OM_MOWING_ANGLE_OFFSET_IS_ABSOLUTE=False
export OM_MOWING_ANGLE_INCREMENT=0     # rotate angle each full pass (deg)

# Mowing behaviour
export OM_ENABLE_MOWER=true
# 0=manual, 1=semi-auto (mow once), 2=full auto
export OM_AUTOMATIC_MODE=0

# Battery thresholds (25.2V nominal, 7S Li-ion)
export OM_BATTERY_FULL_VOLTAGE=28.5
export OM_BATTERY_EMPTY_VOLTAGE=24.0   # dock if below for 20s
export OM_BATTERY_CRITICAL_VOLTAGE=23.0 # immediate dock

# Motor temperature limits (°C)
export OM_MOWING_MOTOR_TEMP_HIGH=80.0
export OM_MOWING_MOTOR_TEMP_LOW=40.0

# Rain: 0=ignore, 1=dock, 2=dock_until_dry, 3=pause_auto
export OM_RAIN_MODE=2
export OM_RAIN_DELAY_MINUTES=30

# Hall / emergency inputs (Yardforce 500 default: low-active)
# Hall1+2 = front wheel lift sensors, Hall3+4 = top cover stop-button
# export OM_EMERGENCY_INPUT_CONFIG="!L, !L, !S, !S"
# export OM_EMERGENCY_LIFT_PERIOD="100"
# export OM_EMERGENCY_TILT_PERIOD="2500"

# Heatmap sensor
export OM_HEATMAP_SENSOR_IDS=om_gps_accuracy

################################
##    External MQTT Broker    ##
################################
# Uncomment to publish status to Home Assistant or other MQTT consumers.
# The mosquitto container in this stack listens on localhost:1883.

# export OM_MQTT_ENABLE=True
# export OM_MQTT_HOSTNAME="localhost"
# export OM_MQTT_PORT="1883"
# export OM_MQTT_USER=""
# export OM_MQTT_PASSWORD=""
# export OM_MQTT_TOPIC_PREFIX="mowgli"
