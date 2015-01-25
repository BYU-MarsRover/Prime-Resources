# BYU Mars Rover Communications Protocol Definitions
# Use tabs=4, written in Notepad++
#
# !!! BE ADVISED !!!
# This file is generated from RoverLink.xml using RoverLinkGenerator.py
# If this file needs to be modified, make the appropriate changes in the XML file.
# The XML file is used to generate the same protocol in the Basestation GUI, 
# thus synchronizing the entire comms system.
#
# Protocol notes are inserted at the bottom of this document.

# Build Timestemp: 2014-12-03T22:57:40.128000

class RoverLinkDefs:

	# Constants
	ROVERLINK_SYS_HEALTH_MSG_ID = 0
	ROVERLINK_ROVER_STATUS_MSG_ID = 1
	ROVERLINK_ARM_STATUS_MSG_ID = 2
	ROVERLINK_RESERVED_1_MSG_ID = 3
	ROVERLINK_GPS_MSG_ID = 4
	ROVERLINK_IMU_MSG_ID = 5
	ROVERLINK_RESERVED_2_MSG_ID = 6
	ROVERLINK_LRS_MSG_ID = 7
	ROVERLINK_DRIVE_MSG_ID = 8
	ROVERLINK_VIDEO_MSG_ID = 9
	ROVERLINK_ARM_MSG_ID = 10
	ROVERLINK_ISHAAMA_MSG_ID = 11
	ROVERLINK_LIFERAY_MSG_ID = 12
	ROVERLINK_CUSTOM_DEBUG_1_MSG_ID = 13
	ROVERLINK_CUSTOM_DEBUG_2_MSG_ID = 14
	ROVERLINK_CUSTOM_DEBUG_3_MSG_ID = 15
	ROVERLINK_PING_MSG_ID = 60
	ROVERLINK_TELEMETRY_CONFIG_MSG_ID = 61
	ROVERLINK_REQ_FIELDS_ONCE_MSG_ID = 62
	ROVERLINK_BROADCAST_MSG_ID = 63

	ROVERLINK_SYS_HEALTH_MSG_LEN = 13
	ROVERLINK_ROVER_STATUS_MSG_LEN = 13
	ROVERLINK_ARM_STATUS_MSG_LEN = 9
	ROVERLINK_RESERVED_1_MSG_LEN = 0
	ROVERLINK_GPS_MSG_LEN = 26
	ROVERLINK_IMU_MSG_LEN = 18
	ROVERLINK_RESERVED_2_MSG_LEN = 0
	ROVERLINK_LRS_MSG_LEN = 16
	ROVERLINK_DRIVE_MSG_LEN = 4
	ROVERLINK_VIDEO_MSG_LEN = 7
	ROVERLINK_ARM_MSG_LEN = 14
	ROVERLINK_ISHAAMA_MSG_LEN = 4
	ROVERLINK_LIFERAY_MSG_LEN = 1
	ROVERLINK_CUSTOM_DEBUG_1_MSG_LEN = 16
	ROVERLINK_CUSTOM_DEBUG_2_MSG_LEN = 16
	ROVERLINK_CUSTOM_DEBUG_3_MSG_LEN = 16
	ROVERLINK_PING_MSG_LEN = 1
	ROVERLINK_TELEMETRY_CONFIG_MSG_LEN = 6
	ROVERLINK_REQ_FIELDS_ONCE_MSG_LEN = 2
	ROVERLINK_BROADCAST_MSG_LEN = 1


	ROVERLINK_MSG_LEN_DICT = {
		ROVERLINK_SYS_HEALTH_MSG_LEN : -1,
		ROVERLINK_ROVER_STATUS_MSG_LEN : -1,
		ROVERLINK_ARM_STATUS_MSG_LEN : -1,
		ROVERLINK_RESERVED_1_MSG_LEN : -1,
		ROVERLINK_GPS_MSG_LEN : -1,
		ROVERLINK_IMU_MSG_LEN : -1,
		ROVERLINK_RESERVED_2_MSG_LEN : -1,
		ROVERLINK_LRS_MSG_LEN : -1,
		ROVERLINK_DRIVE_MSG_LEN : -1,
		ROVERLINK_VIDEO_MSG_LEN : -1,
		ROVERLINK_ARM_MSG_LEN : -1,
		ROVERLINK_ISHAAMA_MSG_LEN : -1,
		ROVERLINK_LIFERAY_MSG_LEN : -1,
		ROVERLINK_CUSTOM_DEBUG_1_MSG_LEN : -1,
		ROVERLINK_CUSTOM_DEBUG_2_MSG_LEN : -1,
		ROVERLINK_CUSTOM_DEBUG_3_MSG_LEN : -1,
		ROVERLINK_PING_MSG_LEN : -1,
		ROVERLINK_TELEMETRY_CONFIG_MSG_LEN : -1,
		ROVERLINK_REQ_FIELDS_ONCE_MSG_LEN : -1,
		ROVERLINK_BROADCAST_MSG_LEN : -1,
		}

	ROVERLINK_WRITE = 0xC0;
	ROVERLINK_REQUEST = 0x40;
	ROVERLINK_RESPONSE = 0x00;


	# List of aliases. Entries represent the last entry in IP address. (192.168.1.1 for Basestation, 1.10 for Arm, etc)
	BASESTATION_1 = 1
	BASESTATION_2 = 2
	BASESTATION_3 = 3
	MAIN_CONTROLLER = 13
	ARM_CONTROLLER = 10
	BROADCAST_ALIAS = 15

	# systemState field in the heartbeat and status messages that indicates board health
	STARTUP = 0
	NORMAL = 1
	FAILSAFE = 2
	ERROR = 3

	# fix_type in the GPS message
	NONE = 0
	FIX_2D = 1
	FIX_3D = 2
	OTHER = 3

	# The Telemetry Config and One-Shot Message Mask. Basically, 2^(Message Id) forms the bitmask field.
	SYS_HEALTH = 0x0001
	ROVER_STATUS = 0x0002
	ARM_STATUS = 0x0004
	RESERVED_1 = 0x0008
	GPS = 0x0010
	IMU = 0x0020
	RESERVED_2 = 0x0040
	LRS = 0x0080
	DRIVE = 0x0100
	VIDEO = 0x0200
	ARM = 0x0400
	ISHAAMA = 0x0800
	LIFERAY = 0x1000
	CUSTOM_DEBUG_1 = 0x2000
	CUSTOM_DEBUG_2 = 0x4000
	CUSTOM_DEBUG_3 = 0x8000

	# Broadcast Message
	SAVE_FAILSAFE_VALUES_NOW = 0x01
	PANIC_STOP = 0xF0

# -- Message Class Instances
# Each board must provide a status report of its health periodically.
class sys_health:
	def __init__(self,):
		self.systemTimestamp = 0	# Milliseconds from boot
		self.systemState = 0	# System status code, see enum definition
		self.cpuLoad = 0	# The CPU Load, 0% means idle, 100% means saturated
		self.busErrors = 0	# Count of I2C/SPI Bus errors
		self.ethernetErrors = 0	# Count of Ethernet errors
		self.someOtherMetric = 0	# Probably packet lag time, etc
	def toList():
		return self.__dict__


# The Main Controller Board handles the Camera and Gimbal, Drive and Power systems.
class rover_status:
	def __init__(self,):
		self.systemState = 0	# System status code, see enum definition
		self.roverVoltage = 0	# Voltage in mV, Ex: 12450 means 12.450 Volts
		self.roverCurrent = 0	# Current in 10mA, Ex: 16500 means 165.00 Amps
		self.mAhCounter = 0	# Battery used in mAh
		self.ubntLinkInteg = 0	# 100% means perfect link, 0% is lost link, in 0.1% units
		self.dragonLinkRSSI = 0	# Same as ubntLinkInteg, for backup-link LRS receiver
	def toList():
		return self.__dict__


# The Arm Controller Board handles the Arm, including PID controllers. May also handle soil taster "ISHAAMA".
class arm_status:
	def __init__(self,):
		self.systemState = 0	# System status code
		self.dynamixelErrors = 0	# Number of bad reads or writes to Dynamixel servos.
		self.pid1_error = 0	# Error as in the error term, a scalar value.
		self.pid2_error = 0	# Same as above, for the 2nd PID controller
		self.pid3_error = 0	# Same as above, for the 3rd PID controller
	def toList():
		return self.__dict__


# Reserved for future use.
class reserved_1:
	def __init__(self,):
	def toList():
		return self.__dict__


# Parsed NMEA sentence data. May disband these fields in favor of echoing entire NMEA sentence.
class gps:
	def __init__(self,):
		self.systemTimestamp = 0	# Milliseconds from boot
		self.lat = 0	# Latitude in 1E-7 units
		self.lon = 0	# Longitude in 1e-7 units
		self.alt = 0	# Altitude in millimeters
		self.hdop = 0	# Horizontal DOP in centimeters
		self.vdop = 0	# Vertical DOP in centimeters
		self.vel = 0	# Velocity in cm/sec
		self.course = 0	# Course in 0.1 degrees units
		self.fix_type = 0	# Fix Type, see the enum definition
		self.sats = 0	# Number of satellites locked on
	def toList():
		return self.__dict__


# The samples from the IMU, probably a MPU-9250. North-East-Down conventions.
class imu:
	def __init__(self,):
		self.xacc = 0	# X Acceleration, Front+
		self.yacc = 0	# Y Acceleration, Starboard+
		self.zacc = 0	# Z Acceleration, Down+
		self.xgyro = 0	# X Rotation Rate
		self.ygyro = 0	# Y Rotation Rate
		self.zgyro = 0	# Z Rotation Rate
		self.xmag = 0	# X B-Field Component
		self.ymag = 0	# Y B-Field Component
		self.zmag = 0	# Z B-Field Component
	def toList():
		return self.__dict__


# Reserved for future use.
class reserved_2:
	def __init__(self,):
	def toList():
		return self.__dict__


# The DragonLink LRS is the backup R/C receiver on the rover, for if the Rocket M2 modems lose link.
class lrs:
	def __init__(self,):
		self.ppmCh1 = 1500	# Channel 1, usually Roll (Rover Left-Right)
		self.ppmCh2 = 1500	# Channel 2, usually Pitch (Rover Foward-Back)
		self.ppmCh3 = 1500	# Channel 3, throttle stick (Gimbal Tilt)
		self.ppmCh4 = 1500	# Channel 4, rudder (Gimbal Pan)
		self.ppmCh5 = 1500	# Channel 5
		self.ppmCh6 = 1500	# Channel 6
		self.ppmCh7 = 1500	# Channel 7, Backup-Link Asserts control if >1800us, doubles as Panic Stop
		self.ppmCh8 = 1500	# Channel 8, DragonLink in Failsafe Mode if >1800us
	def toList():
		return self.__dict__


# Drive commands for rover movement.
class drive:
	def __init__(self,):
		self.driveFwd = 0	# Forward Drive Component
		self.driveTurn = 0	# Turning Drive Component
	def toList():
		return self.__dict__


# Camera selection and zoom, Gimbal Pan and Tilt.
class video:
	def __init__(self,):
		self.gimbalPan = 0	# Gimbal Pan Value
		self.gimbalTilt = 0	# Gimbal Tilt Value
		self.gimbalZoom = 0	# Gimbal Zoom Value
		self.camSelect = 0	# Camera Mux Selector
	def toList():
		return self.__dict__


# Arm/joint positions and Drill and Claw setpoints.
class arm:
	def __init__(self,):
		self.baseAzimuth = 0	# Also known as turret, pans entire arm
		self.shoulder = 0	# Shoulder: first pitch joint
		self.elbow = 0	# Elbow: second pitch joint
		self.wristTilt = 0	# Wrist flap motion, final pitch joint
		self.wristRotate = 0	# Wrist roll motion
		self.effectorA = 0	# Open/Close Claw
		self.effectorB = 0	# Roller or Drill, configuration-dependant
	def toList():
		return self.__dict__


# The soil taste tester. Likely is Read-Only, nothing to write to. Assuming 12-bit ADC
class ishaama:
	def __init__(self,):
		self.hygrometer = 0	# The Soil Hygrometer probe
		self.phMeter = 0	# The Soil ph Meter probe
	def toList():
		return self.__dict__


# Big scary laser.
class liferay:
	def __init__(self,):
		self.laserDutyCycle = 0	# Set the laser's intensity
	def toList():
		return self.__dict__


# All the custom fields can be anything, but the payload size (16 bytes) is constant.
class custom_debug_1:
	def __init__(self,):
		self.custom00 = 0	# Dynamixel Servo 1 Stress/Current Value
		self.custom01 = 0	# Dynamixel Servo 2 Stress/Current Value
		self.custom02 = 0	# Dynamixel Servo 3 Stress/Current Value
		self.custom03 = 0	# Dynamixel Servo 4 Stress/Current Value
		self.custom04 = 0	# Description
		self.custom05 = 0	# Description
		self.custom06 = 0	# Description
		self.custom07 = 0	# Description
	def toList():
		return self.__dict__


# All the custom fields can be anything, but the payload size (16 bytes) is constant.
class custom_debug_2:
	def __init__(self,):
		self.custom20 = 0	# Description
		self.custom21 = 0	# Description
		self.custom22 = 0	# Description
		self.custom23 = 0	# Description
		self.custom24 = 0	# Description
		self.custom25 = 0	# Description
		self.custom26 = 0	# Description
		self.custom27 = 0	# Description
	def toList():
		return self.__dict__


# All the custom fields can be anything, but the payload size (16 bytes) is constant.
class custom_debug_3:
	def __init__(self,):
		self.custom30 = 0	# Description
		self.custom31 = 0	# Description
		self.custom32 = 0	# Description
		self.custom33 = 0	# Description
		self.custom34 = 0	# Description
		self.custom35 = 0	# Description
		self.custom36 = 0	# Description
		self.custom37 = 0	# Description
	def toList():
		return self.__dict__


# Used to verify that the entire UDP comm system is functional. Payload proves this if uniquely generated at basestation and compared to response.
class ping:
	def __init__(self,):
		self.payload[8] = 0	# Description
	def toList():
		return self.__dict__


# The 16 rover messages can be assigned to send cyclically by setting bits in these bitmasks.
class telemetry_config:
	def __init__(self,):
		self.telemetryMask_10HZ = 0b0000000000000110	# Fast Telemetry Loop
		self.telemetryMask_1HZ = 0b0011111110110001	# Intermediate Loop
		self.telemetryMask_10SEC = 0b0100000000000000	# Slow Loop
	def toList():
		return self.__dict__


# Just one time instead of configuring continuous telemetry. Probably superfiluous if Read/Write mechanism of this protocol can stimulate a single register.
class req_fields_once:
	def __init__(self,):
		self.oneShotMask = 0	# Request a set of messages only one time, not set for cyclic
	def toList():
		return self.__dict__


# Broadcast a message to all controller boards at once. Used for universal commands like Panic Stop, etc
class broadcast:
	def __init__(self,):
		self.broadcastCommand = 0	# See broadcast_t description
	def toList():
		return self.__dict__


