#! /usr/bin/env python -m
# -*- coding: utf-8 -*-
#     _____
#    /  _  \
#   / _/ \  \
#  / / \_/   \
# /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
# \  / \_/ \  / /  _\| |  |  _| / _ \ |  _ \ |  _ \ / _ \ |_   _|| | | |
#  \ \_/ \_/ /  | |  | |  | └─┐| |_| || |/ / | |/ /| |_| |  | |  | └─┘ |
#   \  \_/  /   | |_ | |_ | ┌_┘|  _  ||  _ \ |  _/ |  _  |  | |  | ┌─┐ |
#    \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
#            ROBOTICS™
#
#  File: codes.py
#  Desc: Horizon Message Codes
#  Auth: Malcolm Robert
#  
#  Copyright © 2010 Clearpath Robotics, Inc. 
#  All Rights Reserved
# 
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#      * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#      * Redistributions in binary form must reproduce the above copyright
#        notice, this list of conditions and the following disclaimer in the
#        documentation and/or other materials provided with the distribution.
#      * Neither the name of Clearpath Robotics, Inc. nor the
#        names of its contributors may be used to endorse or promote products
#        derived from this software without specific prior written permission.
# 
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
#  ARE DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
#  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#  Please send comments, questions, or patches to code@clearpathrobotics.com
#




################################################################################
# Script



# Check if run as a script
if __name__ == "__main__":
    
    # Warn of Module ONLY status
    print ("ERROR: clearpath.horizon.codes is a module and can NOT be run"\
           " as a script!\nFor a command-line interface demo of Horizon, run:"\
           "\n  python -m clearpath.horizon.demo\n"\
           "For Horizon message forwarding, run:\n"\
           "  python -m clearpath.horizon.forward")

    # Exit Error
    import sys
    sys.exit(1)




################################################################################
# Module



## @package clearpath.horizon.codes
#  Horizon Message Codes Python Module
# 
#  Horizon Message Codes                                                      \n
#  Supported Horizon version(s): 0.1 - 1.0
#
#  @author     Malcolm Robert
#  @date       07/04/10
#  @req        clearpath.utils                                                \n
#              clearpath.horizon.payloads                                     \n
#              clearpath.horizon.versioning
#  @version    1.0
#
#  @section USE
#
#  The intended purpose of this module is to provide a listing of Horizon
#  message codes (HORIZON_CODES), a mapping of messages to their respective 
#  HorizonPayload class (PAYLOAD_MAP), and a mapping of requests to data 
#  responses (RESPONSE_MAP). No further functionality is provided.
#
#  @section HISTORY
#
#  Version 0.1 - 0.3 {Ryan Gariepy}
#  - Initial Creation as protocol_demo.py
#
#  Version 0.4 {Malcolm Robert}
#  - Move to horizon.py
#  - Added logging
#  - Added version support
#  - Added Doxygen documentation
#  - Changed version scheme to match Horizon doc
#  - Horizon support for v0.4 messages
#
#  Version 0.5
#  - Added TCP and UDP support
#  - Horizon support for v0.5
#
#  Version 0.6
#  - Move to horizon package __init__.py
#  - Horizon support for v0.6
#
#  Version 0.7
#  - Added Encryption Support
#  - Horizon support for v0.7
#
#  Version 0.8
#  - Horizon support for v0.8
#
#  Version 1.0
#  - Move to codes.py
#  - Horizon support for v 0.1 - 1.0
#  - Python 2.6+ & 3.x compatible
#
#  @section License
#  @copydoc public_license
#
"""Horizon Message Codes

   Copyright © 2010 Clearpath Robotics, Inc.
   All rights reserved
   
   Created: 07/04/10
   Authors: Malcolm Robert
   Version: 1.0
   """



# Required Clearpath Modules
from .. import utils            # Clearpath Utilities
from .  import payloads         # Horizon Protocol Message Payload Definitions
from .  import versioning       # Horizon Protocol Versions 

# Required Python Modules
import logging                  # Logging Utilities


# Module Support
## Module Version
__version__  = "1.0"
"""Module Version"""
## SVN Code Revision
__revision__ = "$Revision: 495 $"
""" SVN Code Revision"""


## Supported Horizon Versions
versions = versioning.HORIZON_VERSIONS.copy()
"""Supported Horizon Versions"""


## Message Log
logger = logging.getLogger('clearpath.horizon.codes')
"""Horizon Message Codes Module Log"""
logger.setLevel(logging.NOTSET)
logger.addHandler(utils.NullLoggingHandler())
logger.propagate = False
logger.debug("Loading clearpath.horizon.codes ...")




################################################################################
# Message Codes
################################################################################




## Horizon Message Codes
HORIZON_CODES = {}   # Format: code:name 

## Horizon Message Names (reverse lookup on HORIZON_CODES)
# TODO: cache these, possibly in HORIZON_NAMES
def find_code(message_name):
    for code, name in HORIZON_CODES.iteritems():
        if name == message_name:
            return code


## Codes' Payload Class
PAYLOAD_MAP   = {}              # Format: name:HorizonPayload

## Request Response Codes
RESPONSE_MAP  = {}              # Format: request_name:data_name



# PROPOSED FUTURE VERSION
# tuple:                        set       request     data        payload class
NAMES = {}
NAMES['echo']                = (None,     0x4000,     0x8000,     payloads.HorizonPayload_Echo)


################################################################################
# DATA
################################################################################

   
## Data Code: ECHO
HORIZON_CODES[0x8000] = 'data_echo'
PAYLOAD_MAP['data_echo'] = payloads.HorizonPayload_Echo


## Data Code: PLATFORM INFORMATION
HORIZON_CODES[0x8001] = 'data_platform_info'
PAYLOAD_MAP['data_platform_info'] = payloads.HorizonPayload_PlatformInfo
    

## Data Code: PLATFORM NAME
HORIZON_CODES[0x8002] = 'data_platform_name'
PAYLOAD_MAP['data_platform_name'] = payloads.HorizonPayload_PlatformName
    
    
## Data Code: FIRMWARE INFORMATION
HORIZON_CODES[0x8003] = 'data_firmware_info'
PAYLOAD_MAP['data_firmware_info'] = payloads.HorizonPayload_FirmwareInfo
    

## Data Code: SYSTEM STATUS DATA
HORIZON_CODES[0x8004] = 'data_system_status'
PAYLOAD_MAP['data_system_status'] = payloads.HorizonPayload_SystemStatus
    
    
## Data Code: POWER STATUS DATA
HORIZON_CODES[0x8005] = 'data_power_status'
PAYLOAD_MAP['data_power_status'] = payloads.HorizonPayload_PowerStatus
    

## Data Code: PROCESSOR STATUS DATA
HORIZON_CODES[0x8006] = 'data_processor_status'
PAYLOAD_MAP['data_processor_status'] = payloads.HorizonPayload_ProcessorStatus


## Data Code: SAFETY SYSTEM 
HORIZON_CODES[0x8010] = 'data_safety_status'
PAYLOAD_MAP['data_safety_status'] = payloads.HorizonPayload_SafetyStatus
    

## Data Code: DIFFERENTIAL SPEED
HORIZON_CODES[0x8200] = 'data_differential_speed'
PAYLOAD_MAP['data_differential_speed'] = payloads.HorizonPayload_DifferentialSpeed


## Data Code: DIFFERENTIAL CONTROL
HORIZON_CODES[0x8201] = 'data_differential_control'
PAYLOAD_MAP['data_differential_control'] = payloads.HorizonPayload_DifferentialControl


## Data Code: DIFFERENTIAL MOTORS
HORIZON_CODES[0x8202] = 'data_differential_output'
PAYLOAD_MAP['data_differential_output'] = payloads.HorizonPayload_DifferentialOutput

    
## Data Code: ACKERMANN OUTPUT
HORIZON_CODES[0x8203] = 'data_ackermann_output'
PAYLOAD_MAP['data_ackermann_output'] = payloads.HorizonPayload_AckermannOutput


## Data Code: VELOCITY
HORIZON_CODES[0x8204] = 'data_velocity'
PAYLOAD_MAP['data_velocity'] = payloads.HorizonPayload_Velocity


## Data Code: TURN
HORIZON_CODES[0x8205] = 'data_turn'
PAYLOAD_MAP['data_turn'] = payloads.HorizonPayload_Turn


## Data Code: MAX SPEED
HORIZON_CODES[0x8210] = 'data_max_speed'
PAYLOAD_MAP['data_max_speed'] = payloads.HorizonPayload_MaxSpeed


## Data Code: MAX ACCELERATION
HORIZON_CODES[0x8211] = 'data_max_acceleration'
PAYLOAD_MAP['data_max_acceleration'] = payloads.HorizonPayload_MaxAcceleration


## Data Code: GEAR STATUS
HORIZON_CODES[0x8212] = 'data_gear_status'
PAYLOAD_MAP['data_gear_status'] = payloads.HorizonPayload_GearStatus
    
    
## Data Code: GPADC OUTPUT
HORIZON_CODES[0x8300] = 'data_gpadc_output'
PAYLOAD_MAP['data_gpadc_output'] = payloads.HorizonPayload_GPADCOutput


## Data Code: GPIO
HORIZON_CODES[0x8301] = 'data_gpio'
PAYLOAD_MAP['data_gpio'] = payloads.HorizonPayload_GPIO
    
    
## Data Code: GPADC INPUT
HORIZON_CODES[0x8303] = 'data_gpadc_input'
PAYLOAD_MAP['data_gpadc_input'] = payloads.HorizonPayload_GPADCInput


## Data Code: PAN/TILT/
HORIZON_CODES[0x8400] = 'data_pan_tilt_zoom'
PAYLOAD_MAP['data_pan_tilt_zoom'] = payloads.HorizonPayload_PanTiltZoom


## Data Code: DISTANCE
HORIZON_CODES[0x8500] = 'data_distance'
PAYLOAD_MAP['data_distance'] = payloads.HorizonPayload_Distance


## Data Code: DISTANCE & TIMING
HORIZON_CODES[0x8501] = 'data_distance_timing'
PAYLOAD_MAP['data_distance_timing'] = payloads.HorizonPayload_DistanceTiming


## Data Code: PLATFORM ORIENTATION
HORIZON_CODES[0x8600] = 'data_platform_orientation'
PAYLOAD_MAP['data_platform_orientation'] = payloads.HorizonPayload_Orientation


## Data Code: PLATFORM ROTATION
HORIZON_CODES[0x8601] = 'data_platform_rotation'
PAYLOAD_MAP['data_platform_rotation'] = payloads.HorizonPayload_Rotation


## Data Code: PLATFORM ACCELERATION
HORIZON_CODES[0x8602] = 'data_platform_acceleration'
PAYLOAD_MAP['data_platform_acceleration'] = payloads.HorizonPayload_Acceleration


## Data Code: PLATFORM 6-AXIS
HORIZON_CODES[0x8603] = 'data_platform_6axis'
PAYLOAD_MAP['data_platform_6axis'] = payloads.HorizonPayload_Platform6Axis


## Data Code: PLATFORM 6-AXIS & ORIENTATION
HORIZON_CODES[0x8604] = 'data_platform_6axis_orientation'
PAYLOAD_MAP['data_platform_6axis_orientation'] = payloads.HorizonPayload_Platform6AxisOrientation


## Data Code: PLATFORM MAGNETOMETER
HORIZON_CODES[0x8606] = 'data_platform_magnetometer'
PAYLOAD_MAP['data_platform_magnetometer'] = payloads.HorizonPayload_Magnetometer


## Data Code: ENCODERS
HORIZON_CODES[0x8800] = 'data_encoders'
PAYLOAD_MAP['data_encoders'] = payloads.HorizonPayload_Encoders


## Data Code: RAW ENCODERS
HORIZON_CODES[0x8801] = 'data_raw_encoders'
PAYLOAD_MAP['data_raw_encoders'] = payloads.HorizonPayload_RawEncoders


## Data Code: ABSOLUTE JOINT POSITION
HORIZON_CODES[0x9010] = 'data_absolute_joint_position'
PAYLOAD_MAP['data_absolute_joint_position'] = payloads.HorizonPayload_AbsoluteJointPosition


## Data Code: RELATIVE JOINT POSITION
HORIZON_CODES[0x9011] = 'data_relative_joint_position'
PAYLOAD_MAP['data_relative_joint_position'] = payloads.HorizonPayload_RelativeJointPosition


## Data Code: JOINT CONTROL
HORIZON_CODES[0x9012] = 'data_joint_control'
PAYLOAD_MAP['data_joint_control'] = payloads.HorizonPayload_JointControl


## Data Code: JOINT HOMING STATUS
HORIZON_CODES[0x9013] = 'data_joint_homing_status'
PAYLOAD_MAP['data_joint_homing_status'] = payloads.HorizonPayload_JointHomingStatus


## Data Code: JOINT TORQUES
HORIZON_CODES[0x9014] = 'data_joint_torques'
PAYLOAD_MAP['data_joint_torques'] = payloads.HorizonPayload_JointTorques
    

## Data Code: END EFFECTOR POSITION
HORIZON_CODES[0x9020] = 'data_end_effector_position'
PAYLOAD_MAP['data_end_effector_position'] = payloads.HorizonPayload_EndEffectorPosition


## Data Code: END EFFECTOR POSE
HORIZON_CODES[0x9021] = 'data_end_effector_pose'
PAYLOAD_MAP['data_end_effector_pose'] = payloads.HorizonPayload_EndEffectorPose


## Data Code: END EFFECTOR ORIENTATION 
HORIZON_CODES[0x9022] = 'data_end_effector_orientation'
PAYLOAD_MAP['data_end_effector_orientation'] = payloads.HorizonPayload_EndEffectorOrientation


## Data Code: CURRENT SENSOR CONFIG
HORIZON_CODES[0xA100] = 'data_current_sensor_config'
PAYLOAD_MAP['data_current_sensor_config'] = payloads.HorizonPayload_CurrentSensorConfig


## Data Code: VOLTAGE SENSOR CONFIG
HORIZON_CODES[0xA101] = 'data_voltage_sensor_config'
PAYLOAD_MAP['data_voltage_sensor_config'] = payloads.HorizonPayload_VoltageSensorConfig


## Data Code: TEMPERATURE SENSOR CONFIG
HORIZON_CODES[0xA102] = 'data_temperature_sensor_config'
PAYLOAD_MAP['data_temperature_sensor_config'] = payloads.HorizonPayload_TemperatureSensorConfig


## Data Code: ORIENTATION SENSOR CONFIG
HORIZON_CODES[0xA103] = 'data_orientation_sensor_config'
PAYLOAD_MAP['data_orientation_sensor_config'] = payloads.HorizonPayload_OrientationSensorConfig


## Data Code: GYRO CONFIG
HORIZON_CODES[0xA104] = 'data_gyro_config'
PAYLOAD_MAP['data_gyro_config'] = payloads.HorizonPayload_GyroConfig


## Data Code: ACCELEROMETER CONFIG
HORIZON_CODES[0xA105] = 'data_accelerometer_config'
PAYLOAD_MAP['data_accelerometer_config'] = payloads.HorizonPayload_AccelerometerConfig


## Data Code: ACCELEROMETER CONFIG
HORIZON_CODES[0xA106] = 'data_magnetometer_config'
PAYLOAD_MAP['data_magnetometer_config'] = payloads.HorizonPayload_MagnetometerConfig



## Data Code: RAW CURRENT SENSOR
HORIZON_CODES[0xA110] = 'data_raw_current_sensor'
PAYLOAD_MAP['data_raw_current_sensor'] = payloads.HorizonPayload_RawCurrentSensor


## Data Code: RAW VOLTAGE SENSOR
HORIZON_CODES[0xA111] = 'data_raw_voltage_sensor'
PAYLOAD_MAP['data_raw_voltage_sensor'] = payloads.HorizonPayload_RawVoltageSensor


## Data Code: RAW TEMPERATURE SENSOR
HORIZON_CODES[0xA112] = 'data_raw_temperature_sensor'
PAYLOAD_MAP['data_raw_temperature_sensor'] = payloads.HorizonPayload_RawTemperatureSensor


## Data Code: RAW ORIENTATION SENSOR
HORIZON_CODES[0xA113] = 'data_raw_orientation_sensor'
PAYLOAD_MAP['data_raw_orientation_sensor'] = payloads.HorizonPayload_RawOrientationSensor


## Data Code: RAW GYRO
HORIZON_CODES[0xA114] = 'data_raw_gyro'
PAYLOAD_MAP['data_raw_gyro'] = payloads.HorizonPayload_RawGyro


## Data Code: RAW ACCELEROMETER
HORIZON_CODES[0xA115] = 'data_raw_accelerometer'
PAYLOAD_MAP['data_raw_accelerometer'] = payloads.HorizonPayload_RawAccelerometer


## Data Code: RAW MAGNETOMETER
HORIZON_CODES[0xA116] = 'data_raw_magnetometer'
PAYLOAD_MAP['data_raw_magnetometer'] = payloads.HorizonPayload_RawMagnetometer






################################################################################
# COMMANDS
################################################################################



## Command Code: SET PLATFORM INFO
HORIZON_CODES[0x0001] = 'set_platform_info'
PAYLOAD_MAP['set_platform_info'] = payloads.HorizonPayload_PlatformInfo


## Command Code: SET PLATFORM NAME
HORIZON_CODES[0x0002] = 'set_platform_name'
PAYLOAD_MAP['set_platform_name'] = payloads.HorizonPayload_PlatformName


## Command Code: SET PLATFORM TIME
HORIZON_CODES[0x0005] = 'set_platform_time'
PAYLOAD_MAP['set_platform_time'] = payloads.HorizonPayload_PlatformTime


## Command Code: SET SAFETY SYSTEM
HORIZON_CODES[0x0010] = 'set_safety_status'
PAYLOAD_MAP['set_safety_status'] = payloads.HorizonPayload_SafetyStatus


## Command Code: SET DIFFERENTIAL SPEED
HORIZON_CODES[0x0200] = 'set_differential_speed'
PAYLOAD_MAP['set_differential_speed'] = payloads.HorizonPayload_DifferentialSpeed


## Command Code: SET DIFFERENTIAL CONTROL
HORIZON_CODES[0x0201] = 'set_differential_controls'
PAYLOAD_MAP['set_differential_control'] = payloads.HorizonPayload_DifferentialControl


## Command Code: SET DIFFERENTIAL OUTPUT
HORIZON_CODES[0x0202] = 'set_differential_output'
PAYLOAD_MAP['set_differential_output'] = payloads.HorizonPayload_DifferentialOutput


## Command Code: SET ACKERMANN OUTPUT
HORIZON_CODES[0x0203] = 'set_ackermann_output'
PAYLOAD_MAP['set_ackermann_output'] = payloads.HorizonPayload_AckermannOutput
    
   
## Command Code: SET VELOCITY
HORIZON_CODES[0x0204] = 'set_velocity'
PAYLOAD_MAP['set_velocity'] = payloads.HorizonPayload_Velocity


## Command Code: SET TURN  
HORIZON_CODES[0x0205] = 'set_turn'
PAYLOAD_MAP['set_turn'] = payloads.HorizonPayload_Turn


## Command Code: SET MAX SPEED 
HORIZON_CODES[0x0210] = 'set_max_speed'
PAYLOAD_MAP['set_max_speed'] = payloads.HorizonPayload_MaxSpeed


## Command Code: SET MAX ACCELERATION
HORIZON_CODES[0x0211] = 'set_max_acceleration'
PAYLOAD_MAP['set_max_acceleration'] = payloads.HorizonPayload_MaxAcceleration


## Command Code: SET GEAR
HORIZON_CODES[0x0212] = 'set_gear'
PAYLOAD_MAP['set_gear'] = payloads.HorizonPayload_Gear
    
    
## Command Code: SET GPADC OUTPUT 
HORIZON_CODES[0x0300] = 'set_gpadc_output'
PAYLOAD_MAP['set_gpadc_output'] = payloads.HorizonPayload_GPADC
    

## Command Code: SET GPIO DIRECTION
HORIZON_CODES[0x0301] = 'set_gpio_direction'
PAYLOAD_MAP['set_gpio_direction'] = payloads.HorizonPayload_GPIODirection
    
    
## Command Code: SET GPIO OUTPUT
HORIZON_CODES[0x0302] = 'set_gpio_output'
PAYLOAD_MAP['set_gpio_output'] = payloads.HorizonPayload_GPIOOutput
    

## Command Code: SET PTZ POSITION
HORIZON_CODES[0x0400] = 'set_pan_tilt_zoom'
PAYLOAD_MAP['set_pan_tilt_zoom'] = payloads.HorizonPayload_PanTiltZoom
    
    
## Command Code: SET ABSOLUTE JOINT POSITION
HORIZON_CODES[0x1010] = 'set_absolute_joint_position'
PAYLOAD_MAP['set_absolute_joint_position'] = payloads.HorizonPayload_AbsoluteJointPositionTargets
    

## Command Code: SET RELATIVE JOINT POSITION
HORIZON_CODES[0x1011] = 'set_relative_joint_position'
PAYLOAD_MAP['set_relative_joint_position'] = payloads.HorizonPayload_RelativeJointPositionTargets
    

## Command Code: SET JOINT CONTROL
HORIZON_CODES[0x1012] = 'set_joint_control'
PAYLOAD_MAP['set_joint_control'] = payloads.HorizonPayload_JointControl
    
    
## Command Code: SET JOINT HOMING
HORIZON_CODES[0x1013] = 'set_joint_homing'
PAYLOAD_MAP['set_joint_homing'] = payloads.HorizonPayload_JointHoming
    
    
## Command Code: SET END EFFECTOR POSITION
HORIZON_CODES[0x1020] = 'set_end_effector_position'
PAYLOAD_MAP['set_end_effector_position'] = payloads.HorizonPayload_EndEffectorPosition
    
    
## Command Code: SET END EFFECTOR POSE
HORIZON_CODES[0x1021] = 'set_end_effector_pose'
PAYLOAD_MAP['set_end_effector_pose'] = payloads.HorizonPayload_EndEffectorPose
    

## Command Code: SET RESET
HORIZON_CODES[0x2000] = 'set_reset'
PAYLOAD_MAP['set_reset'] = payloads.HorizonPayload_Reset


## Command Code: RESTORE SYSTEM CONFIG
HORIZON_CODES[0x2001] = 'restore_system_config'
PAYLOAD_MAP['restore_system_config'] = payloads.HorizonPayload_RestoreSystemConfig
 

## Command Code: STORE SYSTEM CONFIG
HORIZON_CODES[0x2002] = 'store_system_config'
PAYLOAD_MAP['store_system_config'] = payloads.HorizonPayload_StoreSystemConfig


## Command Code: SET CURRENT SENSOR CONFIG
HORIZON_CODES[0x2100] = 'set_current_sensor_config'
PAYLOAD_MAP['set_current_sensor_config'] = payloads.HorizonPayload_CurrentSensorConfig


## Command Code: SET VOLTAGE SENSOR CONFIG
HORIZON_CODES[0x2101] = 'set_voltage_sensor_config'
PAYLOAD_MAP['set_voltage_sensor_config'] = payloads.HorizonPayload_VoltageSensorConfig


## Command Code: SET TEMPERATURE SENSOR CONFIG
HORIZON_CODES[0x2102] = 'set_temperature_sensor_config'
PAYLOAD_MAP['set_temperature_sensor_config'] = payloads.HorizonPayload_TemperatureSensorConfig


## Command Code: SET ORIENTATION SENSOR CONFIG
HORIZON_CODES[0x2103] = 'set_orientation_sensor_config'
PAYLOAD_MAP['set_orientation_sensor_config'] = payloads.HorizonPayload_OrientationSensorConfig


## Command Code: SET GYRO CONFIG
HORIZON_CODES[0x2104] = 'set_gyro_config'
PAYLOAD_MAP['set_gyro_config'] = payloads.HorizonPayload_GyroConfig


## Command Code: SET ACCELEROMETER CONFIG
HORIZON_CODES[0x2105] = 'set_accelerometer_config'
PAYLOAD_MAP['set_accelerometer_config'] = payloads.HorizonPayload_AccelerometerConfig


## Command Code: SET MAGNETOMETER CONFIG
HORIZON_CODES[0x2106] = 'set_magnetometer_config'
PAYLOAD_MAP['set_magnetometer_config'] = payloads.HorizonPayload_MagnetometerConfig



################################################################################
# REQUESTS
################################################################################


## Request Code: REQUEST ECHO  
HORIZON_CODES[0x4000] = 'request_echo'
PAYLOAD_MAP['request_echo'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_echo'] = 'data_echo'


## Request Code: REQUEST PLATFORM INFO
HORIZON_CODES[0x4001] = 'request_platform_info'
PAYLOAD_MAP['request_platform_info'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_platform_info'] = 'data_platform_info'
    
    
## Request Code: REQUEST PLATFORM NAME
HORIZON_CODES[0x4002] = 'request_platform_name'
PAYLOAD_MAP['request_platform_name'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_platform_name'] = 'data_platform_name'


## Request Code: REQUEST FIRMWARE INFO
HORIZON_CODES[0x4003] = 'request_firmware_info'
PAYLOAD_MAP['request_firmware_info'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_firmware_info'] = 'data_firmware_info'


## Request Code: REQUEST SYSTEM STATUS
HORIZON_CODES[0x4004] = 'request_system_status'
PAYLOAD_MAP['request_system_status'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_system_status'] = 'data_system_status'


## Request Code: REQUEST POWER STATUS
HORIZON_CODES[0x4005] = 'request_power_status'
PAYLOAD_MAP['request_power_status'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_power_status'] = 'data_power_status'


## Request Code: REQUEST PROCESSOR STATUS
HORIZON_CODES[0x4006] = 'request_processor_status'
PAYLOAD_MAP['request_processor_status'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_processor_status'] = 'data_processor_status'


## Request Code: REQUEST SAFETY SYSTEM
HORIZON_CODES[0x4010] = 'request_safety_status'
PAYLOAD_MAP['request_safety_status'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_safety_status'] = 'data_safety_status'


## Request Code: REQUEST DIFFERENTIAL SPEED
HORIZON_CODES[0x4200] = 'request_differential_speed'
PAYLOAD_MAP['request_differential_speed'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_differential_speed'] = 'data_differential_speed'


## Request Code: REQUEST DIFFERENTIAL CONTROL
HORIZON_CODES[0x4201] = 'request_differential_control'
PAYLOAD_MAP['request_differential_control'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_differential_control'] = 'data_differential_control'


## Request Code: REQUEST DIFFERENTIAL OUTPUT
HORIZON_CODES[0x4202] = 'request_differential_output'
PAYLOAD_MAP['request_differential_output'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_differential_output'] = 'data_differential_output'


## Request Code: REQUEST ACKERMANN OUTPUT
HORIZON_CODES[0x4203] = 'request_ackermann_output'
PAYLOAD_MAP['request_ackermann_output'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_ackermann_output'] = 'data_ackermann_output'


## Request Code: REQUEST VELOCITY 
HORIZON_CODES[0x4204] = 'request_velocity'
PAYLOAD_MAP['request_velocity'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_velocity'] = 'data_velocity'


## Request Code: REQUEST TURN
HORIZON_CODES[0x4205] = 'request_turn'
PAYLOAD_MAP['request_turn'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_turn'] = 'data_turn'


## Request Code: REQUEST MAX SPEED
HORIZON_CODES[0x4210] = 'request_max_speed'
PAYLOAD_MAP['request_max_speed'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_max_speed'] = 'data_max_speed'


## Request Code: REQUEST MAX ACCELERATION
HORIZON_CODES[0x4211] = 'request_max_acceleration'
PAYLOAD_MAP['request_max_acceleration'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_max_acceleration'] = 'data_max_acceleration'


## Request Code: REQUEST GEAR STATUS
HORIZON_CODES[0x4212] = 'request_gear_status'
PAYLOAD_MAP['request_gear_status'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_gear_status'] = 'data_gear_status'


## Request Code: REQUEST GPADC OUTPUT
HORIZON_CODES[0x4300] = 'request_gpadc_output'
PAYLOAD_MAP['request_gpadc_output'] = payloads.HorizonPayload_Request_GPADCOutput
RESPONSE_MAP['request_gpadc_output'] = 'data_gpadc_output'


## Request Code: REQUEST GPIO
HORIZON_CODES[0x4301] = 'request_gpio'
PAYLOAD_MAP['request_gpio'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_gpio'] = 'data_gpio'


## Request Code: REQUEST GPADC INPUT
HORIZON_CODES[0x4303] = 'request_gpadc_input'
PAYLOAD_MAP['request_gpadc_input'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_gpadc_input'] = 'data_gpadc_input'


## Request Code: REQUEST PAN/TILT/ZOOM
HORIZON_CODES[0x4400] = 'request_pan_tilt_zoom'
PAYLOAD_MAP['request_pan_tilt_zoom'] = payloads.HorizonPayload_Request_PanTiltZoom
RESPONSE_MAP['request_pan_tilt_zoom'] = 'data_pan_tilt_zoom'


## Request Code: REQUEST DISTANCE
HORIZON_CODES[0x4500] = 'request_distance'
PAYLOAD_MAP['request_distance'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_distance'] = 'data_distance'


## Request Code: REQUEST DISTANCE & TIMING
HORIZON_CODES[0x4501] = 'request_distance_timing'
PAYLOAD_MAP['request_distance_timing'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_distance_timing'] = 'data_distance_timing'


## Request Code: REQUEST PLATFORM ORIENTATION
HORIZON_CODES[0x4600] = 'request_platform_orientation'
PAYLOAD_MAP['request_platform_orientation'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_platform_orientation'] = 'data_platform_orientation'


## Request Code: REQUEST PLATFORM ROTATION
HORIZON_CODES[0x4601] = 'request_platform_rotation'
PAYLOAD_MAP['request_platform_rotation'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_platform_rotation'] = 'data_platform_rotation'


## Request Code: REQUEST PLATFORM ACCELERATION
HORIZON_CODES[0x4602] = 'request_platform_acceleration'
PAYLOAD_MAP['request_platform_acceleration'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_platform_acceleration'] = 'data_platform_acceleration'


## Request Code: REQUEST PLATFORM 6-AXIS
HORIZON_CODES[0x4603] = 'request_platform_6axis'
PAYLOAD_MAP['request_platform_6axis'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_platform_6axis'] = 'data_platform_6axis'


## Request Code: REQUEST PLATFORM 6-AXIS & ORIENTATION
HORIZON_CODES[0x4604] = 'request_platform_6axis_orientation'
PAYLOAD_MAP['request_platform_6axis_orientation'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_platform_6axis_orientation'] = 'data_platform_6axis_orientation'


## Request Code: REQUEST PLATFORM MAGNETOMETER
HORIZON_CODES[0x4606] = 'request_platform_magnetometer'
PAYLOAD_MAP['request_platform_magnetometer'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_platform_magnetometer'] = 'data_platform_magnetometer'


## Request Code: REQUEST ENCODERS
HORIZON_CODES[0x4800] = 'request_encoders'
PAYLOAD_MAP['request_encoders'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_encoders'] = 'data_encoders'


## Request Code: REQUEST RAW ENCODERS
HORIZON_CODES[0x4801] = 'request_raw_encoders'
PAYLOAD_MAP['request_raw_encoders'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_raw_encoders'] = 'data_raw_encoders'


## Request Code: REQUEST ABSOLUTE JOINT POSITION
HORIZON_CODES[0x5010] = 'request_absolute_joint_position'
PAYLOAD_MAP['request_absolute_joint_position'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_absolute_joint_position'] = 'data_absolute_joint_position'


## Request Code: REQUEST RELATIVE JOINT POSITION
HORIZON_CODES[0x5011] = 'request_relative_joint_position'
PAYLOAD_MAP['request_relative_joint_position'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_relative_joint_position'] = 'data_relative_joint_position'


## Request Code: REQUEST JOINT CONTROL
HORIZON_CODES[0x5012] = 'request_joint_control'
PAYLOAD_MAP['request_joint_control'] = payloads.HorizonPayload_Request_JointControl
RESPONSE_MAP['request_joint_control'] = 'data_joint_control'


## Request Code: REQUEST JOINT HOMING STATUS
HORIZON_CODES[0x5013] = 'request_joint_homing_status'
PAYLOAD_MAP['request_joint_homing_status'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_joint_homing_status'] = 'data_joint_homing_status'


## Request Code: REQUEST JOINT TORQUES
HORIZON_CODES[0x5014] = 'request_joint_torques'
PAYLOAD_MAP['request_joint_torques'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_joint_torques'] = 'data_joint_torques'


## Request Code: REQUEST END EFFECTOR POSITION
HORIZON_CODES[0x5020] = 'request_end_effector_position'
PAYLOAD_MAP['request_end_effector_position'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_end_effector_position'] = 'data_end_effector_position'


## Request Code: REQUEST END EFFECTOR POSE
HORIZON_CODES[0x5021] = 'request_end_effector_pose'
PAYLOAD_MAP['request_end_effector_pose'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_end_effector_pose'] = 'data_end_effector_pose'


## Request Code: REQUEST END EFFECTOR ORIENTATION
HORIZON_CODES[0x5022] = 'request_end_effector_orientation'
PAYLOAD_MAP['request_end_effector_orientation'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_end_effector_orientation'] = 'data_end_effector_orientation'


HORIZON_CODES[0x6100] = 'request_current_sensor_config'
PAYLOAD_MAP['request_current_sensor_config'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_current_sensor_config'] = 'data_current_sensor_config'


HORIZON_CODES[0x6101] = 'request_voltage_sensor_config'
PAYLOAD_MAP['request_voltage_sensor_config'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_voltage_sensor_config'] = 'data_voltage_sensor_config'


HORIZON_CODES[0x6102] = 'request_temperature_sensor_config'
PAYLOAD_MAP['request_temperature_sensor_config'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_temperature_sensor_config'] = 'data_temperature_sensor_config'


HORIZON_CODES[0x6103] = 'request_orientation_sensor_config'
PAYLOAD_MAP['request_orientation_sensor_config'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_orientation_sensor_config'] = 'data_orientation_sensor_config'


HORIZON_CODES[0x6104] = 'request_gyro_config'
PAYLOAD_MAP['request_gyro_config'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_gyro_config'] = 'data_gyro_config'


HORIZON_CODES[0x6105] = 'request_accelerometer_config'
PAYLOAD_MAP['request_accelerometer_config'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_accelerometer_config'] = 'data_accelerometer_config'


HORIZON_CODES[0x6106] = 'request_magnetometer_config'
PAYLOAD_MAP['request_magnetometer_config'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_magnetometer_config'] = 'data_magnetometer_config'




HORIZON_CODES[0x6110] = 'request_raw_current_sensor'
PAYLOAD_MAP['request_raw_current_sensor'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_raw_current_sensor'] = 'data_raw_current_sensor'


HORIZON_CODES[0x6111] = 'request_raw_voltage_sensor'
PAYLOAD_MAP['request_raw_voltage_sensor'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_raw_voltage_sensor'] = 'data_raw_voltage_sensor'


HORIZON_CODES[0x6112] = 'request_raw_temperature_sensor'
PAYLOAD_MAP['request_raw_temperature_sensor'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_raw_temperature_sensor'] = 'data_raw_temperature_sensor'


HORIZON_CODES[0x6113] = 'request_raw_orientation_sensor'
PAYLOAD_MAP['request_raw_orientation_sensor'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_raw_orientation_sensor'] = 'data_raw_orientation_sensor'


HORIZON_CODES[0x6114] = 'request_raw_gyro'
PAYLOAD_MAP['request_raw_gyro'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_raw_gyro'] = 'data_raw_gyro'


HORIZON_CODES[0x6115] = 'request_raw_accelerometer'
PAYLOAD_MAP['request_raw_accelerometer'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_raw_accelerometer'] = 'data_raw_accelerometer'


HORIZON_CODES[0x6116] = 'request_raw_magnetometer'
PAYLOAD_MAP['request_raw_magnetometer'] = payloads.HorizonPayload_Request
RESPONSE_MAP['request_raw_magnetometer'] = 'data_raw_magnetometer'



logger.debug("... clearpath.horizon.codes loaded.")
