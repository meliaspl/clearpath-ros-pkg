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


from . import payloads         # Horizon Protocol Message Payload Definitions

from collections import namedtuple


__version__  = "1.0"
__revision__ = "$Revision: 765 $"

VERSION_BYTE = 0x00

Code = namedtuple('Code', 'set request data payload')

codes = {}
names = {}

def extend(new_codes):
    global codes, names
    codes.update(new_codes)
    for name, code in new_codes.items():
        if code.set: names[code.set] = name
        if code.request: names[code.request] = name
        if code.data: names[code.data] = name


extend({
    'echo':          Code( None, 0x4000, 0x8000, payloads.Echo ),
    'platform_info': Code( 0x0001, 0x4001, 0x8001, payloads.PlatformInfo ),
    'platform_name': Code( 0x0002, 0x4002, 0x8002, payloads.PlatformName ),
    'firmware_info': Code( None, 0x4003, 0x8003, payloads.FirmwareInfo ),
    'system_status': Code( None, 0x4004, 0x8004, payloads.SystemStatus ),
    'platform_time': Code( 0x0005, None, None, payloads.PlatformTime ),
    'power_status':  Code( None, 0x4005, 0x8005, payloads.PowerStatus ),
    'processor_status': Code( None, 0x4006, 0x8006, payloads.ProcessorStatus ),
    'safety_status': Code( 0x0010, 0x4010, 0x8010, payloads.SafetyStatus ),
    'differential_speed': Code( 0x0200, 0x4200, 0x8200, payloads.DifferentialSpeed ),
    'differential_control': Code( 0x0201, 0x4201, 0x8201, payloads.DifferentialControl ),
    'differential_output': Code( 0x0202, 0x4202, 0x8202, payloads.DifferentialOutput ),
    'ackermann_output': Code( 0x0203, 0x4203, 0x8203, payloads.AckermannOutput ),
    'differential_current': Code( 0x0220, 0x4220, 0x8220, payloads.DifferentialCurrent ),
    'differential_current_control': Code( 0x0221, 0x4221, 0x8221, payloads.DifferentialCurrentControl ),
    'velocity':      Code( 0x0204, 0x4204, 0x8204, payloads.Velocity ),
    'turn':          Code( 0x0205, 0x4205, 0x8205, payloads.Turn ),
    'max_speed':     Code( 0x0210, 0x4210, 0x8210, payloads.MaxSpeed ),
    'max_accel': Code( 0x0211, 0x4211, 0x8211, payloads.MaxAccel ),
    'gear_status':   Code( 0x0212, 0x4212, 0x8212, payloads.GearStatus ),
    'gpadc_output':  Code( 0x0300, 0x4300, 0x8300, payloads.GPADCOutput ),
    'gpio':          Code( 0x0301, 0x4301, 0x8301, payloads.GPIO ),
    'gpio_output':   Code( 0x0302, None, None, payloads.GPIOOutput ),
    'gpadc_input':   Code( None, 0x4303, 0x8303, payloads.GPADCInput ),
    'pan_tilt_zoom': Code( 0x0400, 0x4400, 0x8400, payloads.PanTiltZoom ),
    'distance':      Code( None, 0x4500, 0x8500, payloads.Distance ),
    'distance_timing': Code( None, 0x4501, 0x8501, payloads.DistanceTiming ),
    'platform_orientation': Code( None, 0x4600, 0x8600, payloads.Orientation ),
    'platform_rotation': Code( None, 0x4601, 0x8601, payloads.Rotation ),
    'platform_acceleration': Code( None, 0x4602, 0x8602, payloads.Acceleration ),
    'platform_6axis': Code( None, 0x4603, 0x8603, payloads.Platform6Axis ),
    'platform_6axis_orientation': Code( None, 0x4604, 0x8604, payloads.Platform6AxisOrientation ),
    'platform_magnetometer': Code( None, 0x4606, 0x8606, payloads.Magnetometer ),
    'encoders':      Code( None, 0x4800, 0x8800, payloads.Encoders ),
    'raw_encoders':  Code( None, 0x4801, 0x8801, payloads.RawEncoders ),
    'encoders_config': Code( 0x0802, 0x4802, 0x8802, payloads.EncodersConfig ),
    'absolute_joint_position': Code( 0x1010, 0x5010, 0x9010, payloads.AbsoluteJointPosition ),
    'relative_joint_position': Code( 0x1011, 0x5011, 0x9011, payloads.RelativeJointPosition ),
    'joint_control': Code( 0x1012, 0x5012, 0x9012, payloads.JointControl ),
    'joint_homing_status': Code( 0x1013, 0x5013, 0x9013, payloads.JointHomingStatus ),
    'joint_torques': Code( None, 0x5014, 0x9014, payloads.JointTorques ),
    'end_effector_position': Code( 0x1020, 0x5020, 0x9020, payloads.EndEffectorPosition ),
    'end_effector_pose': Code( 0x1021, 0x5021, 0x9021, payloads.EndEffectorPose ),
    'end_effector_orientation': Code( None, 0x5022, 0x9022, payloads.EndEffectorOrientation ),
    'reset':         Code( 0x2000, None, None, payloads.Reset ),
    'restore_system_config': Code( 0x2001, None, None, payloads.RestoreSystemConfig ),
    'store_system_config': Code( 0x2002, None, None, payloads.StoreSystemConfig ),
    'current_sensor_config': Code( 0x2100, 0x6100, 0xA100, payloads.CurrentSensorConfig ),
    'voltage_sensor_config': Code( 0x2101, 0x6101, 0xA101, payloads.VoltageSensorConfig ),
    'temperature_sensor_config': Code( 0x2102, 0x6102, 0xA102, payloads.TemperatureSensorConfig ),
    'orientation_sensor_config': Code( 0x2103, 0x6103, 0xA103, payloads.OrientationSensorConfig ),
    'gyro_config':   Code( 0x2104, 0x6104, 0xA104, payloads.GyroConfig ),
    'accelerometer_config': Code( 0x2105, 0x6105, 0xA105, payloads.AccelerometerConfig ),
    'magnetometer_config': Code( 0x2106, 0x6106, 0xA106, payloads.MagnetometerConfig ),
    'battery_estimation_config': Code( 0x2107, 0x6107, 0xA107, payloads.BatteryEstimationConfig ),
    'platform_kinematics': Code( 0x2108, 0x6108, 0xA108, payloads.PlatformKinematics ),
    'raw_current_sensor': Code( None, 0x6110, 0xA110, payloads.RawCurrentSensor ),
    'raw_voltage_sensor': Code( None, 0x6111, 0xA111, payloads.RawVoltageSensor ),
    'raw_temperature_sensor': Code( None, 0x6112, 0xA112, payloads.RawTemperatureSensor ),
    'raw_orientation_sensor': Code( None, 0x6113, 0xA113, payloads.RawOrientationSensor ),
    'raw_gyro':      Code( None, 0x6114, 0xA114, payloads.RawGyro ),
    'raw_accelerometer': Code( None, 0x6115, 0xA115, payloads.RawAccelerometer ),
    'raw_magnetometer': Code( None, 0x6116, 0xA116, payloads.RawMagnetometer ),
    'control_flags': Code( 0x2130, 0x6130, 0xA130, payloads.ControlFlags ) })
