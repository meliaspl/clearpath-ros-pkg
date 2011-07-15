#!/usr/bin/python

import math, time, serial

import roslib; roslib.load_manifest('clearpath_sensors')
import rospy
from gps_common.msg import GPSFix

METERS_PER_SEC_PER_KNOT = 0.514444444
FIRST_TIMEOUT_SECS = 3.0
TIMEOUT_SECS = 1.2
RETRY_OPEN_SECS = 1.0


class GPS:
    def __init__(self):
        rospy.init_node('nmea_gps')
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.serial = None

        self.start_serial()
        self.pub = rospy.Publisher('fix', GPSFix)
        

    def start_serial(self):
        if self.serial:
            self.serial.close()
            self.serial = None

        error = None
        while True:
            try:
                self.serial = serial.Serial(self.port, 4800, timeout=0)
                self.timeout = time.clock() + FIRST_TIMEOUT_SECS
                rospy.loginfo("Opened GPS on port: %s" % self.port)
                break;

            except serial.SerialException as e:
                if not error:
                    error = e
                    rospy.logerr("Error opening GPS serial port: %s" % error)
                    rospy.logerr("Will retry opening GPS every second.")
                time.sleep(RETRY_OPEN_SECS)


    def run(self):
        buffer = "";
        while not rospy.is_shutdown():
            raw = self.serial.read(255)
            if len(raw) > 0:
                self.timeout = time.clock() + TIMEOUT_SECS
                chunks = raw.split('$')
                buffer += chunks.pop(0)
                while len(chunks) > 0:
                    self.process(buffer)
                    buffer = chunks.pop(0)
            else:
                if time.clock() > self.timeout:
                    rospy.logerr("GPS timed out. Attempting to reopen serial port.")
                    self.start_serial()
                    

    def process(self, buffer):
        fields = buffer.split(',')
        if fields[0] == 'GPRMC' and fields[2] == 'A':
            fix_msg = GPSFix()
            fix_msg.latitude = self._lat(fields)
            fix_msg.longitude = self._lon(fields)
            fix_msg.track = float(fields[8])
            fix_msg.speed = float(fields[7]) * METERS_PER_SEC_PER_KNOT
            self.pub.publish(fix_msg)


    @staticmethod
    def _lat(fields):
        inp = float(fields[3])
        out = math.floor(inp / 100) + (inp % 100) / 60
        if fields[4] == 'S':
            out =- out
        return out

        
    @staticmethod
    def _lon(fields):
        inp = float(fields[5])
        out = math.floor(inp / 100) + (inp % 100) / 60
        if fields[6] == 'W':
            out =- out
        return out


GPS().run()
