#!/usr/bin/env python


from collections import namedtuple

from . import check
from .. import utils


class Payload(check.Fields):
    def __init__(self, **kwargs):
        # Setting of field values has already happened in the 
        # tuple setup, in __new__. We just validate them here,
        # and raise any exceptions necessary. For recieved data,
        # any ValueErrors thrown will be captured by the Receiver
        # thread and assigned to Payload.error. For outbound messages
        # where the data is user-originated, Errors thrown will
        # bubble back to the user.
        # logger.debug("Payload: %s" % repr(self))
        self.validate()

    @classmethod
    def parse(cls, data):
        '''Payloads which implement this may be received on the wire, and
        this method will parse the list of bytes and return a Payload object.''' 
        raise NotImplementedError("Payload does not have a parse method.")

    def data(self):
        '''Payloads which may be sent in outbound messages will implement this
        method, which produces a list of bytes. For most messages, the data()
        output will be identical to what is expected for parse(). However, not
        all the messages are symmetrical!''' 
        raise NotImplementedError("Payload does not have a data output method.")

    def validate(self):
        '''All payloads must implement validation, using Fields.check()'''
        raise NotImplementedError("Payloade does not have a validate method.")


class DifferentialSpeed(Payload, check.metatuple('left_speed m/s', 'right_speed m/s',
                                                 'left_accel m/s^2', 'right_accel m/s^2')):

    @classmethod
    def parse(cls, data):
        return cls(left_speed = utils.to_short(data[0:2], 100), 
                    right_speed = utils.to_short(data[2:4], 100),
                    left_accel = utils.to_short(data[4:6], 100), 
                    right_accel = utils.to_short(data[6:8], 100))

    def data(self):
        data = []
        data += utils.from_short(self.left_speed, 100)
        data += utils.from_short(self.right_speed, 100)
        data += utils.from_short(self.left_accel, 100)
        data += utils.from_short(self.right_accel, 100)
        return data;

    def validate(self):
        self.check('left_speed right_speed').range(-320, 320)
        self.check('left_accel right_accel').range(0, 320)


class SystemStatus(Payload, check.metatuple('uptime sec %d', 'voltages V', 
                                            'currents A', 'temperatures C')):

    @classmethod
    def parse(cls, data):
        pass
    #    return cls(left_speed = utils.to_short(data[0:2], 100), 
    #                right_speed = utils.to_short(data[2:4], 100),
    #                left_accel = utils.to_short(data[4:6], 100), 
    #                right_accel = utils.to_short(data[6:8], 100))

    def validate(self):
        self.check('uptime').range(0, 4000000)
        self.check('voltages currents temperatures').each().range(0, 320)



payload = DifferentialSpeed.parse([0x6E, 0x00, 0x88, 0xFF, 0xC8, 0x00, 0xC8, 0x00])
print(payload)
print(repr(payload))

payload = DifferentialSpeed(left_speed = 0.8, right_speed = 0.8, left_accel = 0.2, right_accel = 0.2)
print(payload.data())
print(payload.hex())
print(repr(payload))

# payload = DifferentialSpeed(left_speed = 0.8, right_speed = 400, left_accel = 0.2, right_accel = 0.2)
payload = SystemStatus(uptime=100, voltages=[1,200,3], currents=[100,101,120], temperatures=[])
print(payload)
