import time
import serial
import re
import codecs

class MSRException(Exception):
    pass

class ReadError(MSRException):
    pass

class ReadWriteError(MSRException):
    pass

class CommandFormatError(MSRException):
    pass

class InvalidCommand(MSRException):
    pass

class InvalidCardSwipeForWrite(MSRException):
    pass

class SetError(MSRException):
    pass

class MSR605(serial.Serial):
    ESC_CHR = '\x1B'
    FS_CHR = '\x1C'

    def __init__(self, dev, test=True):
        super(MSR605, self).__init__(dev, 9600, 8, serial.PARITY_NONE, timeout=10)
        self.reset()
        if test:
            self.communication_test()
            self.ram_test()
            self.sensor_test()
            self.reset()

    def _read_status(self):
        exceptions = {
            '\x31': ReadWriteError,
            '\x32': CommandFormatError,
            '\x34': InvalidCommand,
            '\x39': InvalidCardSwipeForWrite,
            '\x41': SetError,
        }
        self._expect(self.ESC_CHR)
        status = self.read(1)
        if status in exceptions:
            raise exceptions[status]()
        return status

    def _expect(self, data):
        read_data = self.read(len(data))
        if read_data != data:
            raise ReadError('Expected %s, got %s.' % (repr(data), repr(read_data)))

    def _read_until(self, end):
        data = ''
        while True:
            data += self.read(1)
            if data.endswith(end):
                return data

    def _send_command(self, command, *args):
        self.flushInput()
        self.flushOutput()
        self.write(self.ESC_CHR + command + ''.join(args))

    def all_leds_off(self):
        self._send_command('\x81')

    def all_leds_on(self):
        self._send_command('\x82')

    def led_green_on(self):
        self._send_command('\x83')

    def led_yellow_on(self):
        self._send_command('\x84')

    def led_red_on(self):
        self._send_command('\x85')

    def sensor_test(self):
        self._send_command('\x86')
        if self.read(2) != (self.ESC_CHR + '\x30'):
            raise MSRException('Sensor test failed.')

    def communication_test(self):
        self._send_command('\x65')
        if self.read(2) != (self.ESC_CHR + '\x79'):
            raise MSRException('Communication test failed.')

    def ram_test(self):
        self._send_command('\x87')
        if self.read(2) != (self.ESC_CHR + '\x30'):
            raise MSRException('RAM test failed.')

    def reset(self):
        self._send_command('\x61')

    def read_iso(self):
        self._set_iso_mode()
        self._send_command('\x72')
        self._expect(self.ESC_CHR + '\x73')
        self._expect(self.ESC_CHR + '\x01')
        track1 = self._read_until(self.ESC_CHR + '\x02')[:-2]
        track2 = self._read_until(self.ESC_CHR + '\x03')[:-2]
        track3 = self._read_until(self.FS_CHR)[:-1]
        self._read_status()
        return track1, track2, track3

    def read_raw(self):
        self._send_command('\x6D')
        self._expect(self.ESC_CHR + '\x73')
        tracks = [''] * 3
        for tn in xrange(1, 4):
            self._expect(self.ESC_CHR + chr(tn))
            str_len = ord(self.read(1))
            tracks[tn - 1] = self.read(str_len)
        self._expect('\x3F' + self.FS_CHR)
        self._read_status()
        return tracks

    def get_device_model(self):
        self._send_command('\x74')
        self._expect(self.ESC_CHR)
        model = self.read(1)
        self._expect('S')
        return model

    def get_firmware_version(self):
        self._send_command('\x76')
        self._expect(self.ESC_CHR)
        return self.read(8)

    def set_hico(self):
        self._send_command('\x78')
        self._expect(self.ESC_CHR + '\x30')

    def set_lowco(self):
        self._send_command('\x79')
        self._expect(self.ESC_CHR + '\x30')

    def get_co_status(self):
        self._send_command('\x79')
        self._expect(self.ESC_CHR)
        return self.read(1)

    def set_leading_zero(self, t13, t2):
        self._send_command('\x7A', chr(t13), chr(t2))
        self._read_status()

    def check_leading_zero(self):
        self._send_command('\x6C')
        self._expect(self.ESC_CHR)
        t13 = ord(self.read(1))
        t2 = ord(self.read(1))
        return t13, t2

    def erase_card(self, t1=True, t2=True, t3=True):
        flags = (t1 and 1 or 0) | (t2 and 2 or 0) | (t3 and 4 or 0)
        self._send_command('\x63', chr(flags))
        self._read_status()

    def select_bpi(self, t1_density, t2_density, t3_density):
        self._send_command('\x62', t1_density and '\xD2' or '\x4B')
        self._read_status()
        self._send_command('\x62', t2_density and '\xA1' or '\xA0')
        self._read_status()
        self._send_command('\x62', t3_density and '\xC1' or '\xC0')
        self._read_status()

    def set_bpc(self, t1, t2, t3):
        self._send_command('\x6F', chr(t1), chr(t2), chr(t3))
        self._expect(self.ESC_CHR + '\x30' + chr(t1) + chr(t2) + chr(t3))

    def write_raw(self, *tracks):
        raw_data_block = self.ESC_CHR + '\x73'
        for tn, track in enumerate(tracks):
            raw_data_block += \
                self.ESC_CHR +\
                chr(tn + 1) +\
                chr(len(track)) +\
                track
        raw_data_block += '\x3F' + self.FS_CHR
        self._send_command('\x6E', raw_data_block)
        self._read_status()

    def _set_iso_mode(self):
        self.select_bpi(True, False, True)
        self.set_bpc(7, 5, 5)

    def write_iso(self, *tracks):
        self._set_iso_mode()
        tracks = (re.sub(r'^%|^;|\?$', '', track) for track in tracks)
        data_block = self.ESC_CHR + '\x73'
        data_block += ''.join(
            self.ESC_CHR + chr(tn + 1) + track
            for tn, track in enumerate(tracks)
        )
        data_block += '\x3F' + self.FS_CHR
        self._send_command('\x77', raw_data_block)
        self._read_status()

    def read_iso_soft(self):
        self._set_iso_mode()
        track1, track2, track3 = self.read_raw()
        return (
            track1.decode('iso7811-2-track1'),
            track2.decode('iso7811-2-track2'),
            track3.decode('iso7811-2-track3'),
        )

    def write_iso_soft(self, *tracks):
        self._set_iso_mode()
        tracks = [re.sub(r'^%|^;|\?$', '', track) for track in tracks]
        tracks[0] = ('%' + tracks[0] + '?').encode('iso7811-2-track1')
        tracks[1] = (';' + tracks[1] + '?').encode('iso7811-2-track2')
        tracks[2] = (';' + tracks[2] + '?').encode('iso7811-2-track3')
        return self.write_raw(*tracks)


class ISO7811_2(codecs.Codec):
    TRACK1_CHARS = ' !"#$%&\'()*+`,./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_'
    TRACK23_CHARS = '0123456789:;<=>?'

    @classmethod
    def _reverse_bits(cls, value, nbits):
        return sum(
            1 << (nbits - 1 - i)
            for i in xrange(nbits)
            if (value >> i) & 1
        )

    @classmethod
    def _with_parity(cls, value, nbits):
        if sum(1 for i in xrange(nbits) if (value >> i) & 1) % 2 != 0:
            return value
        return value | (1 << (nbits - 1))

    @classmethod
    def _iso_encode_data(cls, data, mapping, nbits):
        def make_data():
            lrc = 0
            for v in map(mapping.index, data):
                lrc ^= v
                yield chr(cls._with_parity(v, nbits))
            yield chr(cls._with_parity(lrc, nbits))
        enc = ''.join(make_data())
        return enc, len(enc)

    @classmethod
    def _iso_decode_data(cls, data, mapping, nbits):
        dec = ''.join(
            mapping[cls._reverse_bits(ord(c) >> 1, nbits - 1)]
            for c in data
        )
        return dec, len(dec)

    @classmethod
    def encode_track1(cls, data):
        return cls._iso_encode_data(data, cls.TRACK1_CHARS, 7)

    @classmethod
    def encode_track23(cls, data):
        return cls._iso_encode_data(data, cls.TRACK23_CHARS, 5)

    @classmethod
    def decode_track1(cls, data):
        return cls._iso_decode_data(data, cls.TRACK1_CHARS, 7)

    @classmethod
    def decode_track23(cls, data):
        return cls._iso_decode_data(data, cls.TRACK23_CHARS, 5)

    @classmethod
    def codec_search(cls, name):
        return {
            'iso7811-2-track1': (cls.encode_track1, cls.decode_track1, None, None),
            'iso7811-2-track2': (cls.encode_track23, cls.decode_track23, None, None),
            'iso7811-2-track3': (cls.encode_track23, cls.decode_track23, None, None),
        }.get(name, None)
codecs.register(ISO7811_2.codec_search)
