import time
import serial
import struct

class MSRException(Exception):
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

# defining the core object
class MSR605(serial.Serial):
    # protocol
    ESC_CHR = '\x1B'
    FS_CHR = '\x1C'
    END_CHR = '\x1C'

    HiCo = True
    HiBPI = True

    enabled_tracks = [1, 2, 3]

    def __init__(self, dev):
        super(MSR605, self).__init__(dev, 9600, 8, serial.PARITY_NONE, timeout=10)
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
        if status in exceptions.keys():
            raise exceptions[status]()
        return status

    def _expect(self, data):
        assert self.read(len(data)) == data

    def _read_until(self, end_byte):
        data = ""
        while True:
            byte = self.read(1)
            if byte == end_byte:
                return data
            data += byte

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
        return self.read(2) == (self.ESC_CHR + '\x30')

    def communication_test(self):
        self._send_command('\x65')
        return self.read(2) == (self.ESC_CHR + '\x79')

    def ram_test(self):
        self._send_command('\x87')
        return self.read(2) == (self.ESC_CHR + '\x30')

    def reset(self):
        self._send_command('\x61')

    def read_iso(self):
        self._send_command('\x72')
        self._expect(self.ESC_CHR + '\x73')
        self._expect(self.ESC_CHR)
        track1 = self._read_until(self.ESC_CHR)
        assert track1[:2] == '\x01%' and track1[-1] == '?'
        track2 = self._read_until(self.ESC_CHR)
        assert track2[:2] == '\x02;' and track2[-1] == '?'
        track3 = self._read_until(self.FS_CHR)
        assert track3[:2] == ('\x03' + self.ESC_CHR) and track3[-1] == '?'

        return track1[2:-1], track2[2:-1], track3[2:-1]

    def read_raw(self):
        self._send_command('\x6D')
        self._expect(self.ESC_CHR + '\x73')
        tracks = [''] * 3
        for tn in self.enabled_tracks:
            self._expect(self.ESC_CHR + chr(tn))
            str_len = struct.unpack('B', self.read(1))[0]
            tracks[tn - 1] = self.read(str_len)
        self._expect('\x3F' + self.FS_CHR)
        self._read_status()
        return tracks

    def _send_command(self, command, *args):
        self.flushInput()
        self.write(self.ESC_CHR + command + ''.join(args))

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

    def set_leading_zero(self, t13, t2):
        self._send_command('\x7A', chr(t13), chr(t2))
        self._read_status()

    def check_leading_zero(self):
        self._send_command('\x6C')
        self._expect(self.ESC_CHR)
        return ord(self.read(1)), ord(self.read(1))

    def erase_card(self, t1=True, t2=True, t3=True):
        flags = (t1 and 1 or 0) | ((t2 and 1 or 0) << 1) | ((t3 and 1 or 0) << 2)
        self._send_command('\x63', chr(flags))
        self._read_status()

    def select_bpi(self, t1_density, t2_density, t3_density):
        self._send_command('\x62', t1_density and '\xD2' or '\x4B')
        self._read_status()
        self._send_command('\x62', t2_density and '\xA1' or '\xA0')
        self._read_status()
        self._send_command('\x62', t3_density and '\xC1' or '\xC0')
        self._read_status()

    def write_raw(self, *tracks):
        raw_data_block = self.ESC_CHR + '\x73'
        for tn, track in enumerate(tracks):
            raw_data_block += \
                self.ESC_CHR +\
                chr(tn + 1) +\
                chr(len(track)) +\
                track
        raw_data_block += '\x3F\x1C'
        print repr(raw_data_block)
        self._send_command('\x6E', raw_data_block)
        self._read_status()
