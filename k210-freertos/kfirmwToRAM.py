#!/usr/bin/env python3
import sys
import time
import zlib
import copy
import struct
from enum import Enum
import binascii
import hashlib
import argparse
import math
import zipfile, tempfile
import json
import re
import os

BASH_TIPS = dict(NORMAL='\033[0m',BOLD='\033[1m',DIM='\033[2m',UNDERLINE='\033[4m',
                    DEFAULT='\033[39m', RED='\033[31m', YELLOW='\033[33m', GREEN='\033[32m',
                    BG_DEFAULT='\033[49m', BG_WHITE='\033[107m')

ERROR_MSG   = BASH_TIPS['RED']+BASH_TIPS['BOLD']+'[ERROR]'+BASH_TIPS['NORMAL']
WARN_MSG    = BASH_TIPS['YELLOW']+BASH_TIPS['BOLD']+'[WARN]'+BASH_TIPS['NORMAL']
INFO_MSG    = BASH_TIPS['GREEN']+BASH_TIPS['BOLD']+'[INFO]'+BASH_TIPS['NORMAL']

VID_LIST_FOR_AUTO_LOOKUP = "(1A86)|(0403)|(067B)|(10C4)"
#                            WCH    FTDI    PL     CL
timeout = 0.5

MAX_RETRY_TIMES = 10

class TimeoutError(Exception): pass

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print(ERROR_MSG,'PySerial must be installed, run '+BASH_TIPS['GREEN']+'`pip3 install pyserial`',BASH_TIPS['DEFAULT'])
    sys.exit(1)


def printProgressBar (iteration, total, prefix = '', suffix = '', decimals = 1, length = 100, fill = '█'):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        length      - Optional  : character length of bar (Int)
        fill        - Optional  : bar fill character (Str)
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print('\r%s |%s| %s%% %s' % (prefix, bar, percent, suffix), end = '\r')
    # Print New Line on Complete
    if iteration == total:
        print()

def slip_reader(port):
    partial_packet = None
    in_escape = False

    while True:
        waiting = port.inWaiting()
        read_bytes = port.read(1 if waiting == 0 else waiting)
        if read_bytes == b'':
            raise Exception("Timed out waiting for packet %s" % ("header" if partial_packet is None else "content"))
        for b in read_bytes:

            if type(b) is int:
                b = bytes([b])  # python 2/3 compat

            if partial_packet is None:  # waiting for packet header
                if b == b'\xc0':
                    partial_packet = b""
                else:
                    raise Exception('Invalid head of packet (%r)' % b)
            elif in_escape:  # part-way through escape sequence
                in_escape = False
                if b == b'\xdc':
                    partial_packet += b'\xc0'
                elif b == b'\xdd':
                    partial_packet += b'\xdb'
                else:
                    raise Exception('Invalid SLIP escape (%r%r)' % (b'\xdb', b))
            elif b == b'\xdb':  # start of escape sequence
                in_escape = True
            elif b == b'\xc0':  # end of packet
                yield partial_packet
                partial_packet = None
            else:  # normal byte in packet
                partial_packet += b


class ISPResponse:
    class ISPOperation(Enum):
        ISP_ECHO = 0xC1
        ISP_NOP = 0xC2
        ISP_MEMORY_WRITE = 0xC3
        ISP_MEMORY_READ = 0xC4
        ISP_MEMORY_BOOT = 0xC5
        ISP_DEBUG_INFO = 0xD1

    class ErrorCode(Enum):
        ISP_RET_DEFAULT = 0
        ISP_RET_OK = 0xE0
        ISP_RET_BAD_DATA_LEN = 0xE1
        ISP_RET_BAD_DATA_CHECKSUM = 0xE2
        ISP_RET_INVALID_COMMAND = 0xE3

    @staticmethod
    def parse(data):
        op = data[0]
        reason = data[1]
        text = ''
        try:
            if ISPResponse.ISPOperation(op) == ISPResponse.ISPOperation.ISP_DEBUG_INFO:
                text = data[2:].decode()
        except ValueError:
            print('Warning: recv unknown op', op)

        return (op, reason, text)


class FlashModeResponse:
    class Operation(Enum):
        ISP_DEBUG_INFO = 0xD1
        ISP_NOP = 0xD2
        ISP_FLASH_ERASE = 0xD3
        ISP_FLASH_WRITE = 0xD4
        ISP_REBOOT = 0xD5
        ISP_UARTHS_BAUDRATE_SET = 0xD6
        FLASHMODE_FLASH_INIT = 0xD7

    class ErrorCode(Enum):
        ISP_RET_DEFAULT = 0
        ISP_RET_OK = 0xE0
        ISP_RET_BAD_DATA_LEN = 0xE1
        ISP_RET_BAD_DATA_CHECKSUM = 0xE2
        ISP_RET_INVALID_COMMAND = 0xE3

    @staticmethod
    def parse(data):
        op = data[0]
        reason = data[1]
        text = ''
        if FlashModeResponse.Operation(op) == FlashModeResponse.Operation.ISP_DEBUG_INFO:
            text = data[2:].decode()

        return (op, reason, text)


def chunks(l, n):
    """Yield successive n-sized chunks from l."""
    for i in range(0, len(l), n):
        yield l[i:i + n]


class MAIXLoader:
    def change_baudrate(self, baudrate):
        print(INFO_MSG,"Selected Baudrate: ", baudrate, BASH_TIPS['DEFAULT'])
        out = struct.pack('III', 0, 4, baudrate)
        crc32_checksum = struct.pack('I', binascii.crc32(out) & 0xFFFFFFFF)
        out = struct.pack('HH', 0xd6, 0x00) + crc32_checksum + out
        self.write(out)
        time.sleep(0.05)
        self._port.baudrate = baudrate

    def __init__(self, port='/dev/ttyUSB1', baudrate=115200):
        # configure the serial connections (the parameters differs on the device you are connecting to)
        self._port = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.1
        )
        print(INFO_MSG, "Default baudrate for ISP is", baudrate, ".",  BASH_TIPS['DEFAULT'])

        self._port.isOpen()
        self._slip_reader = slip_reader(self._port)

    """ Read a SLIP packet from the serial port """

    def read(self):
        return next(self._slip_reader)

    """ Write bytes to the serial port while performing SLIP escaping """

    def write(self, packet):
        buf = b'\xc0' \
              + (packet.replace(b'\xdb', b'\xdb\xdd').replace(b'\xc0', b'\xdb\xdc')) \
              + b'\xc0'
        #print('[WRITE]', binascii.hexlify(buf))
        return self._port.write(buf)

    def read_loop(self):
        out = b''
        # while self._port.inWaiting() > 0:
        #     out += self._port.read(1)

        # print(out)
        while 1:
            sys.stdout.write('[RECV] raw data: ')
            sys.stdout.write(binascii.hexlify(self._port.read(1)).decode())
            sys.stdout.flush()

    def recv_one_return(self):
        timeout_init = time.time()
        data = b''
        # find start boarder
        #sys.stdout.write('[RECV one return] raw data: ')
        while 1:
            if time.time() - timeout_init > timeout:
                raise TimeoutError
            c = self._port.read(1)
            #sys.stdout.write(binascii.hexlify(c).decode())
            sys.stdout.flush()
            if c == b'\xc0':
                break

        in_escape = False
        while 1:
            if time.time() - timeout_init > timeout:
                raise TimeoutError
            c = self._port.read(1)
            #sys.stdout.write(binascii.hexlify(c).decode())
            sys.stdout.flush()
            if c == b'\xc0':
                break

            elif in_escape:  # part-way through escape sequence
                in_escape = False
                if c == b'\xdc':
                    data += b'\xc0'
                elif c == b'\xdd':
                    data += b'\xdb'
                else:
                    raise Exception('Invalid SLIP escape (%r%r)' % (b'\xdb', b))
            elif c == b'\xdb':  # start of escape sequence
                in_escape = True

            data += c

        #sys.stdout.write('\n')
        return data

    def reset_to_isp_kd233(self):
        self._port.dtr = False
        self._port.rts = False
        time.sleep(0.01)
        #print('-- RESET to LOW, IO16 to HIGH --')
        # Pull reset down and keep 10ms
        self._port.dtr = True
        self._port.rts = False
        time.sleep(0.01)
        #print('-- IO16 to LOW, RESET to HIGH --')
        # Pull IO16 to low and release reset
        self._port.rts = True
        self._port.dtr = False
        time.sleep(0.01)

    def reset_to_isp_dan(self):
        self._port.dtr = False
        self._port.rts = False
        time.sleep(0.01)
        #print('-- RESET to LOW, IO16 to HIGH --')
        # Pull reset down and keep 10ms
        self._port.dtr = False
        self._port.rts = True
        time.sleep(0.01)
        #print('-- IO16 to LOW, RESET to HIGH --')
        # Pull IO16 to low and release reset
        self._port.rts = False
        self._port.dtr = True
        time.sleep(0.01)

    def reset_to_boot_kd233(self):
        self._port.dtr = False
        self._port.rts = False
        time.sleep(0.01)
        #print('-- RESET to LOW --')
        # Pull reset down and keep 10ms
        self._port.dtr = True
        self._port.rts = False
        time.sleep(0.01)
        #print('-- RESET to HIGH, BOOT --')
        # Pull IO16 to low and release reset
        self._port.rts = False
        self._port.dtr = False
        time.sleep(0.01)
    

    def reset_to_boot_dan(self):
        self._port.dtr = False
        self._port.rts = False
        time.sleep(0.01)
        #print('-- RESET to LOW --')
        # Pull reset down and keep 10ms
        self._port.dtr = False
        self._port.rts = True
        time.sleep(0.01)
        #print('-- RESET to HIGH, BOOT --')
        # Pull IO16 to low and release reset
        self._port.rts = False
        self._port.dtr = False
        time.sleep(0.01)

    def greeting(self):
        self._port.write(b'\xc0\xc2\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xc0')
        op, reason, text = ISPResponse.parse(self.recv_one_return())

        #print('MAIX return op:', ISPResponse.ISPOperation(op).name, 'reason:', ISPResponse.ErrorCode(reason).name)


    def flash_greeting(self):
        retry_count = 0
        while 1:
            self._port.write(b'\xc0\xd2\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xc0')
            retry_count = retry_count + 1
            try:
                op, reason, text = FlashModeResponse.parse(self.recv_one_return())
            except IndexError:
                if retry_count > MAX_RETRY_TIMES:
                    print(ERROR_MSG,"Failed to Connect to K210's Stub",BASH_TIPS['DEFAULT'])
                    sys.exit(1)
                time.sleep(0.1)
                continue
                print(WARN_MSG,"Unexcepted Return recevied, retrying...",BASH_TIPS['DEFAULT'])
            #print('MAIX return op:', FlashModeResponse.Operation(op).name, 'reason:',
            #      FlashModeResponse.ErrorCode(reason).name)
            if FlashModeResponse.Operation(op) == FlashModeResponse.Operation.ISP_NOP:
                print(INFO_MSG,"Boot to Flashmode Successfully",BASH_TIPS['DEFAULT'])
                break
            else:
                if retry_count > MAX_RETRY_TIMES:
                    print(ERROR_MSG,"Failed to Connect to K210's Stub",BASH_TIPS['DEFAULT'])
                    sys.exit(1)
                print(WARN_MSG,"Unexcepted Return recevied, retrying...",BASH_TIPS['DEFAULT'])
                time.sleep(0.1)
                continue

    def boot(self, address=0x80000000):
        print(INFO_MSG,"Booting From " + hex(address),BASH_TIPS['DEFAULT'])

        out = struct.pack('II', address, 0)

        crc32_checksum = struct.pack('I', binascii.crc32(out) & 0xFFFFFFFF)

        out = struct.pack('HH', 0xc5, 0x00) + crc32_checksum + out  # op: ISP_MEMORY_WRITE: 0xc3
        self.write(out)

    def recv_debug(self):
        op, reason, text = ISPResponse.parse(self.recv_one_return())
        #print('[RECV] op:', ISPResponse.ISPOperation(op).name, 'reason:', ISPResponse.ErrorCode(reason).name)
        if text:
            print('-' * 30)
            print(text)
            print('-' * 30)
        if ISPResponse.ErrorCode(reason) not in (ISPResponse.ErrorCode.ISP_RET_DEFAULT, ISPResponse.ErrorCode.ISP_RET_OK):
            print('Failed, retry, errcode=', hex(reason))
            return False
        return True

    def flash_recv_debug(self):
        op, reason, text = FlashModeResponse.parse(self.recv_one_return())
        #print('[Flash-RECV] op:', FlashModeResponse.Operation(op).name, 'reason:',
        #      FlashModeResponse.ErrorCode(reason).name)
        if text:
            print('-' * 30)
            print(text)
            print('-' * 30)

        if FlashModeResponse.ErrorCode(reason) not in (FlashModeResponse.ErrorCode.ISP_RET_OK, FlashModeResponse.ErrorCode.ISP_RET_OK):
            print('Failed, retry')
            return False
        return True

    def init_flash(self, chip_type):
        chip_type = int(chip_type)
        print(INFO_MSG,"Selected Flash: ",("In-Chip", "On-Board")[chip_type],BASH_TIPS['DEFAULT'])
        out = struct.pack('II', chip_type, 0)
        crc32_checksum = struct.pack('I', binascii.crc32(out) & 0xFFFFFFFF)

        out = struct.pack('HH', 0xd7, 0x00) + crc32_checksum + out

        sent = self.write(out)
        op, reason, text = FlashModeResponse.parse(self.recv_one_return())
        #print('MAIX return op:', FlashModeResponse.Operation(op).name, 'reason:',
        #      FlashModeResponse.ErrorCode(reason).name)

    def flash_dataframe(self, data, address=0x80000000):
        DATAFRAME_SIZE = 1024
        data_chunks = chunks(data, DATAFRAME_SIZE)
        #print('[DEBUG] flash dataframe | data length:', len(data))
        total_chunk = math.ceil(len(data)/DATAFRAME_SIZE)

        for n, chunk in enumerate(data_chunks):
            while 1:
                #print('[INFO] sending chunk', i, '@address', hex(address), 'chunklen', len(chunk))
                out = struct.pack('II', address, len(chunk))

                crc32_checksum = struct.pack('I', binascii.crc32(out + chunk) & 0xFFFFFFFF)

                out = struct.pack('HH', 0xc3, 0x00) + crc32_checksum + out + chunk  # op: ISP_MEMORY_WRITE: 0xc3
                sent = self.write(out)
                #print('[INFO]', 'sent', sent, 'bytes', 'checksum', binascii.hexlify(crc32_checksum).decode())

                address += len(chunk)

                if self.recv_debug():
                    break
            printProgressBar(n+1, total_chunk, prefix = 'Downloading ISP:', suffix = '', length = 50)

    def dump_to_flash(self, data, address=0):
        '''
        typedef struct __attribute__((packed)) {
            uint8_t op;
            int32_t checksum; // 下面的所有字段都要参与checksum的计算
            uint32_t address;
            uint32_t data_len;
            uint8_t data_buf[1024];
        } isp_request_t;
        '''

        DATAFRAME_SIZE = 4096
        data_chunks = chunks(data, DATAFRAME_SIZE)
        #print('[DEBUG] flash dataframe | data length:', len(data))



        for n, chunk in enumerate(data_chunks):
            #print('[INFO] sending chunk', i, '@address', hex(address))
            out = struct.pack('II', address, len(chunk))

            crc32_checksum = struct.pack('I', binascii.crc32(out + chunk) & 0xFFFFFFFF)

            out = struct.pack('HH', 0xd4, 0x00) + crc32_checksum + out + chunk
            #print("[$$$$]", binascii.hexlify(out[:32]).decode())
            retry_count = 0
            while True:
                try:
                    sent = self.write(out)
                    #print('[INFO]', 'sent', sent, 'bytes', 'checksum', crc32_checksum)
                    self.flash_recv_debug()
                except:
                    retry_count = retry_count + 1
                    if retry_count > MAX_RETRY_TIMES:
                        print(ERROR_MSG,"Error Count Exceeded, Stop Trying",BASH_TIPS['DEFAULT'])
                        sys.exit(1)
                    continue
                break
            address += len(chunk)



    def flash_erase(self):
        #print('[DEBUG] erasing spi flash.')
        self._port.write(b'\xc0\xd3\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xc0')
        op, reason, text = FlashModeResponse.parse(self.recv_one_return())
        #print('MAIX return op:', FlashModeResponse.Operation(op).name, 'reason:',
        #      FlashModeResponse.ErrorCode(reason).name)

    def install_flash_bootloader(self, data):
        # 1. 刷入 flash bootloader
        self.flash_dataframe(data, address=0x80000000)

    def flash_firmware(self, firmware_bin: bytes, aes_key: bytes = None, address_offset = 0, sha256Prefix = True):
        #print('[DEBUG] flash_firmware DEBUG: aeskey=', aes_key)

        if sha256Prefix == True:
            # 固件加上头
            # 格式: SHA256(after)(32bytes) + AES_CIPHER_FLAG (1byte) + firmware_size(4bytes) + firmware_data
            aes_cipher_flag = b'\x01' if aes_key else b'\x00'

            # 加密
            if aes_key:
                enc = AES_128_CBC(aes_key, iv=b'\x00'*16).encrypt
                padded = firmware_bin + b'\x00'*15 # zero pad
                firmware_bin = b''.join([enc(padded[i*16:i*16+16]) for i in range(len(padded)//16)])

            firmware_len = len(firmware_bin)

            data = aes_cipher_flag + struct.pack('I', firmware_len) + firmware_bin

            sha256_hash = hashlib.sha256(data).digest()

            firmware_with_header = data + sha256_hash

            total_chunk = math.ceil(len(firmware_with_header)/4096)
            # 3. 分片刷入固件
            data_chunks = chunks(firmware_with_header, 4096)  # 4kb for a sector
        else:
            total_chunk = math.ceil(len(firmware_bin)/4096)
            data_chunks = chunks(firmware_bin, 4096)

        for n, chunk in enumerate(data_chunks):
            chunk = chunk.ljust(4096, b'\x00')  # align by 4kb

            # 3.1 刷入一个dataframe
            #print('[INFO]', 'Write firmware data piece')
            self.dump_to_flash(chunk, address= n * 4096 + address_offset)
            printProgressBar(n+1, total_chunk, prefix = 'Downloading:', suffix = '', length = 50)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--port", help="COM Port", default="DEFAULT")
    parser.add_argument("-b", "--baudrate", type=int, help="UART baudrate for terminal", default=115200)
    parser.add_argument("-B", "--Board", type=str, help="Select dev board, dan or kd233, default dan", default="dan")
    parser.add_argument("-n", "--noansi", help="Do not use ANSI colors, recommended in Windows CMD", default=False, action="store_true")
    parser.add_argument("firmware", help="firmware bin path")

    args = parser.parse_args()

    if (args.noansi == True):
        BASH_TIPS = dict(NORMAL='',BOLD='',DIM='',UNDERLINE='',
                            DEFAULT='', RED='', YELLOW='', GREEN='',
                            BG_DEFAULT='', BG_WHITE='')
        ERROR_MSG   = BASH_TIPS['RED']+BASH_TIPS['BOLD']+'[ERROR]'+BASH_TIPS['NORMAL']
        WARN_MSG    = BASH_TIPS['YELLOW']+BASH_TIPS['BOLD']+'[WARN]'+BASH_TIPS['NORMAL']
        INFO_MSG    = BASH_TIPS['GREEN']+BASH_TIPS['BOLD']+'[INFO]'+BASH_TIPS['NORMAL']
        print(INFO_MSG,'ANSI colors not used',BASH_TIPS['DEFAULT'])
                            
    if args.port == "DEFAULT":
        try:
            list_port_info = next(serial.tools.list_ports.grep(VID_LIST_FOR_AUTO_LOOKUP)) #Take the first one within the list
            print(INFO_MSG,"COM Port Auto Detected, Selected ",list_port_info.device,BASH_TIPS['DEFAULT'])
            _port = list_port_info.device
        except StopIteration:
            print(ERROR_MSG,"No vaild COM Port found in Auto Detect, Check Your Connection or Specify One by"+BASH_TIPS['GREEN']+'`--port/-p`',BASH_TIPS['DEFAULT'])
            sys.exit(1)
    else:
        _port = args.port
        print(INFO_MSG,"COM Port Selected Manually: ",_port,BASH_TIPS['DEFAULT'])

    loader = MAIXLoader(port=_port, baudrate=115200)


    # 1. Greeting.
    print(INFO_MSG,"Trying to Enter the ISP Mode...",BASH_TIPS['DEFAULT'])

    retry_count = 0

    while 1:
        retry_count = retry_count + 1
        if retry_count > 15:
            print("\n" + ERROR_MSG,"No vaild Kendryte K210 found in Auto Detect, Check Your Connection or Specify One by"+BASH_TIPS['GREEN']+'`-p '+('/dev/ttyUSB0', 'COM3')[sys.platform == 'win32']+'`',BASH_TIPS['DEFAULT'])
            sys.exit(1)
        if args.Board == "dan":
            try:
                print('.', end='')
                loader.reset_to_isp_dan()
                loader.greeting()
                break
            except TimeoutError:
                pass
        elif args.Board == "kd233":
            try:
                print('_', end='')
                loader.reset_to_isp_kd233()
                loader.greeting()
                break
            except TimeoutError:
                pass
        else:
            print(ERROR_MSG,"Board unknown!!")
            sys.exit(1)
    timeout = 3
    print()
    print(INFO_MSG,"Greeting Message Detected, Start Downloading firmware to RAM\r\n",BASH_TIPS['DEFAULT'])
    # 2. flash only firmware

    # install firmware at 0x80000000
    loader.install_flash_bootloader(open(args.firmware, 'rb').read())

    loader.boot()

    print(INFO_MSG,"Start miniterm\r\n", BASH_TIPS['DEFAULT'])

    import serial.tools.miniterm
    # For using the terminal with MaixPy the 'filter' option must be set to 'direct'
    # because some control characters are emited
    sys.argv = ['kflash.py', _port, str(args.baudrate), '--filter=direct']
    serial.tools.miniterm.main(default_port=_port, default_baudrate=args.baudrate)

    sys.exit(0)
