#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import tty
import termios
import time
import argparse
import binascii
import re
from threading import Thread
from datetime import datetime
try:
    import serial
except ImportError:
    print("\033[1;31mPySerial must be installed, run \033[1;34m`pip3 install pyserial`\033[0m\r\n")
    sys.exit(1)

#============
class PyTerm:

    #-----------------------------------------------------------------
    def __init__(self, baudrate=115200, device='/dev/ttyUSB0', rst=0):
        self.DEVICE     = device
        self.BAUDRATE   = baudrate
        self.ESCAPECHAR = "\033"
        self.VERSION = "5.1.2"
        self.ShutdownReceiver = False
        self.ReceiverToStdout = True
        self.DefaultTimeout = 0.1

        print("\n\033[1;31m--[ \033[1;34mMicroPython terminal \033[1;31m ver. \033[1;34m" + self.VERSION + "\033[1;31m ]-- \033[0m")
        print("\033[1;31m--[ \033[1;34mPress ESC twice for command mode\033[1;31m ]-- \033[0m\n")
        # Open remote terminal device
        try:
            self.uart = serial.Serial(
                port    = self.DEVICE,
                baudrate= self.BAUDRATE,
                bytesize= serial.EIGHTBITS,
                parity  = serial.PARITY_NONE,
                stopbits= serial.STOPBITS_ONE,
                timeout = self.DefaultTimeout,
                xonxoff = 0,
                rtscts  = 0,
                interCharTimeout=None
            )
            if rst:
                self.uart.dtr = False
                time.sleep(0.1)
                self.uart.dtr = True
        except Exception as e:
            raise Exception("\033[1;31mAccessing \033[1;37m" + self.DEVICE + " \033[1;31mfailed\r\n\033[1;37mPyTerm exit\033[0m\r\n")

        # Setup local terminal
        self.stdinfd          = sys.stdin.fileno()
        self.oldstdinsettings = termios.tcgetattr(self.stdinfd)
        tty.setraw(self.stdinfd) # from now on, end-line must be "\r\n"
        
        # Start receiver thread
        self.ReceiverThread = Thread(target=self.ReceiveData, args=(self.uart, False))
        self.ReceiverThread.start()

        # this is the main loop of this software
        try:
            self.HandleUnbufferedUserInput();
        except Exception as e:
            print("\r\n\033[1;31mError: failed with the following exception:\033[0m\r\n")
            print(e, "\r\n")

        # Shutdown receiver thread
        self.ShutdownReceiver = True
        if self.ReceiverThread.isAlive():
            self.ReceiverThread.join()

        # Clean up everything
        termios.tcsetattr(self.stdinfd, termios.TCSADRAIN, self.oldstdinsettings)
        self.uart.close()

    #-----------------------------------------
    def ReceiveData(self, uart, binary=False):
        data = ""
        last_char = '?'
        while not self.ShutdownReceiver:

            if not self.ReceiverToStdout:
                time.sleep(0.01);
                continue

            try:
                data = self.uart.read(self.uart.inWaiting())
            except:
                return

            if data:
                try:
                    string = data.decode("utf-8")
                    ostr = ""
                    for c in string:
                        if ord(c) != 4:
                            if (c == '\n') and (last_char != '\r'):
                                ostr = ostr + '\r'
                            last_char = c
                            ostr = ostr + str(c)
                        else:
                            ostr = ostr + "[{}]".format(hex(ord(c)))
                except UnicodeDecodeError:
                    string = "[" + str(data) + "]"
                    ostr = string

                sys.stdout.write(ostr)
                sys.stdout.flush()
        
            time.sleep(0.01);

    #---------------------
    def crc_16(self, buf):
        crc = 0xFFFF
        for c in buf:
            crc = (crc ^ (c << 8)) & 0xFFFF
            for i in range(8):
                if (crc & 0x8000):
                    crc = ((crc << 1) & 0xFFFF) ^ 0x1021
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    #-----------------------------------------
    def EnterRawREPL(self, imprt, cmd, bdr=0):
        self.ReceiverToStdout = False
        time.sleep(0.1)
        dummy = self.uart.read()
        self.uart.timeout = 4
        # enter raw REPL
        self.uart.write(b'\x01')
        resp = self.uart.read(28)
        if resp == b'\r\nraw REPL; CTRL-B to exit\r\n':
            #print("In Raw REPL", end="\r\n")
            time.sleep(0.1)
            self.uart.write(imprt)
            self.uart.write(cmd)
            self.uart.write(b'\x04') # Execute
            time.sleep(0.1)
            if bdr > 0:
                print("baudrate changed to {}".format(bdr), end="\r\n")
                self.uart.baudrate = bdr
                time.sleep(0.1)

            return True

        print("\r\nerror waiting for Raw REPL", end="\r\n")
        self.uart.timeout = self.DefaultTimeout
        self.ReceiverToStdout = False
        return False

    #---------------------
    def ExitRawREPL(self):
        # exit raw REPL
        self.uart.timeout = self.DefaultTimeout
        tmo = 10
        while True:
            bb = self.uart.read(1)
            if bb == b'\04':
                #print("Confirmation received ({})".format(tmo), end="\r\n")
                pass
            elif bb == b'>':
                #print("MPy prompt received ({})".format(tmo), end="\r\n")
                break
            tmo -= 1
            if tmo == 0:
                print("\r\nExit Raw REPL: timeout", end="\r\n")
                break
        self.uart.write(b'\x02') # Exit RawREPL

        time.sleep(0.1)
        tmo = 0
        bb = self.uart.read(1)
        while len(bb) > 0:
            tmo += 1
            bb = self.uart.read(1)

        self.ReceiverToStdout = True
        #print("Exit Raw REPL ({})".format(tmo), end="\r\n")

    #-------------------------------------------------
    def SendFileToDevice(self, src_fname, dest_fname):
        try:
            filesize = os.path.getsize(src_fname)
            src_file = open(src_fname, 'rb')
            send_cmd = "os.get_file('{}', {})\r\n".format(dest_fname, str(filesize))
        except:
            print("Error opening file", end="\r\n")
            return

        print("Sending local file \033[1;34m{}\033[0m to \033[1;34m{}\033[0m\r\n".format(src_fname, dest_fname), end="\r\n")

        if not self.EnterRawREPL(b'import os\r\n', bytes(send_cmd.encode('utf-8'))):
            return

        ack = b'\x00'
        while ack != b'\x06':
            ack = self.uart.read(1)
            if len(ack) == 0:
                break
        try:
            if len(ack) == 0:
                src_file.close()
                print("timeout waiting for device acknowledge", end="\r\n")
                self.ExitRawREPL()
                return

            start_time = time.time()
            bytes_remaining = filesize
            while bytes_remaining > 0:
                read_size = min(bytes_remaining, 1024)
                buf = src_file.read(read_size)
                crc = self.crc_16(buf)
                bcrc = bytes([crc >> 8, crc & 0xFF])
                buff = b''.join([buf,bcrc])
                bytes_remaining -= read_size

                # Wait for ack from remote
                if ack == b'\x00':
                    ack = self.uart.read(1)
                if ack == b'\x07' or ack == b'\x06':
                    time.sleep(0.01)
                    self.uart.write(buff)
                    if ack == b'\x06':
                        sys.stdout.write("\r--> {0:.2f}%".format((filesize-bytes_remaining) / filesize * 100))
                    else:
                        sys.stdout.write("\r-R> {0:.2f}%".format((filesize-bytes_remaining) / filesize * 100))
                    sys.stdout.flush()
                elif ack == b'\x08' or ack == b'\x09' or ack == b'\x0A':
                    if bytes_remaining > 0:
                        print("\r\nabort requested from remote [{}]".format(ack[0]), end="\r\n")
                        break
                else:
                    if bytes_remaining > 0:
                        print("\r\ntimed out or error in sending file to remote [{}]".format(ack), end="\r\n")
                        break
                ack = b'\x00'
        except Exception as e:
            print("\r\nexception while sending file to remote ({})".format(e), end="\r\n")

        src_file.close()
        print("", end="\r\n")
        if bytes_remaining <= 0:
            end_time = time.time()
            print("OK, took \033[1;34m%.3f\033[0m seconds, \033[1;34m%.3f\033[0m KB/s" % (end_time - start_time, (filesize / (end_time - start_time)) / 1024), end="\r\n")
        self.ExitRawREPL()

    #------------------------------------------------------
    def ReceiveFileFromDevice(self, src_fname, dest_fname):
        try:
            dst_file = open(dest_fname, 'wb')
            recv_cmd = "os.send_file('{}', 0)\r\n".format(src_fname)
        except:
            print("Error opening file", end="\r\n")
            return

        print("Receiving remote file \033[1;34m{}\033[0m to \033[1;34m{}\033[0m\r\n".format(src_fname, dest_fname), end="\r\n")

        if not self.EnterRawREPL(b'import os\r\n', bytes(recv_cmd.encode('utf-8'))):
            return

        ack = b'\x00'
        while ack != b'\x06':
            ack = self.uart.read(1)
            if len(ack) == 0:
                break
        if len(ack) == 0:
            print("timeout waiting for file", end="\r\n")
            dst_file.close()
            self.ExitRawREPL()
            return

        # receive filesize first
        fsblock_ok = False
        fs_bufr = self.uart.read(7)
        if len(fs_bufr) == 7:
            if fs_bufr[0] == 0x5B:
                fs_buf = b'\x06' + fs_bufr
                # check crc (last 2 bytes received)
                crc = self.crc_16(fs_buf[0:6])
                rcrc = (fs_buf[6] << 8) + fs_buf[7]
                if crc == rcrc:
                    fsblock_ok = True
        if fsblock_ok == False:
            self.uart.write(b'\x08')   # ASCII ACK is 0x08, abort
            print("Error receiving file size", end="\r\n")
            dst_file.close()
            self.ExitRawREPL()
            return

        filesize = (fs_buf[5] << 24) + (fs_buf[4] << 16) + (fs_buf[3] << 8) + fs_buf[2]
        self.uart.write(b'\x06')   # ASCII ACK is 0x06
        try:
            start_time = time.time()
            bytes_remaining = filesize
            ntry = 3
            while bytes_remaining > 0:
                read_size = min(bytes_remaining, 1024)
                while ntry > 0:
                    read_buf = self.uart.read(read_size+2)
                    if len(read_buf) != read_size+2:
                        print("\r\nwrong block size received: {}, expected {} [{}]".format(len(read_buf), read_size+2, read_buf), end="\r\n")
                        ntry = 0
                        continue
                    # check for abort block (all bytes 0x5A)
                    cc = True
                    for idx in range(read_size+2):
                        if read_buf[idx] != 0x5A:
                            cc = False
                            break
                    if cc:
                        #abort block received
                        print("\r\nabort requested from remote", end="\r\n")
                        ntry = 0
                        continue
                    # check crc (last 2 bytes received)
                    bcrc = read_buf[-2:]
                    rcrc = (bcrc[0] << 8) + bcrc[1]
                    crc = self.crc_16(read_buf[0:-2])
                    if crc == rcrc:
                        dst_file.write(read_buf[0:-2])
                        # Send an ack to the remote as a form of flow control
                        sys.stdout.write("\r<<< {0:.2f}%".format((filesize-bytes_remaining) / filesize * 100))
                        sys.stdout.flush()
                        self.uart.write(b'\x06')   # ASCII ACK is 0x06
                        break
                    else:
                        sys.stdout.write("\r<R< {0:.2f}%".format((filesize-bytes_remaining) / filesize * 100))
                        sys.stdout.flush()
                        self.uart.write(b'\x07')   # ASCII ACK is 0x07, repeat
                        ntry -= 1
                if ntry == 0:
                    print("\r\ntimed out or error in receiving file from remote", end="\r\n")
                    self.uart.write(b'\x08')   # ASCII ACK is 0x08, abort
                    bytes_remaining = 0
                    continue
                bytes_remaining -= read_size
            if ntry > 0:
                sys.stdout.write("\r<<< {0:.2f}%\n".format((filesize-bytes_remaining) / filesize * 100))
        except Exception as e:
            print("\r\nexception while receiving file from remote ({})".format(e), end="\r\n")

        print("", end="\r\n")
        if bytes_remaining <= 0:
            end_time = time.time()
            print("OK, took \033[1;34m%.3f\033[0m seconds, \033[1;34m%.3f\033[0m KB/s" % (end_time - start_time, (filesize / (end_time - start_time)) / 1024), end="\r\n")
        dst_file.close()
        self.ExitRawREPL()

    #------------------
    def SyncTime(self):
        now = time.localtime(time.time())
        tz = int(time.strftime("%z", time.localtime())) // 100
        tt = str((now.tm_year, now.tm_mon, now.tm_mday, now.tm_hour, now.tm_min, now.tm_sec, 0, 0))
        cmd = "_ = time.mktime({}, tz={}, setrtc=True)\r\n".format(tt, tz)

        if not self.EnterRawREPL(b'import time\r\n', bytes(cmd.encode('utf-8'))):
            return

        self.ExitRawREPL()

    #-----------------------------
    def SetBaudrate(self, bdrate):
        cmd = "print(machine.repl_baudrate({}))\r\n".format(bdrate)

        if not self.EnterRawREPL(b'import machine\r\n', bytes(cmd.encode('utf-8')), bdr=bdrate):
            return

        time.sleep(0.5)
        self.ExitRawREPL()

    #--------------------------------------------------
    def ReadDirFromRemote(self, remote_dir, short=True):
        cmd = "try:\r\n    print(os.listdirex('{}', {}))\r\nexcept:\r\n    print('[]')\r\n".format(remote_dir, str(short))

        if not self.EnterRawREPL(b'import os\r\n', bytes(cmd.encode('utf-8'))):
            return

        response = ""
        rdb = b'\x00'
        while rdb != b'\x04':
            rdb = self.uart.read(1)
            if len(rdb) == 0:
                break
            if rdb == b'\x04':
                break
            response = response + chr(rdb[0])

        self.ExitRawREPL()

        _locals = locals()
        dirlist = []
        if response[:3] == ">OK":
            response = response[3:]
            if len(response) >= 2:
                response = response.strip('\r\n')
                response = "dirlist = " + response
                try:
                    exec(response, globals(), _locals)
                    dirlist = _locals['dirlist']
                except:
                    pass
        return dirlist

    #---------------------
    def ReadCommand(self):
        char    = ""
        command = ""

        while True:
            char = sys.stdin.read(1)
            if char == "\r":
                break
            elif char == self.ESCAPECHAR:
                if len(command) == 0:
                    command = self.ESCAPECHAR
                break
            else:
                sys.stdout.write(char)
                sys.stdout.flush()
                command += char

        return command

    #----------------------
    def Get2ndEscape(self):
            char = sys.stdin.read(1)
            if char == self.ESCAPECHAR:
                return True
            elif char == "[":
                self.uart.write("\033[".encode("utf-8"))
                return False
            data = char.encode("utf-8")
            self.uart.write(data)
            return False

    #-----------------------------------
    def HandleUnbufferedUserInput(self):
        char = ""

        while True:
            char = sys.stdin.read(1)

            if char == self.ESCAPECHAR:
                if self.Get2ndEscape():
                    print("\r\n\033[1;31m--[\033[1;34mmpTerm command: \033[0m", end="")
                    command = self.ReadCommand()

                    if command == self.ESCAPECHAR:
                        sys.stdout.write("\r\n")
                        sys.stdout.flush()
                        self.uart.write(self.ESCAPECHAR.encode("utf-8"))

                    if command == "exit":
                        print("\r\n\033[1;34m Exit PyTerm \033[1;31m]--\033[0m\r\n", end="")
                        break

                    elif command == "version":
                        sys.stdout.write("\r\n")
                        sys.stdout.flush()
                        print("Version: " + self.VERSION, end="\r\n")

                    elif command[0:5] == "send ":
                        try:
                            cmd = re.sub(' +', ' ', command.strip()).split(' ')
                            if len(cmd) == 3:
                                sys.stdout.write("\r\n")
                                self.SendFileToDevice(cmd[1], cmd[2])
                            elif len(cmd) == 2:
                                # send to '/flash', same name as source
                                fname = cmd[1].split('/')
                                if len(fname) > 0:
                                    sys.stdout.write("\r\n")
                                    self.SendFileToDevice(cmd[1], "/flash/" + fname[len(fname)-1])
                                else:
                                    print("\r\nWrong command arguments", end="\r\n")
                            else:
                                print("\r\nWrong command arguments", end="\r\n")
                        except:
                            print("\r\nError", end="\r\n")

                    elif command[0:8] == "senddir ":
                        try:
                            cmd = re.sub(' +', ' ', command.strip()).split(' ')
                            if len(cmd) == 3:
                                if os.path.isdir(cmd[1]):
                                    ldir = cmd[1].rstrip('/') + '/'
                                    rdir = cmd[2].rstrip('/') + '/'
                                    sys.stdout.write("\r\n")
                                    for f in os.listdir(cmd[1]):
                                        if os.path.isfile(ldir + f):
                                            self.SendFileToDevice(ldir + f, rdir + f)
                                            time.sleep(0.5)
                                else:
                                    print("\r\n{} is not a directory".format(cmd[1]), end="\r\n")
                            else:
                                print("\r\nWrong command arguments", end="\r\n")
                        except:
                            print("\r\nError", end="\r\n")

                    elif command[0:5] == "recv ":
                        try:
                            cmd = re.sub(' +', ' ', command.strip()).split(' ')
                            if len(cmd) == 3:
                                if os.path.isdir(cmd[2]):
                                    dirname = cmd[2].rstrip('/') + '/'
                                    fname = cmd[1].split('/')
                                    if len(fname) > 0:
                                        fname = dirname + fname[len(fname)-1]
                                    else:
                                        print("\r\nWrong command arguments", end="\r\n")
                                else:
                                    fname = cmd[2]
                                sys.stdout.write("\r\n")
                                self.ReceiveFileFromDevice(cmd[1], fname)
                            elif len(cmd) == 2:
                                # receive to current directory, same name as source
                                fname = cmd[1].split('/')
                                if len(fname) > 0:
                                    sys.stdout.write("\r\n")
                                    self.ReceiveFileFromDevice(cmd[1], fname[len(fname)-1])
                                else:
                                    print("\r\nWrong command arguments", end="\r\n")
                            else:
                                print("\r\nWrong command arguments", end="\r\n")
                        except Exception as e:
                            print("\r\nError", e, end="\r\n")

                    elif command[0:8] == "recvdir ":
                        try:
                            cmd = re.sub(' +', ' ', command.strip()).split(' ')
                            if len(cmd) == 3:
                                if not os.path.isdir(cmd[2]):
                                    os.mkdir(cmd[2])
                                dirlist = self.ReadDirFromRemote(cmd[1])
                                if len(dirlist) > 0:
                                    rdir = cmd[1].rstrip('/') + '/'
                                    ldir = cmd[2].rstrip('/') + '/'
                                    sys.stdout.write("\r\n")
                                    for f in dirlist:
                                        self.ReceiveFileFromDevice(rdir + f, ldir + f)
                                        time.sleep(0.5)
                                else:
                                    print("\r\nNo files to receive\r\n{}\r\n".format(dirlist))
                            else:
                                print("\r\nWrong command arguments", end="\r\n")
                        except Exception as e:
                            print("\r\nError", e, end="\r\n")

                    elif command[0:3] == "ls ":
                        try:
                            cmd = re.sub(' +', ' ', command.strip()).split(' ')
                            if (len(cmd) == 2) or (len(cmd) == 3):
                                short_list = False
                                if (len(cmd) == 3):
                                    if cmd[2] == "short":
                                        short_list = True
                                rdir = cmd[1].rstrip('/') + '/'
                                dirlist = self.ReadDirFromRemote(rdir, short=short_list)
                                if len(dirlist) > 0:
                                    dirlist.sort(key=lambda x: (not x[1], x[0].lower()))
                                    sys.stdout.write("\r\n\r\nList of directory '{}':\r\n".format(rdir))
                                    sys.stdout.write("{}\r\n".format("".rjust(21+len(rdir), '-')))
                                    if short_list is False:
                                        max_name_len = 0
                                        max_size_len = 0
                                        for f in dirlist:
                                            if len(f[0]) > max_name_len:
                                                max_name_len = len(f[0])
                                            if len(str(f[2])) > max_size_len:
                                                max_size_len = len(str(f[2]))
                                        max_name_len += 1
                                        max_size_len += 1
                                        for f in dirlist:
                                            print("{}  {} {}  {}".format(f[0].rjust(max_name_len), " <dir>" if f[1] else "<file>", str(f[2]).rjust(max_size_len), datetime.utcfromtimestamp(f[3]).strftime('%Y-%m-%d %H:%M:%S')), end="\r\n")
                                    else:
                                        dirlist.sort(key=lambda name: name.lower())
                                        for f in dirlist:
                                            print("{}".format(f), end="\r\n")
                                else:
                                    print("\r\nNo files to list\r\n{}\r\n".format(dirlist))
                            else:
                                print("\r\nWrong command arguments", end="\r\n")
                        except Exception as e:
                            print("\r\nError", e, end="\r\n")

                    elif command[0:9] == "baudrate ":
                        try:
                            cmd = re.sub(' +', ' ', command.strip()).split(' ')
                            if len(cmd) == 2:
                                baudrate = int(cmd[1])
                                print("\r\nbautrate set to {}\r\n".format(baudrate), end="\r\n")
                                self.uart.baudrate = baudrate
                            else:
                                print("\r\nWrong command arguments", end="\r\n")
                        except:
                            print("\r\nError", end="\r\n")

                    elif command[0:13] == "set_baudrate ":
                        try:
                            cmd = re.sub(' +', ' ', command.strip()).split(' ')
                            if len(cmd) == 2:
                                baudrate = int(cmd[1])
                                print("\r\nset device and terminal bautrate to {}\r\n".format(baudrate), end="\r\n")
                                self.SetBaudrate(baudrate)
                            else:
                                print("\r\nWrong command arguments", end="\r\n")
                        except Exception as e:
                            print("\r\nError", e, end="\r\n")

                    elif command == "synctime":
                        try:
                            sys.stdout.write("\r\n")
                            self.SyncTime()
                            print("OK.", end="\r\n")
                        except:
                            print("Error", end="\r\n")

                    else:
                        print(""" \033[1;37munknown command\033[0m, use one of the following commands:\r
\033[1;34m        exit                \033[0m - exit the terminal\r
\033[1;34m     version                \033[0m - print version info\r
\033[1;34m    synctime                \033[0m - synchronize device time to the PC time\r
\033[1;34m    baudrate <bdr>          \033[0m - set terminal baudrate\r
\033[1;34mset_baudrate <bdr>          \033[0m - set device and terminal baudrate\r
\033[1;34m        send <lfile> <rfile>\033[0m - send file to device\r
\033[1;34m        recv <rfile> <lfile>\033[0m - receive file from device\r
\033[1;34m     senddir <ldir>  <rdir> \033[0m - send all files from local directory to device's directory\r
\033[1;34m     recvdir <rdir>  <ldir> \033[0m - receive all files from device's directory to local directory\r
\033[1;34m          ls <rdir>  [short]\033[0m - list remote directory, if 'short' is given, only file names are printed\r
""")

                    print("\033[1;34mback to device \033[1;31m]--\033[0m\r\n", end="")
            else:
                data = char.encode("utf-8")
                self.uart.write(data)

#=========================
if __name__ == '__main__':
    cli = argparse.ArgumentParser(
    description="Serial ternimal optimized for K210 MicroPython.",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    cli.add_argument("-b", "--baudrate", default=115200,         type=int, action="store",
        help="The baudrate used for the communication.")
    cli.add_argument("-r", "--reset",    default=False,          type=str, action="store",
        help="Reset the device on start")
    cli.add_argument("-d", "--device",   default='/dev/ttyUSB0', type=str, action="store",
        help="Path to the serial communication device.")

    args = cli.parse_args()

    trm = PyTerm(baudrate=args.baudrate, device=args.device, rst=args.reset)
