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
import shutil
from threading import Thread
from datetime import datetime
try:
    import serial
except ImportError:
    print("PySerial must be installed, run `pip3 install pyserial`\r\n")
    sys.exit(1)

KEY_NONE      = 0x00
KEY_LEFT      = 0x1f
KEY_RIGHT     = 0x1e
KEY_HOME      = 0x10
KEY_END       = 0x03
KEY_QUIT      = 0x11
KEY_ENTER     = 0x0a
KEY_BACKSPACE = 0x08
KEY_DELETE    = 0x7f
KEY_TAB       = 0x09
KEY_DUP       = 0x04

#============
class PyTerm:

    KEYMAP = { ## Gets lengthy
    "\x1b[D" : KEY_LEFT,
    "\x1b[C" : KEY_RIGHT,
    "\x1b[H" : KEY_HOME, ## in Linux Terminal
    "\x1bOH" : KEY_HOME, ## Picocom, Minicom
    "\x1b[1~": KEY_HOME, ## Putty
    "\x1b[F" : KEY_END,  ## Linux Terminal
    "\x1bOF" : KEY_END,  ## Picocom, Minicom
    "\x1b[4~": KEY_END,  ## Putty
    "\x03"   : KEY_DUP, ## Ctrl-C
    "\r"     : KEY_ENTER,
    "\x7f"   : KEY_BACKSPACE, ## Ctrl-? (127)
    "\x1b[3~": KEY_DELETE,
    "\x11"   : KEY_QUIT, ## Ctrl-Q
    "\x1bq"  : KEY_QUIT, ## Alt-Q
    "\n"     : KEY_ENTER,
    "\x04"   : KEY_DUP, ## Ctrl-D
    "\x09"   : KEY_TAB,
    }

    #----------------------------------------------------------------------------
    def __init__(self, baudrate=115200, device='/dev/ttyUSB0', rst=0, clr=False):
        self.DEVICE     = device
        self.BAUDRATE   = baudrate
        self.ESCAPECHAR = "\033"
        self.VERSION = "5.1.3"
        self.ShutdownReceiver = False
        self.ReceiverToStdout = True
        self.DefaultTimeout = 0.1
        self.width, self.height = shutil.get_terminal_size()
        self.colors = clr;
        if clr is True:
            self.TCLR = dict(
                NORMAL = '\033[0m',
                RED    = '\033[1;31m',
                BLUE   = '\033[1;34m',
                YELLOW = '\033[1;33m',
                WHITE  = '\033[1;37m'
                )
        else:
            self.TCLR = dict(
                NORMAL = '',
                RED    = '',
                BLUE   = '',
                YELLOW = '',
                WHITE  = ''
                )

        print("\n"+self.TCLR['RED']+"--[ "+self.TCLR['BLUE']+"MicroPython terminal "+self.TCLR['RED']+" ver. "+self.TCLR['BLUE']+self.VERSION + self.TCLR['RED']+" ]-- "+self.TCLR['NORMAL'])
        print(self.TCLR['RED']+"--[ "+self.TCLR['BLUE']+"Press ESC twice for command mode"+self.TCLR['RED']+" ]-- "+self.TCLR['NORMAL']+"\n")
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
            else:
                self.uart.write(b'\r\n')

        except Exception as e:
            raise Exception(self.TCLR['RED']+"Accessing "+self.TCLR['WHITE'] + self.DEVICE + " "+self.TCLR['RED']+"failed\r\n"+self.TCLR['WHITE']+"PyTerm exit"+self.TCLR['NORMAL']+"\r\n")

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
            print("\r\n"+self.TCLR['RED']+"Error: failed with the following exception:"+self.TCLR['NORMAL']+"\r\n")
            print(e, "\r\n")

        # Shutdown receiver thread
        self.ShutdownReceiver = True
        if self.ReceiverThread.isAlive():
            self.ReceiverThread.join()

        # Clean up everything
        termios.tcsetattr(self.stdinfd, termios.TCSADRAIN, self.oldstdinsettings)
        self.uart.close()

    #----------------------
    def clear_to_eol(self):
        sys.stdout.write("\x1b[0K")
        sys.stdout.flush()

    #-------------------
    def get_input(self):  ## read from interface/keyboard one byte each and match against function keys
        while True:
            in_buffer = sys.stdin.read(1)
            if in_buffer == '\x1b': ## starting with ESC, must be fct
                while True:
                    in_buffer += sys.stdin.read(1)
                    c = in_buffer[-1]
                    if c == '~' or (c.isalpha() and c != 'O'):
                        break
            if in_buffer in self.KEYMAP:
                c = self.KEYMAP[in_buffer]
                return c, None
            elif ord(in_buffer[0]) >= 32:
                return KEY_NONE, in_buffer

    # Line editor
    #------------------------------------------------
    def line_edit(self, prompt, prompt_len, default):
        # Write a message and move cursor back
        push_msg = lambda msg: sys.stdout.write(msg + "\b" * len(msg))
        sys.stdout.write(prompt)
        sys.stdout.write(default)
        sys.stdout.flush()
        self.clear_to_eol()
        res = default
        pos = len(res)
        while True:
            key, char = self.get_input()  ## Get Char of Fct.
            if key == KEY_NONE: ## char to be inserted
                if (prompt_len + len(res)) < (self.width - 2):
                    res = res[:pos] + char + res[pos:]
                    sys.stdout.write(res[pos])
                    sys.stdout.flush()
                    pos += len(char)
                    push_msg(res[pos:]) ## update tail
                    sys.stdout.flush()
            elif key in (KEY_ENTER, KEY_TAB): ## Finis
                return res, len(res)
            elif key in (KEY_QUIT, KEY_DUP): ## Abort
                return None, len(res)
            elif key == KEY_LEFT:
                if pos > 0:
                    sys.stdout.write("\b")
                    sys.stdout.flush()
                    pos -= 1
            elif key == KEY_RIGHT:
                if pos < len(res):
                    sys.stdout.write(res[pos])
                    sys.stdout.flush()
                    pos += 1
            elif key == KEY_HOME:
                sys.stdout.write("\b" * pos)
                sys.stdout.flush()
                pos = 0
            elif key == KEY_END:
                sys.stdout.write(res[pos:])
                sys.stdout.flush()
                pos = len(res)
            elif key == KEY_DELETE: ## Delete
                if pos < len(res):
                    res = res[:pos] + res[pos+1:]
                    push_msg(res[pos:] + ' ') ## update tail
                    sys.stdout.flush()
            elif key == KEY_BACKSPACE: ## Backspace
                if pos > 0:
                    res = res[:pos-1] + res[pos:]
                    sys.stdout.write("\b")
                    sys.stdout.flush()
                    pos -= 1
                    push_msg(res[pos:] + ' ') ## update tail
                    sys.stdout.flush()

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

        print("Sending local file "+self.TCLR['BLUE']+src_fname+self.TCLR['NORMAL']+" to "+self.TCLR['BLUE']+dest_fname+self.TCLR['NORMAL']+"\r\n", end="\r\n")

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
            print("OK, took "+self.TCLR['BLUE'] + "%.3f" % (end_time - start_time) + self.TCLR['NORMAL']+" seconds, " + self.TCLR['BLUE'] + "%.3f" % ((filesize / (end_time - start_time)) / 1024) + self.TCLR['NORMAL']+" KB/s", end="\r\n")
        self.ExitRawREPL()

    #------------------------------------------------------
    def ReceiveFileFromDevice(self, src_fname, dest_fname):
        try:
            dst_file = open(dest_fname, 'wb')
            recv_cmd = "os.send_file('{}', 0)\r\n".format(src_fname)
        except:
            print("Error opening file", end="\r\n")
            return

        print("Receiving remote file "+self.TCLR['BLUE']+src_fname+self.TCLR['NORMAL']+" to "+self.TCLR['BLUE']+dest_fname+self.TCLR['NORMAL']+"\r\n", end="\r\n")

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
            print("OK, took "+self.TCLR['BLUE']+"%.3f" % (end_time - start_time) + self.TCLR['NORMAL']+" seconds, "+self.TCLR['BLUE']+"%.3f" % ((filesize / (end_time - start_time)) / 1024)+self.TCLR['NORMAL']+" KB/s", end="\r\n")
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
                    prompt = self.TCLR['RED']+"--["+self.TCLR['BLUE']+"mpTerm command: "+self.TCLR['NORMAL']
                    print("\r\n")
                    command, cmd_len = self.line_edit(prompt, 19, '')

                    if command is None:
                        if self.colors is True:
                            print("\r{}".format(prompt) + self.TCLR['WHITE']+"aborted"+self.TCLR['NORMAL']+"\033[0K", end="\r\n")
                        else:
                            cmd_blank = " "*cmd_len
                            print("\r{}".format(prompt) + self.TCLR['WHITE']+"aborted"+self.TCLR['NORMAL']+cmd_blank, end="\r\n")

                    elif command == "exit":
                        print("\r\n"+self.TCLR['BLUE']+" Exit PyTerm "+self.TCLR['RED']+"]--"+self.TCLR['NORMAL']+"\r\n", end="")
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

                    elif command[0:8] == "lslocal ":
                        try:
                            cmd = re.sub(' +', ' ', command.strip()).split(' ')
                            if (len(cmd) == 2) or (len(cmd) == 3):
                                short_list = False
                                if (len(cmd) == 3):
                                    if cmd[2] == "short":
                                        short_list = True
                                rdir = cmd[1]
                                lpath = os.path.abspath(rdir)
                                dirlst = os.listdir(rdir)
                                    
                                if len(dirlst) > 0:
                                    sys.stdout.write("\r\n\r\nList of directory '{}':\r\n".format(lpath))
                                    sys.stdout.write("{}\r\n".format("".rjust(21+len(lpath), '-')))
                                    if short_list is False:
                                        dirlist = []
                                        for f in dirlst:
                                            file_path = os.path.abspath(lpath + "/" + f)
                                            st = os.stat(file_path)
                                            dirlist.append((f, (st[0] & 0x8000) == 0, st[6], st[8]))

                                        dirlist.sort(key=lambda x: (not x[1], x[0].lower()))

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
                                        dirlst.sort(key=lambda name: name.lower())
                                        for f in dirlst:
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
                        if self.colors is True:
                            print("\r{}".format(prompt)+self.TCLR['RED']+"unknown command !"+self.TCLR['NORMAL']+"\033[0K"+"\r\n", end="\r\n")
                        else:
                            cmd_blank = " "*cmd_len
                            print("\r{}".format(prompt)+self.TCLR['RED']+"unknown command !"+self.TCLR['NORMAL']+cmd_blank+"\r\n", end="\r\n")
                        print(self.TCLR['WHITE']+"Available commands:"+self.TCLR['NORMAL'], end="\r\n")
                        print(self.TCLR['BLUE']  +"        exit              "+self.TCLR['NORMAL']+" - exit the terminal", end="\r\n")
                        print(self.TCLR['BLUE']  +"     version              "+self.TCLR['NORMAL']+" - print version info", end="\r\n")
                        print(self.TCLR['BLUE']  +"    synctime              "+self.TCLR['NORMAL']+" - synchronize device time to the PC time", end="\r\n")
                        print(self.TCLR['BLUE']  +"    baudrate "+self.TCLR['WHITE']+"bdr          "+self.TCLR['NORMAL']+" - set terminal baudrate", end="\r\n")
                        print(self.TCLR['BLUE']  +"set_baudrate "+self.TCLR['WHITE']+"bdr          "+self.TCLR['NORMAL']+" - set device and terminal baudrate", end="\r\n")
                        print(self.TCLR['BLUE']  +"        send "+self.TCLR['WHITE']+"lfile rfile"+self.TCLR['NORMAL']+"   - send file to device", end="\r\n")
                        print(self.TCLR['BLUE']  +"        recv "+self.TCLR['WHITE']+"rfile lfile"+self.TCLR['NORMAL']+"   - receive file from device", end="\r\n")
                        print(self.TCLR['BLUE']  +"     senddir "+self.TCLR['WHITE']+"ldir  rdir "+self.TCLR['NORMAL']+"   - send all files from local directory to device's directory", end="\r\n")
                        print(self.TCLR['BLUE']  +"     recvdir "+self.TCLR['WHITE']+"rdir  ldir "+self.TCLR['NORMAL']+"   - receive all files from device's directory to local directory", end="\r\n")
                        print(self.TCLR['BLUE']  +"          ls "+self.TCLR['WHITE']+"rdir  [short]"+self.TCLR['NORMAL']+" - list remote directory, if 'short' is given, only file names are printed", end="\r\n")
                        print(self.TCLR['BLUE']  +"     lslocal "+self.TCLR['WHITE']+"rdir  [short]"+self.TCLR['NORMAL']+" - list local directory, if 'short' is given, only file names are printed", end="\r\n")
                        print(self.TCLR['YELLOW']+"       Enter              "+self.TCLR['NORMAL']+" - accept and execute command", end="\r\n")
                        print(self.TCLR['YELLOW']+"      Ctrl-Q              "+self.TCLR['NORMAL']+" - aborts command mode\r", end="\r\n")

                    print(self.TCLR['BLUE']+"back to device "+self.TCLR['RED']+"]--"+self.TCLR['NORMAL']+"\r\n", end="")
                    self.uart.write(b'\r\n')
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
    cli.add_argument("-c", "--color",    default=True,           type=str, action="store",
        help="Use ANSI colors or not")
    cli.add_argument("-d", "--device",   default='/dev/ttyUSB0', type=str, action="store",
        help="Path to the serial communication device.")

    args = cli.parse_args()

    trm = PyTerm(baudrate=args.baudrate, device=args.device, rst=args.reset, clr=args.color)
