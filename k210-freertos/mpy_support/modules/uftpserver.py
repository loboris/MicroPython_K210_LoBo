'''
 * This file is part of the MicroPython K210 project, https://github.com/loboris/MicroPython_K210_LoBo
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 LoBo (https://github.com/loboris)
 *

 * inspired by https://github.com/cpopp/MicroFTPServer
'''

import network, socket, uos, utime
import sys, gc, _thread, uerrno

'''
Features:
=========
- active and passive FTP modes are supported

Supported FTP commands:
-----------------------
USR, PASS, SYST, NOOP, FEAT, AUTH,
PWD, CWD, CDUP, TYPE, SIZE, MDTM, QUIT,
PORT, PASV, LIST, RETR, STOR, APPE, REST,
DELE, RMD, MKD, RNFR, RNTO, STAT

The response to all other commands is "502 Command not implemented.\r\n"


Usage:
======

The server is started by 'Start' method which accepts some optional parameters:

--------------------------------------------------------------------------------------------------------
ftpserv.Start(timeout = 300, threaded = False, user = None, passwd = None, passive = True, debug = None)
--------------------------------------------------------------------------------------------------------
* timeout     timeout in seconds for ftp client socket
* threaded    run the server in thread (recommended)
* user        user name needed to connect to this server, if None, user name is not requested
* passwd      password needed to connect to this server, if None, password is not requested
* passive     enable passive mode, if False, only active FTP mode is possible
              If passive mode is used, two WiFi server sockets are needed
* debug       if True, some debug information will be printed during run

---------------------------------------------------------------
If running ftpserver in main thread, it will block until
timeout expires, remote client disconnets or Ctrl-C is pressed
---------------------------------------------------------------

from uftpserver import MicroFtpSrv

ftpserv = MicroFtpSrv()

ftpserv.Start(passive = False)
ftpserv.Start(user="micro", passwd="python", passive = False)
ftpserv.Start(user="micro", passwd="python")

------------------------
Run ftpserver in thread:
------------------------
from uftpserver import MicroFtpSrv

ftpserv = MicroFtpSrv()

ftpserv.Start(user="micro", passwd="python", threaded=True)

To stop it, execute:

ftpserv.Stop()

'''

class MicroFtpSrv:

    #-----------------------------------
    @staticmethod
    def get_absolute_path(cwd, payload):
        # Just a few special cases "..", "." and ""
        # If payload start's with /, set cwd to /
        # and consider the remainder a relative path
        if payload.startswith('/'):
            cwd = "/"
        for token in payload.split("/"):
            if token == '..':
                if cwd != '/':
                    cwd = '/'.join(cwd.split('/')[:-1])
                    if cwd == '':
                        cwd = '/'
            elif token != '.' and token != '':
                if cwd == '/':
                    cwd += token
                else:
                    cwd = cwd + '/' + token
        return cwd
    
    # compare fname against pattern. Pattern may contain
    # wildcards ? and *.
    #-------------------------
    @staticmethod
    def fncmp(fname, pattern):
        pi = 0
        si = 0
        while pi < len(pattern) and si < len(fname):
            if (fname[si] == pattern[pi]) or (pattern[pi] == '?'):
                si += 1
                pi += 1
            else:
                if pattern[pi] == '*': # recurse
                    if (pi + 1) == len(pattern):
                        return True
                    while si < len(fname):
                        if MicroFtpSrv.fncmp(fname[si:], pattern[pi+1:]) == True:
                            return True
                        else:
                            si += 1
                    return False
                else:
                    return False
        if pi == len(pattern.rstrip("*"))  and si == len(fname):
            return True
        else:
            return False

    #-----------------------
    @staticmethod
    def check_notify_quit():
        notif = _thread.getnotification()
        if notif == _thread.EXIT:
            print ("[MicroFtpSrv] Received QUIT notification, exiting")
            return True
        elif notif != 0:
            print("[MicroFtpSrv] Notification {} unknown".format(notif))
        return False
    
    #---------------------------------------
    def make_description(path, fname, full):
        if full:
            if (path == '/') and ((fname == 'flash') or (fname == 'sd')):
                file_permissions = "drwxrwxrwx"
                file_size = 0
                file_time = utime.strftime("%b %d %H:%M")
            else:
                stat = uos.stat(MicroFtpSrv.get_absolute_path(path, fname))
                file_size = stat[6]
                ftime = utime.localtime(stat[7])
                if (stat[0] & 0o170000 == 0o040000):
                    # directory
                    file_permissions = "drwxrwxrwx"
                else:
                    # file
                    file_permissions = "-rw-rw-rw-"
                if utime.localtime()[0] == ftime[0]:
                    file_time = utime.strftime("%b %d %H:%M", ftime)
                else:
                    file_time = utime.strftime("%b %d %Y", ftime)
            description = "{}    1 owner group {:>10} {} {}\r\n".format(
                    file_permissions, file_size, file_time, fname)
        else:
            description = fname + "\r\n"
        return description
    
    # ===( Constructor )==================
    def __init__( self ) :
        # config & status variables
        self.thID               = 0
        self.debug              = False
        self._started           = False
        self._state             = "Stopped"
        self.threaded           = False
        self.passive            = True
        self.local_IP           = ""
        # socket variables
        self.ftpsocket          = None
        self.datasocket         = None
        self.dataclient         = None
        self._passive_port      = 1050
        self._sock_timeout      = 300
        # Ftp variables
        self.user               = None
        self.passwd             = None
        self.path               = "/"
        self.cmd_client         = None
        self.remote_addr        = None
        self.fromname           = None
        self.do_run             = True
        self.active_mode        = False
        self.last_transfer_path = None
        self.last_transfer_save = False
        self.data_request       = False

    #----------------------
    def msg_550_fail(self):
        self.cmd_client.write("550 Failed\r\n")

    #--------------------
    def msg_250_OK(self):
        self.cmd_client.write("250 OK\r\n")

    #------------------------------------------------
    def send_list_data(self, path, dataclient, full):
        lst = bytearray()
        try: # whether path is a directory name
            for fname in sorted(uos.listdir(path), key = str.lower):
                lst.extend(MicroFtpSrv.make_description(path, fname, full))
        except: # path may be a file name or pattern
            pattern = path.split("/")[-1]
            path = path[:-(len(pattern) + 1)]
            if path == "":
                path = "/"
            for fname in sorted(uos.listdir(path), key = str.lower):
                if MicroFtpSrv.fncmp(fname, pattern) == True:
                    lst.extend(MicroFtpSrv.make_description(path, fname, full))
        if (len(lst) > 0):
            dataclient.write(lst)
    
    #-----------------------------
    def send_file_data(self, pos):
        try:
            file = open(self.path, "rb")
            if pos > 0:
                file.seek(pos)
        except:
            return False
        try:
            chunk = file.read(2048)
            while len(chunk) > 0:
                _ = self.dataclient.write(chunk)
                chunk = file.read(2048)
                utime.sleep_ms(100)
            file.close()
            return True
        except Exception as err:
            file.close()
            if self.debug:
                sys.print_exception(err)
            return False
    
    #-----------------------------
    def save_file_data(self, pos):
        try:
            if pos > 0:
                file = open(self.path, "rwb")
                file.seek(pos)
            elif pos < 0:
                file = open(self.path, "ab")
            else:
                file = open(self.path, "wb")
        except:
            if self.debug:
                print ("Error opening file.")
            return False
    
        rec_cnt = 0;
        try:
            while True:
                chunk = self.dataclient.read(1024)
                if len(chunk) > 0:
                    file.write(chunk)
                    rec_cnt += len(chunk)
                else:
                    if self.dataclient.peer_closed():
                        if self.debug:
                            print ("OK finished, received {}.".format(rec_cnt))
                        break
                    else:
                        utime.sleep_ms(50)
    
            file.close()
            return True
        except Exception as err:
            file.close()
            if self.debug:
                sys.print_exception(err)
            if err.args[0] == uerrno.ETIMEDOUT:
                if self.debug:
                    print ("OK finished (timeout), received {}.".format(rec_cnt))
                return True

            if self.debug:
                print ("Error receiving file ({})".format(err))
            return False

    #---------------------------------
    def processCommand(self, command):    

        if command == "USER":
            if self.user is None and self.passwd == None:
                self.cmd_client.write("230 Logged in.\r\n")
            else:
                if self.payload == self.user:
                    self.cmd_client.write("331 User name okay, need password.\r\n")
                else:
                    self.msg_550_fail()
                
        elif command == "PASS":
            if self.user is None and self.passwd == None:
                self.cmd_client.write("230 Logged in.\r\n")
            else:
                if self.payload == self.passwd:
                    self.cmd_client.write("230 Logged in.\r\n")
                else:
                    self.msg_550_fail()

        elif command == "SYST":
            self.cmd_client.write("215 UNIX Type: L8\r\n")

        elif (command == "NOOP") or (command == "CLNT") or (command == "OPTS"):
            self.cmd_client.write("200 OK\r\n")

        elif command == "FEAT":
            self.cmd_client.write("502 no-features\r\n")

        elif command == "AUTH":
            self.cmd_client.write("504 not-supported\r\n")

        elif command == "PWD" or command == "XPWD":
            if self.debug:
                print("       CWD: [{}]".format(self.cwd))
            self.cmd_client.write('257 "{}"\r\n'.format(self.cwd))

        elif command == "CWD":
            try:
                files = uos.listdir(self.path) # will raise an error if wrong path
                self.cwd = self.path
                if self.debug:
                    print("       CWD: [{}]".format(self.cwd))
                self.msg_250_OK()
            except:
                self.cmd_client.write(self.msg_550_fail)

        elif command == "CDUP":
            self.cwd = MicroFtpSrv.get_absolute_path(self.cwd, "..")
            if self.debug:
                print("       CWD: [{}]".format(self.cwd))
            self.msg_250_OK()

        elif command == "TYPE":
            # probably should switch between binary and not
            self.cmd_client.write('200 Transfer mode set\r\n')

        elif command == "SIZE":
            try:
                size = uos.stat(self.path)[6]
                self.cmd_client.write('213 {}\r\n'.format(size))
            except:
                self.cmd_client.write(self.msg_550_fail)

        elif command == "MDTM":
            try:
                time = uos.stat(self.path)[7]
                self.cmd_client.write('213 {}\r\n'.format(time))
            except:
                self.cmd_client.write(self.msg_550_fail)

        elif command == "QUIT":
            self.cmd_client.write('221 Bye.\r\n')
            if not self.threaded:
                self.do_run = False;

        # ---- Commands using data channel ---------------------------------------------------
        elif command == "PORT":
            # ==== The client requests data in ACTIVE mode ====
            raddrs = self.payload.split(",")
            raddr = raddrs[0] + '.' + raddrs[1] + '.'+raddrs[2] + '.' + raddrs[3]
            rport = int(raddrs[4]) * 256 + int(raddrs[5])
            active_addr = (raddr, rport)
            # Create client socket to connect to the FTP client's data port
            self.dataclient = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP)
            self.dataclient.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.dataclient.settimeout(1)
            if self.debug:
                print("Active mode requested, connect to {}:{}".format(raddr, rport))

            self.active_mode = True
            self.cmd_client.write('200 PORT command  successful\r\n')
            # Connect to the client IP address and port FROM local PORT 20
            utime.sleep_ms(100)
            try:
                self.dataclient.connect(active_addr, 20)
                self.data_request = True
            except:
                if self.debug:
                    print ("No active data connection")
                self.dataclient = None
                self.msg_550_fail()

        elif command == "PASV":
            # ==== The client requests data in PASSIVE mode ====
            if self.passive is True:
                self.active_mode = False
                if self.debug:
                    print ("Passive mode requested")
                result = '227 Entering Passive Mode ({},{},{}).\r\n'.format(self.local_IP.replace('.',','), self._passive_port>>8, self._passive_port%256)
                self.cmd_client.write(result)
                if self.debug:
                    print ("Waiting for data connection")

                # === Accept the data connection from client ===
                self.dataclient, data_addr = self.datasocket.accepted()

                if self.dataclient is None:
                    if self.debug:
                        print ("No passive data connection")
                    self.msg_550_fail()
                else:
                    if self.debug:
                        print("Data connection from:", data_addr)
                    self.dataclient.settimeout(1)
                    self.data_request = True
            else:
                self.msg_550_fail()
            utime.sleep_ms(50)

        elif command == "LIST" or command == "NLST":
            if not self.payload.startswith("-"):
                place = self.path
            else:
                place = self.cwd
            if self.debug:
                print ("LIST:", place)

            if self.dataclient is None:
                self.msg_550_fail()
            else:
                try:
                    # Some clients requires the following confirmation, some don't
                    self.cmd_client.write("150 OK Directory listing follows.\r\n")
                    # send the listing
                    self.send_list_data(place, self.dataclient, ((command == "LIST") or (self.payload == "-l")))
                    self.cmd_client.write("226 Listed.\r\n")
                except:
                    self.msg_550_fail()
            utime.sleep_ms(50)

        elif command == "RETR":
            if self.debug:
                print("       CWD: [{}]".format(self.cwd))
            self.last_transfer_save = False

            if self.dataclient is None:
                self.msg_550_fail()
            else:
                # Some clients requires the following confirmation, some don't
                self.cmd_client.write("150 OK Ready to send data.\r\n")
                # send the file
                if self.send_file_data(0) is True:
                    self.last_transfer_path = None
                    self.cmd_client.write("226 Transfer complete.\r\n")
                else:
                    self.last_transfer_path = self.path
                    self.msg_550_fail()

        elif (command == "STOR") or (command == "APPE"):
            if self.debug:
                print("       CWD: [{}]".format(self.cwd))
            self.last_transfer_save = True

            if self.dataclient is None:
                self.msg_550_fail()
            else:
                # Some clients requires the following confirmation, some don't
                # Check if data already received
                utime.sleep_ms(100)
                if (self.dataclient.inbuf() == 0):
                    # nothing received, send request
                    self.cmd_client.write("150 OK Ready to receive data.\r\n")
                # receive the file
                if command == "STOR":
                    res = self.save_file_data(0)
                else:
                    res = self.save_file_data(-1)
                if res is True:
                    self.last_transfer_path = None
                    self.cmd_client.write("226 Transfer complete.\r\n")
                else:
                    self.last_transfer_path = self.path
                    self.msg_550_fail()

        elif (command == "REST") and (last_transfer_path is not None):
            if self.debug:
                print("       CWD: [{}]".format(self.cwd))

            if self.dataclient is None:
                self.msg_550_fail()
            else:
                if self.last_transfer_save is True:
                    # Some clients requires the following confirmation, some don't
                    self.cmd_client.write("150 OK Ready to receive data.\r\n")
                    # receive the file
                    res = self.save_file_data(int(self.payload))
                else:
                    # Some clients requires the following confirmation, some don't
                    self.cmd_client.write("150 OK Ready to send data.\r\n")
                    # send the file
                    try:
                        pos = int(self.payload)
                        res = self.send_file_data(pos)
                    except:
                        res = False
                if res is True:
                    self.last_transfer_path = None
                    self.cmd_client.write("226 Transfer complete.\r\n")
                else:
                    self.last_transfer_path = self.path
                    self.msg_550_fail()
        # ------------------------------------------------------------------------------------

        elif command == "DELE":
            try:
                uos.remove(self.path)
                self.msg_250_OK()
            except:
                self.msg_550_fail()

        elif command == "RMD":
            try:
                uos.rmdir(self.path)
                self.msg_250_OK()
            except:
                self.msg_550_fail()

        elif command == "MKD":
            try:
                uos.mkdir(self.path)
                self.msg_250_OK()
            except:
                self.msg_550_fail()

        elif command == "RNFR":
                self.fromname = self.path
                try:
                    _ = uos.stat(self.fromname)
                    self.cmd_client.write("350 Rename from\r\n")
                except:
                    self.msg_550_fail()

        elif command == "RNTO":
                if self.fromname is not None:
                    try:
                        uos.rename(self.fromname, self.path)
                        self.msg_250_OK()
                    except:
                        self.msg_550_fail()
                else:
                    self.msg_550_fail()
                self.fromname = None

        elif command == "STAT":
            if self.payload == "":
                self.cmd_client.write("211-Connected to ({})\r\n"
                         "    Data address ({})\r\n"
                         "211 TYPE: Binary STRU: File MODE: Stream\r\n".format(
                           self.remote_addr[0], self.local_IP))
            else:
                self.cmd_client.write("213-Directory listing:\r\n")
                self.send_list_data(self.cmd_client, True)
                self.cmd_client.write("213 Done.\r\n")

        else:
            self.cmd_client.write("502 Command not implemented.\r\n")
            if self.debug:
                print("Unsupported command [{}] with self.payload [{}]".format(command, self.payload))

        # ============================================================================================

    #-----------------------
    def serverProcess(self):
        try:
            self.fromname = None
            self.do_run = True
            self.active_mode = None
            self.last_transfer_path = None
            self.last_transfer_save = False
            self.data_request = False
            self._state = "Running"
            self._started = True
            if self.threaded:
                gc_tmo = 150
            else:
                gc_tmo = 30

            while self.do_run:
                # === Wait for the connection from client ===
                self._state = "Waiting connection"
                self.cmd_client, self.remote_addr = self.ftpsocket.accepted()
                if self.cmd_client == None:
                    if self.threaded:
                        # If running in thread, check for Quit request
                        if MicroFtpSrv.check_notify_quit() is True:
                            self.do_run = False;
                            break
                        #utime.sleep_ms(2)
                    gc_tmo -= 1
                    if gc_tmo <= 0:
                        if self.threaded:
                            gc_tmo = 150
                        else:
                            gc_tmo = 30
                        gc.collect()
                    continue

                # ==== Client connected ====
                self.cmd_client.settimeout(self._sock_timeout)
                if self.threaded:
                    gc_tmo = 150
                else:
                    gc_tmo = 30

                self.cwd = '/'
                try:
                    if self.debug:
                        print("FTP connection from:", self.remote_addr)
    
                    self.cmd_client.write("220 Hello, welcome to MicroPython FTP Server.\r\n")

                    # ======================
                    # ==== Command loop ====
                    # ======================
                    while self.do_run:
                        self._state = "Connected"
                        if self.threaded:
                            # If running in thread, check for Quit request
                            if MicroFtpSrv.check_notify_quit() is True:
                                self.do_run = False;
                                break

                        # Check if the client was disconnected
                        if self.cmd_client.peer_closed():
                            if self.debug:
                                print("Client disconnected (peer closed)")
                            if not self.threaded:
                                self.do_run = False;
                            break

                        # Close data connection if i was opened for previous command
                        if self.data_request is True:
                            self.data_request = False
                        else:
                            if self.dataclient is not None:
                                try:
                                    self.dataclient.close()
                                except:
                                    pass
                                self.dataclient = None
    
                        # Read data (command) from client
                        if self.cmd_client.inbuf() == 0:
                            utime.sleep_ms(100)
                            continue
                        try:
                            data = self.cmd_client.wifi_readline("\r\n").decode("utf-8").rstrip("\r\n")
                        except:
                            data = ()
                        if len(data) <= 0:
                            utime.sleep_ms(100)
                            continue
    
                        # Command received
                        self._state = "Processing command"
                        command = data.split(" ")[0].upper()
                        self.payload = data[len(command):].lstrip()
                        self.path = MicroFtpSrv.get_absolute_path(self.cwd, self.payload)
    
                        if (self.last_transfer_path is not None) and (command != "REST") and (command != "PORT") and (command != "PASSV"):
                            self.last_transfer_path = None
                        if self.debug:
                            if self.active_mode is True:
                                dmode = "Active"
                            elif self.active_mode is False:
                                dmode = "Passive"
                            else:
                                dmode = "?"
                            print("Command: [{}]\r\n   Payload: [{}]\r\n      Path: [{}]\r\n  Transfer: [{}]\r\n      Mode: [{}]".format(command, self.payload, self.path, self.last_transfer_path, dmode))
    
                        # ==== Parse the command ===========================================================
                        self.processCommand(command)
                        gc.collect()
    
                # === Command loop exited ===
                except Exception as err:
                    print("FTP: Cmd loop Exception: {}".format(err))
                    if self.debug:
                        sys.print_exception(err)
                    if not self.threaded:
                        self.do_run = False;
    
                finally:
                    if self.dataclient is not None:
                        # Data socket was used, close it
                        try:
                            self.dataclient.close()
                        except:
                            pass
                        self.dataclient = None
                    try:
                        self.cmd_client.close()
                    except:
                        pass
                    self.cmd_client = None

        # === Main FTP loop exited ===
        except Exception as err:
            print("[MicroFtpSrv]: Main loop Exception: {}".format(err))
            if self.debug:
                sys.print_exception(err)
    
        finally:
            if self.debug:
                print("Exit: Closing sockets.")
            if self.dataclient is not None:
                try:
                    self.dataclient.close()
                except:
                    pass
            if self.datasocket is not None:
                try:
                    self.datasocket.close()
                except:
                    pass
                self.datasocket = None
            if self.ftpsocket is not None:
                try:
                    self.ftpsocket.close()
                except:
                    pass
                self.ftpsocket = None
            self.thID = 0
            self._started = False
            self._state = "Stopped"

        gc.collect()
        print("[MicroFtpSrv] Server finished.")

    #==========================================================================================================
    def Start(self, timeout = 300, threaded = False, user = None, passwd = None, passive = True, debug = None):
        self.debug = debug
        self.threaded = threaded
        self.passive = passive
        self.user = user
        self.passwd = passwd
        self._sock_timeout = timeout
    
        print ("Starting ftp server. Version 1.2")
    
        if not network.wifi_active():
            print("WLAN not connected!")
            return
     
        # Open the listening socket on port 21
        try:
            self.ftpsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP)
            self.ftpsocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.ftpsocket.listen(1) # only one connection allowed
            self.ftpsocket.bind(("0.0.0.0", 21), self._sock_timeout+60)
            if self.threaded is True:
                self.ftpsocket.settimeout(2)
            else:
                self.ftpsocket.settimeout(10)
        except:
            print("Error opening command socket")
            return
    
        if self.passive is True:
            # Open the data listening socket for passive mode
            try:
                self.datasocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP)
                self.datasocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.datasocket.listen(1) # only one connection allowed
                self.datasocket.bind(("0.0.0.0", self._passive_port), 60)
                self.datasocket.settimeout(1)
            except:
                self.ftpsocket.close()
                print("Error opening data socket")
                return
        else:
            self.datasocket = None
    
        self.dataclient = None
        self.local_IP = network.wifi.ifconfig()[0]
        print("Running on IP address:", self.local_IP)

        if self.threaded :
            try :
                gc.collect()
                th = _thread.start_new_thread("MicroFtpSrv", self.serverProcess, (), stacksize=2048)
            except Exception as e:
                print("Exception starting thread: {}".format(e))
                if self.debug:
                    print("Exit: Closing sockets.")
                if self.datasocket is not None:
                    try:
                        self.datasocket.close()
                    except:
                        pass
                    self.datasocket = None
                if self.ftpsocket is not None:
                    try:
                        self.ftpsocket.close()
                    except:
                        pass
                        self.ftpsocket = None
                self.thID = 0
                self._started = False
                self._state = "Stopped"
            if th > 0:
                self.thID = th
            else:
                print("Failed to start FTP server thread")
                if self.debug:
                    print("Exit: Closing sockets.")
                if self.datasocket is not None:
                    try:
                        self.datasocket.close()
                    except:
                        pass
                    self.datasocket = None
                if self.ftpsocket is not None:
                    try:
                        self.ftpsocket.close()
                    except:
                        pass
                        self.ftpsocket = None
        else :
            self.serverProcess()

    #--------------
    def Stop(self):
        if self._started:
            if self.threaded:
                _ = _thread.notify(self.thID, _thread.EXIT)

    #-------------------
    def IsStarted(self):
        return self._started

    #------------------
    def threadID(self):
        return self.thID

    #---------------
    def State(self):
        return self._state

