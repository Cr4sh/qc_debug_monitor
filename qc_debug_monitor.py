#!/usr/bin/env python

###############################################################
#
#  Debug messages monitor for Qualcomm cellular modems.
#
#  This program talks to the baseband firmware over the 
#  diag protocol serial port.
#
#  Written by:
#  Dmytro Oleksiuk (aka Cr4sh)
#
#  cr4sh0@gmail.com
#  http://blog.cr4.sh
#
###############################################################

import sys, os, signal, time, tempfile, logging

from struct import pack, unpack
from Queue import Queue
from threading import Thread
from optparse import OptionParser, make_option
from serial import Serial

SERIAL_BAUDRATE = 115200

SERIAL_TIMEOUT = 0.1

TIMESTAMP = '%H:%M:%S'

PID_FILE_PATH = os.path.join(tempfile.gettempdir(), 'diag_msg.pid')

KNOWN_SUBSYSTEMS = [

    ( 0x0000, 0x00c8 ), ( 0x01f4, 0x02bc ),
    ( 0x03e8, 0x04b0 ), ( 0x07d0, 0x0898 ),
    ( 0x0bb8, 0x0c80 ), ( 0x0fa0, 0x1068 ),
    ( 0x1194, 0x12c0 ), ( 0x1388, 0x1450 ),
    ( 0x157c, 0x1644 ), ( 0x1770, 0x1838 ),
    ( 0x1964, 0x1a2c ), ( 0x1b58, 0x1ce8 ),
    ( 0x1f40, 0x2008 ), ( 0x2134, 0x21fc ),
    ( 0x2328, 0x23f0 ), ( 0x251c, 0x25e4 ),
    ( 0x27d8, 0x2968 ) ]

class Daemon(object):

    UMASK = 0
    REDIRECT_TO = os.devnull

    def __init__(self):  

        sys.stdout.flush()
        sys.stderr.flush()     
          
        # fork a child process so the parent can exit
        if os.fork() == 0:
          
            # To become the session leader of this new session and the process group
            # leader of the new process group, we call os.setsid().  The process is
            # also guaranteed not to have a controlling terminal.
            os.setsid()    

            # fork a second child and exit immediately to prevent zombies
            if os.fork() == 0:

                # give the child complete control over permissions
                os.umask(self.UMASK)

            else:

                time.sleep(2)

                # exit parent of the second child
                os._exit(0) 

        else:

            time.sleep(2)

            #exit parent of the first child
            os._exit(0) 

        # redirect the standard I/O file descriptors to the /dev/null        
        si = file(self.REDIRECT_TO, 'r')
        so = file(self.REDIRECT_TO, 'a+')
        se = file(self.REDIRECT_TO, 'a+', 0)

        os.dup2(si.fileno(), sys.stdin.fileno())
        os.dup2(so.fileno(), sys.stdout.fileno())
        os.dup2(se.fileno(), sys.stderr.fileno())

class Singleton(type):

    _instances = {}

    def __call__(cls, *args, **kwargs):

        if cls not in cls._instances:

            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)

        return cls._instances[cls]

class DiagThread(Thread):

    def __init__(self, diag):        

        super(DiagThread, self).__init__()

        self.diag = diag
        self.daemon = True                

class ReceiveThread(DiagThread):

    __metaclass__ = Singleton

    def run(self):
        
        self.diag.do_receive()

class DispatchThread(DiagThread):         

    __metaclass__ = Singleton

    def run(self):
        
        self.diag.do_dispatch()

class DiagSerial(object):

    def __init__(self, name, baudrate = None):

        baudrate = SERIAL_BAUDRATE if baudrate is None else baudrate

        self.port = Serial(name, baudrate = baudrate, timeout = SERIAL_TIMEOUT,
                                 rtscts = True, dsrdtr = True)
        self.flush()

    def flush(self):

        self.port.flush()

    def read(self, size):

        return self.port.read(size)

    def write(self, data):

        from hexdump import hexdump

        self.port.write(data)

class DiagProt(DiagSerial):

    CMD_MAX_SIZE = 0x1000
    CMD_TIMEOUT = 5

    VERNO_F           = 0x00
    PEEKB_F           = 0x02
    POKEB_F           = 0x05
    EXT_MSG_CONFIG_F  = 0x7d
    EXT_MSG_F         = 0x79
    EXT_MSG_TERSE_F   = 0x92

    EXT_MSG_SUBCMD_SET_RT_MASK = 0x04    

    EXT_MSG_SUBCMD_GET_RANGES = 0x01

    MAX_SSID = 0xcfff

    crc_table = [

        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
        0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
        0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
        0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
        0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
        0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
        0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
        0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
        0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
        0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
        0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
        0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
        0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
        0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
        0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
        0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
        0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
        0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
        0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
        0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
        0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
        0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
        0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
        0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
        0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
        0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
        0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
        0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
        0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
        0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
        0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
        0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78 ]

    def __init__(self, device_name, output_file = None, baudrate = None):

        self.running = True
        self.initialized = False
        self.q = Queue()

        self.log_init(output_file)

        self.logger.info('[+] Opening %s' % device_name)

        super(DiagProt, self).__init__(device_name, baudrate = baudrate)

    def join(self):

        self.q.join()

    def start_threads(self):

        # start packets dispatcher thread
        DispatchThread(self).start()

        # start paclets reader thread
        ReceiveThread(self).start()

    def log_init(self, output_file = None):

        self.logger = logging.getLogger(__name__)                

        if output_file:

            if os.path.isfile(output_file):

                # delete old output file
                os.unlink(output_file)

            self.logger.addHandler(logging.FileHandler(output_file))

        self.logger.addHandler(logging.StreamHandler(sys.stdout))
        self.logger.setLevel(logging.DEBUG)

    def crc_16(self, data):

        ret = 0xffff

        for b in data:

            ret = (ret >> 8) ^ self.crc_table[(ret ^ ord(b)) & 0xff]

        return ret ^ 0xffff

    def escape(self, data):

        return data.replace('\x7d', '\x7d\x5d').replace('\x7e', '\x7d\x5e')

    def unescape(self, data):

        return data.replace('\x7d\x5e', '\x7e').replace('\x7d\x5d', '\x7d')

    def msg_format(self, message, args):

        message = message.replace('%p', '%x')

        return message % tuple(args[: message.count('%')])

    def make_packet(self, data):

        data = data + pack('<H', self.crc_16(data))        
        data = self.escape(data) + '\x7e'

        return data

    def write(self, cmd, sub_cmd = None, data = ''):

        hdr = pack('<BB', cmd, sub_cmd) if sub_cmd else pack('<B', cmd)

        super(DiagProt, self).write(self.make_packet(hdr + data))

    def read(self):

        return super(DiagProt, self).read(self.CMD_MAX_SIZE)

    def send_verno(self):

        cnt = 0
        self.model, self.revision = None, None

        # send request
        self.write(self.VERNO_F)

        # wait for the reply
        while self.model is None and self.revision is None:
            
            if cnt >= self.CMD_TIMEOUT:

                raise(Exception('VERNO_F reply timeout'))

            time.sleep(1)
            cnt += 1

        self.logger.info('[+] Device model: %s' % self.model)
        self.logger.info('[+] Revision: %d' % self.revision)        

        return self.model, self.revision

    def recv_verno(self, data):
    
        # parse VERNO_F header
        comp_date, comp_time, rel_date, rel_time, \
        model, scm, mob_cai_rev, mob_model, \
        mob_firmware_rev, slot_cycle_index, \
        msm_ver, _ = unpack('<11s8s11s8s8sBBBHBBB', data)

        self.model, self.revision = model, mob_firmware_rev

    def send_msg_config_set_rt_mask(self, ssid_start, ssid_end, on = True):

        data = ''

        # create masks list for specified subsystems
        for i in range(0, ssid_end - ssid_start + 1):
            
            data += pack('<I', 0xffffffff if on else 0)

        # send request
        self.write(self.EXT_MSG_CONFIG_F, self.EXT_MSG_SUBCMD_SET_RT_MASK,
                   pack('<HHH', ssid_start, ssid_end, 0) + data)      

    def recv_msg_config_set_rt_mask(self, data):

        pass

    def recv_msg(self, data):

        args_list = []

        if not self.initialized: return

        # parse EXT_MSG_F header
        _, num_args, _, timestamp, line, ssid, _ = unpack('<BBBQHHL', data[: 19])

        args = data[19 :]
        message = args[(num_args * 4) :]
        message, file_name, _ = message.split('\0')

        for i in range(0, num_args):
        
            # get argument value
            args_list.append(unpack('<I', args[(i * 4) : (i * 4) + 4])[0])

        message = message.strip()
        file_name = file_name.strip()

        try:

            message = self.msg_format(message, args_list)

        except:

            self.logger.error('[!] EXT_MSG_F format string error for "%s" (%d args)' % \
                              (message, num_args))
            return

        # process message
        self.handle_message(ssid, message, file_name, line)

    def recv_msg_terse(self, data):

        args_list = []

        if not self.initialized: return

        # parse EXT_MSG_TERSE_F header
        _, num_args, _, timestamp, line, ssid, _, msg = unpack('<BBBQHHLL', data[: 23])

        args = data[23 :]

        for i in range(0, num_args):
        
            # get argument value
            args_list.append(unpack('<I', args[(i * 4) : (i * 4) + 4])[0])

        # process message
        self.handle_message_terse(ssid, line, msg, args_list)

    def do_receive(self):

        old_data = ''

        self.flush()

        while True:

            # read data stream
            data = self.read()
            if data is None: continue

            data = old_data + data
            data = data.split('\x7e')
            old_data = data.pop()

            for packet in data:

                # queue packet
                self.q.put(packet)

    def do_dispatch(self):

        while True:
        
            packet = self.q.get()
        
            if len(packet) >= 3:

                # parse packet
                self.handle_packet(self.unescape(packet))

            self.q.task_done()        

    def handle_packet(self, packet):        

        # parse packet header
        data = packet[1 : -2]
        cmd = ord(packet[0])
        crc, = unpack('<H', packet[-2 :])        

        # verify packet control sum
        if self.crc_16(packet[: -2]) != crc:

            self.logger.error('[!] Bad diag packet control sum')
            self.running = False

        else:

            self.handle_command(cmd, data)

    def handle_command(self, cmd, data):        

        try:

            #
            # Parse packet of specific type
            #
            {          self.VERNO_F: self.recv_verno,
              self.EXT_MSG_CONFIG_F: self.recv_msg_config_set_rt_mask,
                     self.EXT_MSG_F: self.recv_msg,
               self.EXT_MSG_TERSE_F: self.recv_msg_terse }[cmd](data)

        except KeyError:

            self.logger.error('[!] Unknown command: 0x%.2x' % cmd)

        except:

            self.running = False
            raise

    def handle_message(self, ssid, message, file_name, line):

        pass

    def handle_message_terse(self, ssid, message, args):

        pass

class DiagSubsystemsFinder(DiagProt):

    def __init__(self, device_name, output_file = None, baudrate = None):

        self.known_subsystems = []
        self.ssid_start = self.ssid_end = None

        super(DiagSubsystemsFinder, self).__init__(device_name, output_file = output_file,
                                                                baudrate = baudrate)

    def recv_msg_config_set_rt_mask(self, data):

        # parse EXT_MSG_CONFIG_F header
        _, _, ssid, count = unpack('<BHHH', data[: 7])

        if count > 0:

            # valid subsystem ID was found
            if self.ssid_end is not None:

                if self.ssid_end + 1 != ssid:

                    print('0x%.4x:0x%.4x' % (self.ssid_start, self.ssid_end))

                    self.known_subsystems.append((self.ssid_start, self.ssid_end))
                    self.ssid_start = self.ssid_end = ssid

                else: self.ssid_end += 1

            else: self.ssid_start = self.ssid_end = ssid

    def start(self):

        self.start_threads()

        # verify diag connection
        self.send_verno()

        self.logger.info('[+] Finding all available subsystem IDs, it may take a while...\n')

        subsys = 0

        # enumerate and test all spossible ubsystems
        while subsys < self.MAX_SSID:
                        
            self.send_msg_config_set_rt_mask(subsys, subsys)
            subsys += 1

        self.logger.info('\n[+] DONE')

class DiagLogger(DiagProt):

    terse_show = False
    terse_hash_db = None    

    def __init__(self, device_name, output_file = None, output_dir = None,
                       baudrate = None, show_terse = False, hash_db = None):

        self.subsystems = None
        self.output_dir, self.output_fd = output_dir, {}

        self.terse_enable(on = show_terse, hash_db_path = hash_db)

        super(DiagLogger, self).__init__(device_name, output_file = output_file, 
                                                      baudrate = baudrate)

    def get_ssid_log_fd(self, ssid):

        if self.output_dir is None: 

            # dedicated subsystems logs are not used
            return None

        if self.output_fd.has_key(ssid): 

            # return existing log file handle
            return self.output_fd[ssid]

        log_path = os.path.join(self.output_dir, 'ssid_%.4x.log' % ssid)

        # create new log file for this subsystem
        fd = open(log_path, 'wb')

        self.output_fd[ssid] = fd

        return fd

    def handle_message(self, ssid, message, file_name = None, line = None):

        supress = False
        timestamp = location = ''        

        if TIMESTAMP is not None:

            timestamp = '[%s] ' % time.strftime(TIMESTAMP, time.localtime(time.time()))
            
        if file_name is not None and line is not None:

            location = ' : %s(%d)' % (file_name, line)

        if self.subsystems is not None:

            supress = True

            for ssid_start, ssid_end in self.subsystems:

                # check if we need to show message for given subsystem
                if ssid >= ssid_start and ssid <= ssid_end:

                    supress = False
                    break

        message = '%s0x%.4x%s : %s' % (timestamp, ssid, location, message)

        if not supress:

            # write message to the main log
            self.logger.debug(message)    

        fd = self.get_ssid_log_fd(ssid)
        if fd is not None:

            # write message to the subsystem specific log
            fd.write(message + '\n')

    def handle_message_terse(self, ssid, line, message, args):

        if self.terse_hash_db is not None and \
           self.terse_hash_db.has_key(message):

            # get message text and file name by hash
            msg_file, msg_text = self.terse_hash_db[message]

            try:

                msg_text = self.msg_format(msg_text, args)

            except:

                self.logger.error('[!] EXT_MSG_TERSE_F format string error for "%s" (%d args)' % \
                                  (msg_text, len(args)))
                return

            # log plaintext terse message
            self.handle_message(ssid, msg_text, msg_file, line)

        elif self.terse_show:

            args = ', '.join(map(lambda v: '0x%.8x' % v, args))

            # log raw terse message
            self.handle_message(ssid, 'MSG = 0x%.8x, ARGS = ( %s )' % (message, args))    

    def terse_enable(self, on = True, hash_db_path = None):

        self.terse_show = on
        self.terse_hash_db = None

        if hash_db_path is not None: 

            print('[+] Loading hash DB from %s' % hash_db_path)

            self.terse_hash_db = self.terse_load_hash_db(hash_db_path)

            print('[+] %d hashes loaded' % len(self.terse_hash_db))

    def terse_load_hash_db(self, file_path):

        ret = {}

        # load messages hash DB from file
        with open(file_path, 'rb') as fd:

            line = fd.readline()

            while line:
               
                items = line.strip().split(',')
                if len(items) > 2:

                    try: 

                        # decode hash value
                        val = int(items[0], 16) 

                    except ValueError: val = None

                    if val is not None:

                        # decode file name and message text
                        msg_file = items[1]
                        msg_text = ','.join(items[2: ])

                        ret[val] = ( msg_file, msg_text )


                line = fd.readline()

        return ret

    def start(self, subsystems = None):

        self.start_threads()

        # verify diag connection
        self.send_verno()

        self.subsystems = subsystems

        for ssid_start, ssid_end in KNOWN_SUBSYSTEMS:

            # enable/disable messages for known subsystem
            self.send_msg_config_set_rt_mask(ssid_start, ssid_end, on = subsystems is None)

        if subsystems is not None:

            for ssid_start, ssid_end in subsystems:

                # enable/disable messages for specific subsystem
                self.send_msg_config_set_rt_mask(ssid_start, ssid_end)

        self.logger.info('')

        self.initialized = True     

g_pid_file = None

def pid_cleanup():

    global g_pid_file

    if g_pid_file is not None:

        g_pid_file.close()
        g_pid_file = None
        
        try: os.unlink(PID_FILE_PATH)
        except: pass

def pid_write(pid):

    global g_pid_file

    assert g_pid_file is None

    g_pid_file = open(PID_FILE_PATH, 'wb')

    g_pid_file.write(str(pid))
    g_pid_file.flush()

def pid_read():       

    if os.path.isfile(PID_FILE_PATH):

        with open(PID_FILE_PATH, 'rb') as fd:

            return int(fd.read().strip())

    return None

def pid_stop():

    # read background process PID
    pid = pid_read()

    if pid is None:
        
        return None

    print('[+] Sending SGINT to process %d...' % pid)

    try:

        # terminate backgroind process
        os.kill(pid, signal.SIGINT)
        time.sleep(1)

    except OSError, e:

        try: os.unlink(PID_FILE_PATH)
        except: pass

        print(str(e))

    return pid

def main():

    option_list = [

        make_option('-d', '--device-name', dest = 'device_name', default = None,
            help = 'device name of diag interface serial port'),

        make_option('-b', '--baudrate', dest = 'baudrate', default = None,
            help = 'serial port speed', metavar = 'NUMBER', type = 'int'),

        make_option('-o', '--output-file', dest = 'output_file', default = None,
            help = 'output file to write debug messages log'),

        make_option('--output-dir', dest = 'output_dir', default = None,
            help = 'output directory to write separate subsystem logs'),

        make_option('-f', '--find-subsystems', dest = 'find_subsystems', default = False,
            help = 'find all valid subsystem IDs for this device', action = 'store_true'),

        make_option('-s', '--subsystems', dest = 'subsystems', default = None,
            help = 'show messages only for specified subsystems'),

        make_option('-t', '--show-terse', dest = 'show_terse', default = False,
            help = 'show not decoded EXT_MSG_TERSE_F messages', action = 'store_true'),

        make_option('--hash-db', dest = 'hash_db', default = None,
            help = 'messages hash DB file path to decode EXT_MSG_TERSE_F messages') ] 

    if sys.platform != 'win32':

        option_list.append(make_option('--daemon', dest = 'daemon', default = False,
            help = 'run in the background', action = 'store_true'))

        option_list.append(make_option('--stop', dest = 'stop', default = False,
            help = 'stop background process', action = 'store_true'))

    options, _ = OptionParser(option_list = option_list).parse_args()

    if options.stop:

        # terminate background process
        if pid_stop() is None:

            print('[!] Background process is not running')
            return -1

        return 0

    if options.device_name is None:

        print('ERROR: Device name is not specified')
        return -1

    subsystems = None

    if options.subsystems is not None:

        subsystems = []

        try:

            # parse subsystems list
            for ssid_range in options.subsystems.split(' '):

                ssid_range = ssid_range.strip().replace(',', ':').split(':')

                assert len(ssid_range) in [ 1, 2 ]

                if len(ssid_range) == 1:

                    ssid_start, ssid_end = ssid_range[0], ssid_range[0]

                else:

                    ssid_start, ssid_end = ssid_range

                ssid_val = lambda v: int('0x' + v.strip().replace('0x', ''), 16)

                subsystems.append(( ssid_val(ssid_start), ssid_val(ssid_end) ))
        except:

            print('ERROR: Bad subsystems list')
            return -1    

    if options.find_subsystems:
    
        diag = DiagSubsystemsFinder(options.device_name, output_file = options.output_file,
                                                         baudrate = options.baudrate)
        try:
        
            diag.start()

        except KeyboardInterrupt:

            print('\nEXIT\n')

    else:

        diag = DiagLogger(options.device_name, output_file = options.output_file,
                                               output_dir = options.output_dir,
                                               baudrate = options.baudrate, 
                                               show_terse = options.show_terse,
                                               hash_db = options.hash_db) 

        # terminate already running background process
        pid_stop()

        if options.daemon:            

            print('[+] Going to the background...')

            # daemonize current process
            Daemon()

            # save background process PID
            pid_write(os.getpid())

        diag.start(subsystems = subsystems)

        try:

            while diag.running: time.sleep(1)

        except KeyboardInterrupt: pass

        diag.join()

        print('\nEXIT\n')

    pid_cleanup()
    
    return 0

if __name__ == '__main__':

    exit(main())

#
# EoF
#
