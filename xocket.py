
import time
import serial
import thread
import os
import struct

PACKET_OFFSET_ID = 0
PACKET_OFFSET_TYPE = 1
PACKET_OFFSET_PID = 2
PACKET_OFFSET_SEQ = 3
PACKET_OFFSET_CNT = 3
PACKET_HEADER_LEN = 4
#PACKET_OFFSET_CRC = 254
PACKET_OFFSET_CRC = 1022
#PACKET_LEN  = 256
PACKET_LEN  = 1024
CRC_LEN = 2
PACKET_DATA_LEN = PACKET_LEN - PACKET_HEADER_LEN - CRC_LEN

PACKET_TYPE_REQ = 0 # request data packet
PACKET_TYPE_REPLY = 1 #reply packet
PACKET_TYPE_ACK = 2 # acknowledge pacekt
PACKET_TYPE_DATA = 3 # data packet
PACKET_TYPE_QUIT = 4 # quit transfer

GET_PACKET_STATE_IDLE = 0
GET_PACKET_STATE_REQ = 1 
GET_PACKET_STATE_WAIT_REPLY = 2 
GET_PACKET_STATE_RECEIVE_DATA = 3 
GET_PACKET_STATE_SEND_ACK = 4 
GET_PACKET_STATE_QUIT = 5 

SEND_PACKET_STATE_IDLE = 0
SEND_PACKET_STATE_REPLY = 1 
SEND_PACKET_STATE_SEND = 2 
SEND_PACKET_STATE_WAIT_ACK = 3 
SEND_PACKET_STATE_TIMEOUT = 4 
SEND_PACKET_STATE_QUIT = 5 


def setup_packet_offset_dict(po_str):
    po_str[PACKET_OFFSET_ID] = "PACKET_OFFSET_ID"
    po_str[PACKET_OFFSET_TYPE] = "PACKET_OFFSET_TYPE"
    po_str[PACKET_OFFSET_PID] = "PACKET_OFFSET_PID"
    po_str[PACKET_OFFSET_SEQ] = "PACKET_OFFSET_SEQ"
    po_str[PACKET_OFFSET_CRC] = "PACKET_OFFSET_CRC"

def setup_packet_type_dict(pt_str):
    pt_str[PACKET_TYPE_REQ] = "PACKET_TYPE_REQ"
    pt_str[PACKET_TYPE_REPLY] = "PACKET_TYPE_REPLY"
    pt_str[PACKET_TYPE_ACK] = "PACKET_TYPE_ACK"
    pt_str[PACKET_TYPE_DATA] = "PACKET_TYPE_DATA"
    pt_str[PACKET_TYPE_QUIT] = "PACKET_TYPE_QUIT"

def setup_get_packet_state_dict(gps_str):
    gps_str[GET_PACKET_STATE_IDLE] = "GET_PACKET_STATE_IDLE"
    gps_str[GET_PACKET_STATE_REQ] = "GET_PACKET_STATE_REQ"
    gps_str[GET_PACKET_STATE_WAIT_REPLY] = "GET_PACKET_STATE_WAIT_REPLY"
    gps_str[GET_PACKET_STATE_RECEIVE_DATA] = "GET_PACKET_STATE_RECEIVE_DATA"
    gps_str[GET_PACKET_STATE_SEND_ACK] = "GET_PACKET_STATE_SEND_ACK"
    gps_str[GET_PACKET_STATE_QUIT] = "GET_PACKET_STATE_QUIT"

def setup_send_packet_state_dict(sps_str):
    sps_str[SEND_PACKET_STATE_IDLE] = "SEND_PACKET_STATE_IDLE"
    sps_str[SEND_PACKET_STATE_REPLY] = "SEND_PACKET_STATE_REPLY"
    sps_str[SEND_PACKET_STATE_SEND] = "SEND_PACKET_STATE_SEND"
    sps_str[SEND_PACKET_STATE_WAIT_ACK] = "SEND_PACKET_STATE_WAIT_ACK"
    sps_str[SEND_PACKET_STATE_TIMEOUT] = "SEND_PACKET_STATE_TIMEOUT"
    sps_str[SEND_PACKET_STATE_QUIT] = "SEND_PACKET_STATE_QUIT"

    
class xocket(object):

    def __init__(self, dev = "/dev/tty/USB0", baudrate = 115200, my_board_id = 0, target_board_id = 1, pid = 0, packet_len = 1024, out_file = "log.txt"):
        self.dev = dev
        self.baudrate = baudrate
        self.my_board_id = my_board_id
        self.target_board_id = target_board_id
        self.packet_len = packet_len
        self.out_file = out_file
        self.po_str = {}
        self.pt_str = {}
        self.gps_str = {}
        self.sps_str = {}
        self.pid = pid
        self.total_received_byres = 0
        self.total_send_bytes = 0
        self.receive_packet_done = False
        self.send_packet_done = False
        self.receive_state = GET_PACKET_STATE_IDLE
        self.send_state = SEND_PACKET_STATE_IDLE
        
        setup_packet_offset_dict(self.po_str)
        setup_packet_type_dict(self.pt_str)
        setup_send_packet_state_dict(self.sps_str)
        setup_get_packet_state_dict(self.gps_str)
        self.ser = serial.Serial(port = dev,
                                 baudrate = baudrate,
                                 parity = serial.PARITY_NONE,
                                 stopbits = serial.STOPBITS_ONE,
                                 bytesize = serial.EIGHTBITS,
                                 timeout = 1)

    ''' save msg to file '''
    def save_msg_to_file(self, msg):
        try:
            file = open(self.out_file, 'a+')
            file.write(msg)
        finally:
            file.close()


    def receive_packet(self):
        print("flyer get packet thread running")
        self.receive_state = GET_PACKET_STATE_IDLE
        packet_total_cnt = 0
        packet_cnt = 0
        _packet_seq = 0
        packet_seq = 0

        cur_time = time.time()

        while True:
            ''' initial state, ready go! '''
            if (self.receive_state == GET_PACKET_STATE_IDLE):
                self.total_receive_bytes = 0
                self.receive_pacekt_done = False
                print("start to receive data...")
                self.receive_state = GET_PACKET_STATE_REQ
                continue
                
            ''' request data from other side '''
            if (self.receive_state == GET_PACKET_STATE_REQ):
                print("flyer go into state GET_PACKET_STATE_REQ ")
                hdr = struct.pack('4B', self.target_board_id, PACKET_TYPE_REQ, self.pid, 0)

                packet = hdr + (PACKET_DATA_LEN + CRC_LEN) * '\0'
                print ("flyer wirte req packet len %d, target_board_id %d, type %s, pid %d,"
                        % (len(packet), self.target_board_id, self.pt_str[PACKET_TYPE_REQ], self.pid))
                print ("write start...")
                self.ser.write(packet)
                print("write done")
                self.receive_state = GET_PACKET_STATE_WAIT_REPLY
                cur_time = time.time()
                continue

            ''' wait reply from other side '''
            if (self.receive_state == GET_PACKET_STATE_WAIT_REPLY):
                print ("flyer go into state GET_PACKET_STATE_WAIT_REPLY")
                if (cur_time + 10 < time.time()):
                    print ("flyer wait reply timeout 10sec... flyer goto state GET_PACKET_STATE_QUIT")
                    self.receive_state = GET_PACKET_STATE_QUIT
                    continue

                packet = self.ser.read(PACKET_LEN)
                if (len(packet) < PACKET_LEN):
                    continue

                '''if (crc16_check(packet) == False):
                    state = GET_PACKET_STATE_QUIT
                    continue
                '''

                board_id, packet_type = struct.unpack('2B', packet[:PACKET_OFFSET_TYPE+1])
                if (board_id != self.my_board_id or packet_type != PACKET_TYPE_REPLY):
                    print("error! received packet len %d, board id: %d, expect id: %d " %(len(packet), board_id, self.my_board_id))
                    print ("type: %d, expected type: %s" %(packet_type, self.pt_str[PACKET_TYPE_REPLY]))
                    self.receive_state = GET_PACKET_STATE_QUIT
                    continue
                    
                pid, packet_total_cnt = struct.unpack('2B', packet[PACKET_OFFSET_PID:PACKET_OFFSET_CNT+1])
                if (packet_total_cnt <= 0 or pid != self.pid):
                    print("receive reply from station, packet total cnt %d" %(packet_total_cnt))
                    self.receive_state = GET_PACKET_STATE_QUIT
                    continue
                    
                print("receive reply from station, board_id: %d, packet_type: %s, _pid: %d, packet_total_cnt %d"
                        %(board_id, self.pt_str[packet_type], pid, packet_total_cnt))
                        
                self.receive_state = GET_PACKET_STATE_SEND_ACK

            ''' receive data state '''
            if (self.receive_state == GET_PACKET_STATE_RECEIVE_DATA):
                print ("flyer go into state GET_PACKET_STATE_RECEIVE_DATA")
                packet = self.ser.read(PACKET_LEN)

                if (len(packet) < 4):
                    if (cur_time + 1 < time.time()):
                        print ("flyer receive date timeout 1sec...")
                    continue

                board_id, packet_type = struct.unpack('2B', packet[:PACKET_OFFSET_TYPE+1])
                if (board_id != self.my_board_id or packet_type != PACKET_TYPE_DATA):
                    print("rcving error! packet id: %d, type %d, expect id: %d, type %s "
                            %(board_id, packet_type,  FLYER_BOARD_ID, self.pt_str[PACKET_TYPE_DATA]))
                    self.receive_state = GET_PACKET_STATE_QUIT
                    continue

                pid, packet_seq = struct.unpack('2B', packet[PACKET_OFFSET_PID:PACKET_OFFSET_SEQ+1])
                if (pid != self.pid or packet_seq != _packet_seq):
                    print("rcving error! packet pid: %d, seq: %d, expected pid: %d, seq %d" %(pid, packet_seq, _pid, _packet_seq))
                    self.receive_state = GET_PACKET_STATE_QUIT
                    continue

                print("receive data from station, board_id: %d, packet_type: %s, pid: %d, packet_seq %d"
                        %(board_id, self.pt_str[packet_type], pid, packet_seq))

                msg = packet[PACKET_OFFSET_SEQ + 1: PACKET_LEN - CRC_LEN]

                #print ("#flyer recved len: %d, msg: %s"
                #        %(PACKET_LEN - CRC_LEN - PACKET_OFFSET_SEQ - 1, msg))

                self.save_msg_to_file(packet[PACKET_OFFSET_SEQ+1: PACKET_LEN - CRC_LEN])
                self.total_receive_bytes += len(packet)
                _packet_seq += 1
                self.receive_state = GET_PACKET_STATE_SEND_ACK


            if (self.receive_state == GET_PACKET_STATE_SEND_ACK):
                print ("flyer go into state GET_PACKET_STATE_SEND_ACK ")
                packet  = struct.pack("4B", self.target_board_id, PACKET_TYPE_ACK, self.pid, packet_seq)
                
                packet = packet + (PACKET_DATA_LEN + CRC_LEN - 4) * '1'
                packet += "phdc"
                print ("media wirte ACK packet len %d, target_board_id %d, packet_type:%s, pid %d, _packet_seq %d,"
                        % (len(packet), self.target_board_id,  self.pt_str[PACKET_TYPE_ACK], self.pid, packet_seq))

                self.ser.write(packet)

                if (_packet_seq == packet_total_cnt):
                    print("flyer recevie data ok, return")
                    ''' !!!delay for a while, waitting for the serial send all the data out '''
                    time.sleep(0.1)
                    self.receive_packet_done = True
                    
                    return True
                else:
                    self.receive_state = GET_PACKET_STATE_RECEIVE_DATA
                    continue


            ''' end the transfer '''
            if (self.receive_state == GET_PACKET_STATE_QUIT):
                print ("flyer go into GEP_PACKET_STATE_QUIT")
                self.receive_packet_done = False
                print ("flyer_get_packet thread exit...")
                time.sleep(0.1)
                return False


    def receive_packet_is_done(self):
        return self.receive_packet_done


    def received_bytes(self):
        return self.total_receive_bytes


    def is_request_packet(self, packet):
        board_id, packet_type = struct.unpack("2B", packet[:PACKET_OFFSET_TYPE+1])
        if (board_id != self.my_board_id or packet_type != PACKET_TYPE_REQ):
            print ("wrong packet,len %d, board id %d, type %d, expect board_id %d, packet_tyep  %s"
                    %(len(packet), board_id, packet_type, self.my_board_id, self.pt_str[PACKET_TYPE_REQ]))               
            return False
        else:
            print("station receive one request packet, board_id %d, packet_type %s"
                    %(board_id, self.pt_str[packet_type]))
            return True

    ''' before go into send_packet, we have got pid  and target board id from the request packet '''        
    def send_packet(self, in_file_name, pid):
        self.pid = pid
        self.send_state = SEND_PACKET_STATE_REPLY
        self.send_packet_done = False
        sendfile = open(in_file_name, 'rb')
        file_size = os.path.getsize(in_file_name)
        packets_cnt = (file_size + PACKET_DATA_LEN - 1) / PACKET_DATA_LEN
        send_packet_cnt = 0
        last_packet_size = file_size % PACKET_DATA_LEN
        print("file size %d, packets_cnt %d, last_packet_size %d" %(file_size, packets_cnt, last_packet_size))
        _packet_seq = 0

        while True:
            if (self.send_state == SEND_PACKET_STATE_REPLY):
                print("station go inot state SEND_PACKET_STATE_REPLY")
                packet = struct.pack("4B", self.target_board_id, PACKET_TYPE_REPLY, self.pid, packets_cnt)
                packet = packet + (PACKET_DATA_LEN + CRC_LEN) * '\0'
                print ("station send packet len %d, target_board_id %d, type %s, pid %d, packets_cnt %d"
                        %(len(packet), self.target_board_id, self.pt_str[PACKET_TYPE_REPLY], self.pid, packets_cnt))
                        
                self.ser.write(packet)

                if (packets_cnt <= 0):
                    self.send_state = SEND_PACKET_STATE_QUIT

                self.send_state = SEND_PACKET_STATE_WAIT_ACK
                continue

            if (self.send_state == SEND_PACKET_STATE_WAIT_ACK):
                print("station go into state SEND_PACKET_STATE_WAIT_ACK")
                packet = self.ser.read(PACKET_LEN)
                if (len(packet) < PACKET_LEN):
                    continue

                board_id, packet_type, pid, packet_seq = struct.unpack("4B", packet[: PACKET_OFFSET_SEQ+1])
                print("station receive ACK, packe len %d, board_id %d, packet_type %d, pid %d, packet_seq %d"
                        %(len(packet), board_id, packet_type, pid, packet_seq))
                        
                if (board_id != self.my_board_id or packet_type != PACKET_TYPE_ACK):
                    print ("received  packet len %d, board_id %d, packet type %d, expected board_id %d, packet_type: %s"
                            %(len(packet), board_id, packet_type, self.my_baord_id, self.pt_str[PACKET_TYPE_ACK]))
                            
                    self.send_state = SEND_PACKET_STATE_QUIT
                    continue

                print ("send packet cnt %d" %(send_packet_cnt))
                if (send_packet_cnt == packets_cnt):
                    print("station send %d packets done" %(send_packet_cnt))
                    
                    self.send_packet_done = True
                    self.send_state = SEND_PACKET_STATE_QUIT
                    continue

                self.send_state = SEND_PACKET_STATE_SEND
                continue

            if (self.send_state == SEND_PACKET_STATE_SEND):
                print("sttion go into state SEND_PACKET_STATE_SEND")
                packet = struct.pack("4B", self.target_board_id, PACKET_TYPE_DATA, self.pid, _packet_seq)
                if (send_packet_cnt != packets_cnt - 1):
                    crc_str = CRC_LEN * '\0'
                    msg = sendfile.read(PACKET_DATA_LEN)
                    #print ("#station sen len %d, msg:%s" %(len(msg), msg))
                    packet =  packet + msg + crc_str
                else:
                    packet = packet + sendfile.read(last_packet_size) + (PACKET_LEN - len(packet) - last_packet_size) * '\0'

                self.ser.write(packet)

                print("station send data, len %d, target_board_id %d, type %s, pid %d, seq %d"
                        %(len(packet), self.target_board_id, self.pt_str[PACKET_TYPE_DATA], pid, _packet_seq))
                        
                self.total_send_bytes += len(packet)
                send_packet_cnt += 1
                _packet_seq += 1

                self.send_state = SEND_PACKET_STATE_WAIT_ACK
                continue

            if (self.send_state == SEND_PACKET_STATE_QUIT):
                print("station go into state SEND_PACKET_STATE_QUIT")
                print ("station_send_packet thread exit...")
                return False

    def send_bytes(self):
        return self.total_send_bytes

    def send_packet_is_done(self):
        return self.send_packet_done

       

def receive_packet_thread(xock):
    return xock.receive_packet()

def request_packet(port, target_board_id, board_id, pid):
    
    if not port:
        port = "/dev/ttyUSB0"
        
    xock = xocket(port, baudrate = 115200, target_board_id = target_board_id, pid = pid,
                  my_board_id = board_id)
    
    thread.start_new_thread(receive_packet_thread, (xock,))
    cur_time = time.time()
    while time.time() < cur_time + 20.0:
        time.sleep(0.01)
        if (xock.receive_packet_is_done()):
            time_interval = time.time() - cur_time
            received_bytes = xock.received_bytes()
            print("receive %d bytes in %.3f" %(received_bytes, time_interval))
            print("speed: %.2fKB/s" %((received_bytes/1000)/time_interval))
            print("\r\nreceive packet done ok\r\n")
            return True

    print ("recevie packet timeout...")
    return False
    
 
def send_packet_thread(xock, file_name, pid):
    return xock.send_packet(file_name, pid)

def reply_packet(port, target_board_id, board_id, send_file_name):

    if not port:
        port = "/dev/ttyUSB0"
        
    xock = xocket(dev = port, baudrate = 115200, target_board_id = target_board_id,
                  my_board_id = board_id)

    while True:
        print("waiting for request packet")
        
        packet = xock.ser.read(PACKET_LEN)
        if (len(packet) < PACKET_LEN):
            continue

        if (xock.is_request_packet(packet)):
            board_id, packet_type, pid = struct.unpack("3B", packet[:PACKET_OFFSET_PID+1])
            thread.start_new_thread(send_packet_thread, (xock, send_file_name, pid)) 
            cur_time = time.time()
            wait_delay = 10.0
            while time.time() < cur_time + wait_delay:
                time.sleep(0.01)
                if (xock.send_packet_is_done()):
                    time_interval = time.time() - cur_time
                    send_bytes = xock.send_bytes()
                    print("send %d bytes in %.3f" %(send_bytes, time_interval))
                    print("speed: %.2fKB/s" %((send_bytes/1000)/time_interval))
                    print("\r\n send packet done ok\r\n")
                    break
                    
            if (time.time <= cur_time + wait_delay):
                print ("send packet timeout...")
    



