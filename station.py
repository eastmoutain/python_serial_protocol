
import time
import serial
import thread
import struct
import os

PACKET_OFFSET_ID = 0
PACKET_OFFSET_TYPE = 1
PACKET_OFFSET_PID = 2
PACKET_OFFSET_SEQ = 3
PACKET_OFFSET_CNT = 3
#PACKET_OFFSET_CRC = 254
#PACKET_LEN  = 256
PACKET_OFFSET_CRC = 1022
PACKET_LEN  = 1024
CRC_LEN = 2
#PACKET_DATA_LEN = 250
PACKET_DATA_LEN = 1018

PACKET_TYPE_REQ = 0 # request data packet
PACKET_TYPE_REPLY = 1 #reply packet
PACKET_TYPE_ACK = 2 # acknowledge pacekt
PACKET_TYPE_DATA = 3 # data packet
PACKET_TYPE_QUIT = 4 # quit transfer

GET_PACKET_STATE_REQ = 0
GET_PACKET_STATE_WAIT_REPLY = 1
GET_PACKET_STATE_RECEIVE_DATA = 2
GET_PACKET_STATE_SEND_ACK = 3
GET_PACKET_STATE_QUIT = 4

SEND_PACKET_STATE_REPLY = 0
SEND_PACKET_STATE_SEND = 1
SEND_PACKET_STATE_WAIT_ACK = 2
SEND_PACKET_STATE_TIMEOUT = 3
SEND_PACKET_STATE_QUIT = 4


FLYER_BOARD_ID = 0
STATION_BOARD_ID = 2

GLOBAL_PID = 0

send_packet_done = False
receive_packet_done = False

total_send_bytes = 0
total_receive_bytes = 0

po_str = {}
pt_str = {}
gps_str = {}
sps_str = {}

def setup_packet_offset_dict():
    po_str[PACKET_OFFSET_ID] = "PACKET_OFFSET_ID"
    po_str[PACKET_OFFSET_TYPE] = "PACKET_OFFSET_TYPE"
    po_str[PACKET_OFFSET_PID] = "PACKET_OFFSET_PID"
    po_str[PACKET_OFFSET_SEQ] = "PACKET_OFFSET_SEQ"
    po_str[PACKET_OFFSET_CRC] = "PACKET_OFFSET_CRC"

def setup_packet_type_dict():
    pt_str[PACKET_TYPE_REQ] = "PACKET_TYPE_REQ"
    pt_str[PACKET_TYPE_REPLY] = "PACKET_TYPE_REPLY"
    pt_str[PACKET_TYPE_ACK] = "PACKET_TYPE_ACK"
    pt_str[PACKET_TYPE_DATA] = "PACKET_TYPE_DATA"
    pt_str[PACKET_TYPE_QUIT] = "PACKET_TYPE_QUIT"

def setup_get_packet_state_dict():
    gps_str[GET_PACKET_STATE_REQ] = "GET_PACKET_STATE_REQ"
    gps_str[GET_PACKET_STATE_WAIT_REPLY] = "GET_PACKET_STATE_WAIT_REPLY"
    gps_str[GET_PACKET_STATE_RECEIVE_DATA] = "GET_PACKET_STATE_RECEIVE_DATA"
    gps_str[GET_PACKET_STATE_SEND_ACK] = "GET_PACKET_STATE_SEND_ACK"
    gps_str[GET_PACKET_STATE_QUIT] = "GET_PACKET_STATE_QUIT"

def setup_send_packet_state_dict():
    sps_str[SEND_PACKET_STATE_REPLY] = "SEND_PACKET_STATE_REPLY"
    sps_str[SEND_PACKET_STATE_SEND] = "SEND_PACKET_STATE_SEND"
    sps_str[SEND_PACKET_STATE_WAIT_ACK] = "SEND_PACKET_STATE_WAIT_ACK"
    sps_str[SEND_PACKET_STATE_TIMEOUT] = "SEND_PACKET_STATE_TIMEOUT"
    sps_str[SEND_PACKET_STATE_QUIT] = "SEND_PACKET_STATE_QUIT"


def save_packet(msg, target_board_id):
    try:
        file = open("msg.txt", 'a+')
        file.write(msg)
    finally:
        file.close()

def flyer_get_packet(media, target_board_id):
    print("flyer get packet thread running")
    state = GET_PACKET_STATE_REQ
    packet_total_cnt = 0
    packet_cnt = 0

    _pid = 0
    _packet_seq = 0

    pid = 0
    packet_seq = 0

    cur_time = time.time()

    while True:
        if (state == GET_PACKET_STATE_REQ):
            print("flyer go into state GET_PACKET_STATE_REQ ")
            hdr = struct.pack('4B', target_board_id, PACKET_TYPE_REQ, GLOBAL_PID, 0)

            packet = hdr + (PACKET_DATA_LEN + CRC_LEN) * '\0'
            print ("flyer wirte req packet len %d, target_board_id %d, type %s, pid %d,"
                    % (len(packet), target_board_id, pt_str[PACKET_TYPE_REQ], GLOBAL_PID))
            print ("write start...")
            media.write(packet)
            print("write done")
            state = GET_PACKET_STATE_WAIT_REPLY
            cur_time = time.time()
            continue

        if (state == GET_PACKET_STATE_WAIT_REPLY):
            print ("flyer go into state GET_PACKET_STATE_WAIT_REPLY")
            if (cur_time + 10 < time.time()):
                print ("flyer wait reply timeout 10sec... flyer goto state GET_PACKET_STATE_QUIT")
                state = GET_PACKET_STATE_QUIT
                continue

            packet = media.read(PACKET_LEN)
            if (len(packet) < PACKET_LEN):
                continue

            '''if (crc16_check(packet) == False):
                state = GET_PACKET_STATE_QUIT
                continue
            '''

            board_id, packet_type = struct.unpack('2B', packet[:PACKET_OFFSET_TYPE+1])
            if (board_id != FLYER_BOARD_ID or packet_type != PACKET_TYPE_REPLY):
                print("error! received packet len %d, board id: %d, expect id: %d " %(len(packet), board_id, FLYER_BOARD_ID))
                print ("type: %d, expected type: %s" %(packet_type, pt_str[PACKET_TYPE_REPLY]))
                state = GET_PACKET_STATE_QUIT
                continue
            _pid, packet_total_cnt = struct.unpack('2B', packet[PACKET_OFFSET_PID:PACKET_OFFSET_CNT+1])
            if (packet_total_cnt <= 0):
                print("receive reply from station, packet total cnt %d" %(packet_total_cnt))
                state = GET_PACKET_STATE_QUIT
                continue
            print("receive reply from station, board_id: %d, packet_type: %s, _pid: %d, packet_total_cnt %d"
                    %(board_id, pt_str[packet_type], _pid, packet_total_cnt))
            state = GET_PACKET_STATE_SEND_ACK

        ''' receive data state '''
        if (state == GET_PACKET_STATE_RECEIVE_DATA):
            print ("flyer go into state GET_PACKET_STATE_RECEIVE_DATA")
            packet = media.read(PACKET_LEN)

            if (len(packet) < 4):
                if (cur_time + 1 < time.time()):
                    print ("flyer receive date timeout 1sec...")
                continue
            '''if (crc16_check(packet) == False):
                print ("crc error at packet %d" % (packet_cnt))
                state = GET_PACKET_STATE_QUIT
                continue
            '''

            board_id, packet_type = struct.unpack('2B', packet[:PACKET_OFFSET_TYPE+1])
            if (board_id != FLYER_BOARD_ID or packet_type != PACKET_TYPE_DATA):
                print("rcving error! received packet id: %d, expect id: %d " %(board_id, FLYER_BOARD_ID))
                print ("packet type: %d, expected type: %d" %(packet_type, PACKET_TYPE_DATA))
                state = GET_PACKET_STATE_QUIT
                continue

            pid, packet_seq = struct.unpack('2B', packet[PACKET_OFFSET_PID:PACKET_OFFSET_SEQ+1])
            if (pid != _pid or packet_seq != _packet_seq):
                print("rcving error! packet pid: %d, seq: %d, expected pid: %d, seq %d" %(pid, packet_seq, _pid, _packet_seq))
                state = GET_PACKET_STATE_QUIT
                continue

            print("receive data from station, board_id: %d, packet_type: %s, pid: %d, packet_seq %d"
                    %(board_id, pt_str[packet_type], _pid, packet_seq))

            save_packet(packet[PACKET_OFFSET_SEQ+1: PACKET_LEN - CRC_LEN], target_board_id)
            _packet_seq += 1
            state = GET_PACKET_STATE_SEND_ACK


        if (state == GET_PACKET_STATE_SEND_ACK):
            print ("flyer go into state GET_PACKET_STATE_SEND_ACK ")
            packet  = struct.pack("4B", target_board_id, PACKET_TYPE_ACK, _pid, packet_seq)
            packet = packet + (PACKET_DATA_LEN + CRC_LEN) * '\0'
            print ("media wirte ACK packet len %d, target_board_id %d, packet_type:%s, _pid %d, _packet_seq %d,"
                    % (len(packet), target_board_id,  pt_str[PACKET_TYPE_ACK], _pid, packet_seq))

            media.write(packet)

            if (_packet_seq == packet_total_cnt):
                receive_packet_done = True
                return True
            else:
                state = GET_PACKET_STATE_RECEIVE_DATA
                continue


        ''' end the transfer '''
        if (state == GET_PACKET_STATE_QUIT):
            print ("flyer go into GEP_PACKET_STATE_QUIT")
            receive_packet_done = False
            print ("flyer_get_packet thread exit...")
            return False


''' request one packet from station '''
def flyer_receive_packet_from_station(media, id):
    thread.start_new_thread(flyer_get_packet, (media, id,))
    cur_time = time.time()
    while time.time() < cur_time + 20.0:
        time.sleep(0.01)
        if (receive_packet_done):
            print("\r\nflyer receive packet done ok\r\n")
            return True

    print ("flyer_recevie_packet_from_station: recevie packet timeout...")

    return False



def is_request_packet(packet):
    board_id, packet_type = struct.unpack("2B", packet[:PACKET_OFFSET_TYPE+1])
    if (board_id != STATION_BOARD_ID or packet_type != PACKET_TYPE_REQ):
        print ("wrong packet,len %d, board id %d, type %d, expect board_id %d, packet_tyep %s" %(len(packet), board_id, packet_type, STATION_BOARD_ID, pt_str[PACKET_TYPE_REQ]))
        return False
    else:
        print("station receive one request packet, board_id %d, packet_type %s"
                %(board_id, pt_str[packet_type]))
        return True

def station_send_packet(media, target_board_id, pid):
    global send_packet_done
    global total_send_bytes
    state = SEND_PACKET_STATE_REPLY
    sendfile = open("sendfile.txt", 'rb')
    file_size = os.path.getsize("sendfile.txt")
    packets_cnt = (file_size + PACKET_DATA_LEN - 1) / PACKET_DATA_LEN
    send_packet_cnt = 0
    last_packet_size = file_size % PACKET_DATA_LEN
    print("file size %d, packets_cnt %d, last_packet_size %d" %(file_size, packets_cnt, last_packet_size))
    _packet_seq = 0

    while True:
        if (state == SEND_PACKET_STATE_REPLY):
            print("station go inot state SEND_PACKET_STATE_REPLY")
            packet = struct.pack("4B", target_board_id, PACKET_TYPE_REPLY, pid, packets_cnt)
            packet = packet + (PACKET_DATA_LEN + CRC_LEN) * '\0'
            print ("station send packet len %d, target_board_id %d, type %s, pid %d, packets_cnt %d"
                    %(len(packet), target_board_id, pt_str[PACKET_TYPE_REPLY], pid, packets_cnt))
            media.write(packet)

            if (packets_cnt <= 0):
                state = SEND_PACKET_STATE_QUIT

            state = SEND_PACKET_STATE_WAIT_ACK
            continue

        if (state == SEND_PACKET_STATE_WAIT_ACK):
            print("station go into state SEND_PACKET_STATE_WAIT_ACK")
            packet = media.read(PACKET_LEN)
            if (len(packet) < PACKET_DATA_LEN):
                continue

            board_id, packet_type = struct.unpack("2B", packet[: PACKET_OFFSET_TYPE+1])
            print("station receive ACK, packe len %d, board_id %d, packet_type %d"
                    %(len(packet), board_id, packet_type))
            if (board_id != STATION_BOARD_ID or packet_type != PACKET_TYPE_ACK):
                print ("received  packet len %d, board_id %d, packet type %d, expected board_id %d, packet_type: %s"
                        %(len(packet), board_id, packet_type, STATION_BOARD_ID, pt_str[PACKET_TYPE_ACK]))
                state = SEND_PACKET_STATE_QUIT
                continue

            if (send_packet_cnt == packets_cnt):
                print("station send %d packets done" %(send_packet_cnt))
                send_packet_done = True
                state = SEND_PACKET_STATE_QUIT
                continue

            state = SEND_PACKET_STATE_SEND
            continue

        if (state == SEND_PACKET_STATE_SEND):
            print("sttion go into state SEND_PACKET_STATE_SEND")
            packet = struct.pack("4B", target_board_id, PACKET_TYPE_DATA, pid, _packet_seq)
            if (send_packet_cnt != packets_cnt - 1):
                crc_str = CRC_LEN * '\0'
                msg = sendfile.read(PACKET_DATA_LEN)
                #print ("#station sen len %d, msg:%s" %(len(msg), msg))
                packet =  packet + msg + crc_str
            else:
                packet = packet + sendfile.read(last_packet_size) + (PACKET_LEN - len(packet) - last_packet_size) * '\0'

            media.write(packet)

            print("station send data, len %d, target_board_id %d, type %s, pid %d, seq %d"
                    %(len(packet), target_board_id, pt_str[PACKET_TYPE_DATA], pid, _packet_seq))
            total_send_bytes += len(packet)
            send_packet_cnt += 1
            _packet_seq += 1

            state = SEND_PACKET_STATE_WAIT_ACK
            continue

        if (state == SEND_PACKET_STATE_QUIT):
            print("station go into state SEND_PACKET_STATE_QUIT")
            print ("station_send_packet thread exit...")
            return False


''' detect whether need to send data to flyer '''
def station_run(media, target_board_id):
    while True:
        packet = media.read(PACKET_LEN)
        if (len(packet) <= 0):
            continue

        if (is_request_packet(packet)):
            board_id, packet_type, pid = struct.unpack("3B", packet[:PACKET_OFFSET_PID+1])
            thread.start_new_thread(station_send_packet, (media, FLYER_BOARD_ID, pid))
            cur_time = time.time()
            while time.time() < cur_time + 25.0:
                time.sleep(0.01)
                if (True == send_packet_done):
                    print("total send bytes %d in %.3fs"
                            %(total_send_bytes, time.time()-cur_time))
                    print("send speed: %.2fKB/s" %((total_send_bytes/1000)/(time.time() - cur_time)))

                    return True

            print ("stationc send packet failed, timeout 20 sec")
            return False
        else:
            print("receive one packet, but is not request packet")



''' set up serial port '''
def setup_serial():
    ser = serial.Serial(port = '/dev/ttyUSB0',
                       baudrate = 115200,
                       parity = serial.PARITY_NONE,
                       stopbits = serial.STOPBITS_ONE,
                       bytesize = serial.EIGHTBITS,
                       timeout = 0.3)
    return ser


setup_packet_offset_dict()
setup_packet_type_dict()
setup_get_packet_state_dict()
setup_send_packet_state_dict()

media = setup_serial()

while True:

    print ("station start detect packet from flyer")
    send_packet_done = False
    total_send_bytes = 0
    station_run(media, FLYER_BOARD_ID)

