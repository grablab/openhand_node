from lib_robotis_mod import *
from registerDict import *

from hands import *

import IPython

dyn = USB2Dynamixel_Device()
s1 = Robotis_Servo_X(dyn,1,'XM')
s2 = Robotis_Servo_X(dyn,2,'XM')

'''
Writes values to single type of register in multiple motors, i.e. only change only thing about all the motors at a time e.g. position, velocity, etc.

servos: list of servos you want to change
positions: list of values you want to change in the register (values should be in same order as servos list)
address = which register you would like to change. See registerDict.py for more info
reg_size = size of the register you are trying to change. See registerDict.py for more info

For more information, see http://emanual.robotis.com/docs/en/dxl/protocol2/
'''
def sync_write(servos, positions, address = registerDict.X_Series["ADDR_GOAL_POSITION"], reg_size = 4 ):

    header = [ 0xFF, 0xFF, 0xFD, 0x00 ] + [0xFE] #Add the length after this, but we need to solve for that at the end

    INST = 0x83 #sync_write
    P1, P2 = DXL_LOBYTE(address), DXL_HIBYTE(address)
    P3,P4 = reg_size, 0x00 #This will be the same for all writes because we send 4 bytes

    total_packet_length = registerDict.DXL_MAKEWORD(P3, P4)*len(servos) +len(servos) + 7 #size of packets *number servos +servo_ids + INST+P1+P2+P3+P4+CRC1+CRC2
    LEN1, LEN2 = DXL_LOBYTE(total_packet_length), DXL_HIBYTE(total_packet_length)

    servo_idx = 0
    msg = []
    for servo in servos:
        msg=msg+[servo.servo_id]
        #now find the proper new encoder location
        n = min( max( positions[servo_idx], 0 ), servo.settings['max_encoder'] )
        msg=msg+[DXL_LOBYTE(n), DXL_HIBYTE(n),0,0]
        servo_idx+=1

    msg = header + [LEN1,LEN2, INST,P1,P2,P3,P4] + msg
    crc = registerDict.updateCRC(0, msg, total_packet_length+5)  # 2: CRC16 #APPARENTLY YOU DON'T SUBTRACT 2 ANYMORE??

    msg = msg+ [DXL_LOBYTE(crc)] + [DXL_HIBYTE(crc)]

    return send_msg(dyn, servos, msg)


'''
Reads values to single type of register in multiple motors, i.e. only reads only thing about all the motors at a time e.g. position, velocity, etc.

servos: list of servos you want to read
address = which register you would like to read. See registerDict.py for more info
reg_size = size of the register you are trying to read. See registerDict.py for more info

For more information, see http://emanual.robotis.com/docs/en/dxl/protocol2/
'''
def sync_read(servos, address = registerDict.X_Series["ADDR_PRESENT_POSITION"], reg_size = 4 ):

    header = [ 0xFF, 0xFF, 0xFD, 0x00 ] + [0xFE]

    msg_len = 5+len(servos)+2
    LEN1,LEN2 = DXL_LOBYTE(msg_len), DXL_HIBYTE(msg_len)

    INST = 0x82
    P1, P2 = DXL_LOBYTE(address), DXL_HIBYTE(address)
    P3,P4 = reg_size, 0x00 #This case if you are reading positions/loads/ etc. Need to change according to register size

    msg = []
    for servo in servos:
        msg = msg +[servo.servo_id]

    msg = header + [LEN1,LEN2, INST,P1,P2,P3,P4] + msg
    crc = registerDict.updateCRC(0, msg, len(msg))

    msg = msg + [DXL_LOBYTE(crc)] + [DXL_HIBYTE(crc)]

    return send_msg(dyn, servos, msg, reg_size)

'''
Sends the message crafted in sync_read or sync_write to the R2D2 bus

dyn: R2D2 object
servos: list of servos you want to send to
msg: message crafted in previous functions
reg_size: reg_size is only required for sync_read and can be anything for sync_write
'''
def send_msg(dyn, servos, msg, reg_size = 0):

    dyn.acq_mutex()
    try:
        out = ''
        for m in msg:
            out += chr(m)
        dyn.send_serial( out )
        data = receive_reply(servos, reg_size)
    except Exception as inst:
        dyn.rel_mutex()
        raise RuntimeError(repr(str(inst)))
    dyn.rel_mutex()

    return data

'''
Listens for the message back from the motors. Returns None for sync_write command. Commands desired values for sync_read

servos: list of servos you want to send to
reg_size: reg_size is only required for sync_read and can be anything for sync_write
'''
def receive_reply(servos, reg_size):
    data = []
    tmp = []
    for servo in servos:
        single_read = dyn.read_serial( 11 + reg_size )	#from pydynamixel: possible that these contain empty bytes
        if len(single_read) == 0: #this is only happens in the sync_write case
            return None
        if ord(single_read[4]) != servo.servo_id:
            raise RuntimeError('lib_robotis: Incorrect servo ID received')
        else:
            if reg_size == 4:
                tmp = [ord(single_read[12]), ord(single_read[11]), ord(single_read[10]), ord(single_read[9])]
                a = DXL_MAKEWORD(tmp[1], tmp[0])
                b = DXL_MAKEWORD(tmp[3], tmp[2])
                data.append(DXL_MAKEDWORD(b,a))
            else:
                data.append(DXL_MAKEDWORD(ord(single_read[9]), ord(single_read[10])))
    return data


servos = [s1]

#IPython.embed()
ret = sync_write(servos, [3000])
ret = sync_read(servos, registerDict.X_Series[ "ADDR_MAX_VOLTAGE_LIMIT"], 2 )
print ret
