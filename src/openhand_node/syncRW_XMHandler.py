from lib_robotis_mod import *
from registerDict import *

from hands import *

import IPython

class syncRW_XMHandler():

    def __init__(self, hand):
        self.hand = hand
        self.servos = self.hand.servos
        self.dyn = self.hand.dyn
        self.max_encoder = self.hand.servos[0].settings['max_encoder']

    '''
    Function used to move the XM motors via the openhandNode
    '''
    def moveXMmotors(self, pos_to_move, address = registerDict.X_Series["ADDR_GOAL_POSITION"], reg_size = 4): #pos_move should be a list with values to move to
        enc_positions = []
        pos_to_move = list(pos_to_move)
        for i in range(len(pos_to_move)):
            amnt = min(max(pos_to_move[i],0.),1.0)

            servo = self.servos[i]
            if self.hand.motorDir[i]>0:	#normal case
                enc_positions.append(int(servo.settings["max_encoder"]*(self.hand.motorMin[i] + amnt*(self.hand.motorMax[i]-self.hand.motorMin[i]))))
            else:				#reverse
                enc_positions.append(int(servo.settings["max_encoder"]*(self.hand.motorMax[i] - amnt*(self.hand.motorMax[i]-self.hand.motorMin[i]))))
        return self.sync_write(enc_positions)

	'''
	Writes values to single type of register in multiple motors, i.e. only change only thing about all the motors at a time e.g. position, velocity, etc.

	servos: list of servos you want to change
	positions: list of values you want to change in the register (values should be in same order as servos list)
	address = which register you would like to change. See registerDict.py for more info
	reg_size = size of the register you are trying to change. See registerDict.py for more info

	For more information, see http://emanual.robotis.com/docs/en/dxl/protocol2/
	'''
    def sync_write(self, positions, address = registerDict.X_Series["ADDR_GOAL_POSITION"], reg_size = 4 ):

        header = [ 0xFF, 0xFF, 0xFD, 0x00 ] + [0xFE] #Add the length after this, but we need to solve for that at the end

        INST = 0x83 #sync_write
        P1, P2 = DXL_LOBYTE(address), DXL_HIBYTE(address)
        P3,P4 = reg_size, 0x00 #This will be the same for all writes because we send 4 bytes

        total_packet_length = registerDict.DXL_MAKEWORD(P3, P4)*len(self.servos) +len(self.servos) + 7 #size of packets *number servos +servo_ids + INST+P1+P2+P3+P4+CRC1+CRC2
        LEN1, LEN2 = DXL_LOBYTE(total_packet_length), DXL_HIBYTE(total_packet_length)

        servo_idx = 0
        msg = []
        for servo in self.servos:
            msg=msg+[servo.servo_id]
            #now find the proper new encoder location
            n = min( max( positions[servo_idx], 0 ), self.max_encoder )
            msg=msg+[DXL_LOBYTE(n), DXL_HIBYTE(n),0,0]
            servo_idx+=1

        msg = header + [LEN1,LEN2, INST,P1,P2,P3,P4] + msg
        crc = registerDict.updateCRC(0, msg, total_packet_length+5)  # 2: CRC16 #APPARENTLY YOU DON'T SUBTRACT 2 ANYMORE??

        msg = msg+ [DXL_LOBYTE(crc)] + [DXL_HIBYTE(crc)]

        return self.send_msg(msg)


    '''
    Reads values to single type of register in multiple motors, i.e. only reads only thing about all the motors at a time e.g. position, velocity, etc.

    servos: list of servos you want to read
    address = which register you would like to read. See registerDict.py for more info
    reg_size = size of the register you are trying to read. See registerDict.py for more info

    For more information, see http://emanual.robotis.com/docs/en/dxl/protocol2/
    '''
    def sync_read(self, address = registerDict.X_Series["ADDR_PRESENT_POSITION"], reg_size = 4 ):

        header = [ 0xFF, 0xFF, 0xFD, 0x00 ] + [0xFE]

        msg_len = 5+len(self.servos)+2
        LEN1,LEN2 = DXL_LOBYTE(msg_len), DXL_HIBYTE(msg_len)

        INST = 0x82
        P1, P2 = DXL_LOBYTE(address), DXL_HIBYTE(address)
        P3,P4 = reg_size, 0x00 #This case if you are reading positions/loads/ etc. Need to change according to register size

        msg = []
        for servo in self.servos:
            msg = msg +[servo.servo_id]

        msg = header + [LEN1,LEN2, INST,P1,P2,P3,P4] + msg
        crc = registerDict.updateCRC(0, msg, len(msg))

        msg = msg + [DXL_LOBYTE(crc)] + [DXL_HIBYTE(crc)]

        return self.send_msg( msg, reg_size)

    '''
    Sends the message crafted in sync_read or sync_write to the R2D2 bus

    dyn: R2D2 object
    servos: list of servos you want to send to
    msg: message crafted in previous functions
    reg_size: reg_size is only required for sync_read and can be anything for sync_write
    '''
    def send_msg(self, msg, reg_size = 0):

        self.dyn.acq_mutex()
        try:
            out = ''
            for m in msg:
                out += chr(m)
            self.dyn.send_serial( out )
            data = self.receive_reply( reg_size)
        except Exception as inst:
            self.dyn.rel_mutex()
            raise RuntimeError(repr(str(inst)))
        self.dyn.rel_mutex()

        return data

    '''
    Listens for the message back from the motors. Returns None for sync_write command. Commands desired values for sync_read

    servos: list of servos you want to send to
    reg_size: reg_size is only required for sync_read and can be anything for sync_write
    '''
    def receive_reply(self, reg_size):
        data = []
        tmp = []
        for servo in self.servos:
            single_read = self.dyn.read_serial( 11 + reg_size )	#from pydynamixel: possible that these contain empty bytes
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
