#!/usr/bin/python

#Andrew Morgan
#Yale University
#Updated 09/2018


import openhand_node.hands as hands
from openhand_node.srv import *
import rospy

#Communication class for Openhand
class OpenHandNode():
	hand = None

	#Initialize to have a hand object
	def __init__(self,hand, series):
		self.series = series
		self.hand = hand
		self.sRefs = [0.]*len(self.hand.servos)
		for i in xrange(len(self.hand.servos)):
			motor_pos,motor_encoder = self.hand.readMotor(i)
			self.sRefs[i] = motor_pos

		self.is_pos_control = [True] * len(self.hand.servos)
		#initialize service handlers:
		rospy.Service('/openhand_node/move_servos',MoveServos,self.MoveServosCallback)
		rospy.Service('/openhand_node/torque_servos',TorqueServos,self.TorqueServosCallback)
		rospy.Service('/openhand_node/read_servos',ReadServos,self.ReadServosCallback)
		rospy.Service('/openhand_node/hold_servos',HoldServos,self.HoldServosCallback)
		rospy.Service('/openhand_node/read_current',ReadCurrent,self.ReadCurrentCallback)
		rospy.Service('/openhand_node/read_load',ReadLoad,self.ReadLoadCallback)
		rospy.Service('/openhand_node/read_temperature',ReadTemperature,self.ReadTemperatureCallback)
		rospy.Service('/openhand_node/set_operating_mode',OperatingMode,self.SetOperatingModeCallback)
		rospy.spin()		#blocking

	#This should work in theory, but dynamixels have a mind of their own.
	def HoldServosCallback(self,req):
		self.hand.preventAllLoadErrors(req.offset_scale);
		resp = HoldServosResponse()
		resp.err = 0
		return resp

	def MoveServosCallback(self,req):
		pos_to_move = req.pos
		resp = MoveServosResponse()
		resp.err = 0

		#If only one value was sent for all motors, move them all to that location
		if len(self.hand.servos)>1 and len(pos_to_move)==1:
			pos_to_move = len(self.hand.servos) * [pos_to_move[0]]

		#Move each of the motors as desired
		for i in xrange(len(pos_to_move)):
			if i < len(self.hand.servos): #just to be safe
				self.hand.moveMotor(i,pos_to_move[i])
				self.sRefs[i] = pos_to_move[i]	#store the reference value sent through ROS
			else:
				resp.err = 1
		return resp

	def SetOperatingModeCallback(self, req):
		#If we send in True for position control, we will be in pos_control
		#OW, false will be for Torque control
		resp = OperatingModeResponse()
		resp.err = 0

		#if self.series != 'XM' or self.series != 'MX':
		#	rospy.logwarn('[ERR] Torque control not implemented for this series of actuator')
		#	resp.err = 1

		#This method really only works for the MX and XM Motors
		is_pos_control = req.pos_control
		servos_to_update = req.servo_indices

		for i in range(len(servos_to_update)):
			if is_pos_control[i] == False:
				self.hand.servos[servos_to_update[i]].enable_torque_mode()
			else: #Then enable position control
				self.hand.servos[servos_to_update[i]].disable_torque_mode()


		return resp


	def TorqueServosCallback(self,req):
		#This typically only works is the finger is already in torque control mode
		resp = TorqueServosResponse()
		resp.err = 0

		torq = req.torq
		servos_to_update = req.servo_indices

		if self.series != 'XM' or self.series != 'MX':
			rospy.logwarn('[ERR] Torque control not implemented for this series of actuator')
			resp.err = 1

		for i in range(len(servos_to_update)):
			self.hand.servos[servos_to_update[i]].apply_torque(torq[i])

		return resp

	def ReadServosCallback(self,req):
		pos = [0.]*len(self.hand.servos)
		ref = self.sRefs
		enc = [0]*len(self.hand.servos)
		for i in xrange(len(self.hand.servos)):
			sp,se = self.hand.readMotor(i)
			pos[i] = sp
			enc[i] = se
		resp = ReadServosResponse()
		resp.pos = pos
		resp.enc = enc
		resp.ref = ref
		return resp

	def ReadCurrentCallback(self,req):
		curr = [0.]*len(self.hand.servos)
		for i in xrange(len(self.hand.servos)):
			curr[i] = self.hand.servos[i].read_current()
		resp = ReadCurrentResponse()
		resp.curr = curr
		return resp

	def ReadLoadCallback(self,req):
		load = [0.]*len(self.hand.servos)
		for i in xrange(len(self.hand.servos)):
			load[i] = self.hand.servos[i].read_load()
		resp = ReadLoadResponse()
		resp.load = list(load)
		return resp

	def ReadTemperatureCallback(self,req):
		temp = [0.]*len(self.hand.servos)
		for i in xrange(len(self.hand.servos)):
			temp[i] = self.hand.servos[i].read_temperature()
		resp = ReadTemperatureResponse()
		resp.temp = temp
		return resp


if __name__=="__main__":
	Hand = None
	#initialize ros node elements: (can't read parameters until node is initialized)
	rospy.init_node("OpenHandNode")

	#query launch parameters for hand model, throw exception if invalid
	if rospy.has_param('~openhand_type'):
		model = rospy.get_param('~openhand_type')
		servo_ids = rospy.get_param('~servo_ids')
		port = rospy.get_param('~servo_port')
		series = rospy.get_param('~servo_type')
		direction = rospy.get_param('~direction')
		motor_offset = rospy.get_param('~motor_offset',[0.])
		if motor_offset == [0.]:
			motor_offset = [0.] * len(servo_ids)

		rospy.loginfo("Initializing "+model+" with servos "+repr(servo_ids)+", port "+port+", Dynamixel model "+series)

		if (model=='Model_T'):		#hand selection from yaml file
			if len(servo_ids)!=1:
				rospy.logerr("ERR: expecting 1 servo id, got "+len(servo_ids))
			try:
				Hand = hands.Model_T(port,servo_ids[0],series, motor_offset[0])
			except:
				rospy.logerr("ERR: Model T failed to initialize (openhandNode.py)")

		elif (model=='Model_T42' or model=='t42'):
			if len(servo_ids)!=2:
				rospy.logerr("ERR: expecting 2 servo ids, got "+repr(len(servo_ids)))
			try:
				Hand = hands.Model_T42(port,servo_ids[0],servo_ids[1],series, motor_offset[0],motor_offset[1])
			except:
				rospy.logerr("ERR: Model T42 failed to initialize (openhandNode.py)")

		elif (model=='Model_O'):
			if len(servo_ids)!=4:
				rospy.logerr("ERR: expecting 4 servo ids, got "+repr(len(servo_ids)))
			try:
				Hand = hands.Model_O(port,servo_ids[0],servo_ids[1],servo_ids[2],servo_ids[3],series,motor_offset[0],motor_offset[1],motor_offset[2],motor_offset[3] )
			except:
				rospy.logerr("ERR: Model O failed to initialize (openhandNode.py)")


	#start node if everything is initialized correctly
	if Hand is None:
		rospy.logerr("ERR: Cannot Initialize the openhand node (openhandNode.py). Please check to ensure your parameters are correct!")
	else:
		OpenHandNode(Hand, series)
