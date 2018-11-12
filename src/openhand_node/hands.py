#!/usr/bin/python -i

#Created by multiple users
#Yale Grablab
#Updated 09/2019

from lib_robotis_mod import *
import time
import numpy as np	#for array handling
import scipy.io
import math
from decimal import Decimal

#Assumptions:
	#only dynamixel servos being used
	#either all RX or MX servos being used (no mixing)
	#different encoder limits for each type
		#motor limits and mov't designated in terms of proportion, not encoder value

act_out0 = []
act_out1 = []

class OpenHand():
	dyn = None		#USB2Dynamixel device
	port = None		#mounted port, /devttyUSB# in Linux, COM# in Windows
	servo_ids = []
	servos = []

	servo_speed = 1.0
	max_torque = 0.4	#Dynamixel suggests capping max torque at 0.4 of stall torque

	motorDir = []
	motorMin = []
	motorMax = []
	modes = []		#position control (True) or torque/pseudo-torque control (False)

	amnt_release = 0.0	#values [0,1.0] for degree of closing and opening
	amnt_close = 0.5

	pause = 0.3		#amount of time to wait for move commands and eeprom updates

	HAND_HEIGHT = 0.14	#hand height from base to palm (m) used for arm IK and approach vectors
	WRIST_OFFSET = -np.pi/3	#J5 offset (rad) to accommodate final orientation

	def __init__(self,port,servo_ids,series="RX"):
		self.port = port
		self.dyn = USB2Dynamixel_Device(port)	#always only one
		self.servo_ids = servo_ids
		num_servos = len(servo_ids)

		print "Initializing..."
		self.servos = []
		for servo_id in self.servo_ids:
			if series == "RX" or series =="MX":
				self.servos.append(Robotis_Servo(self.dyn,servo_id,series))
			else: #We will be using protocol 2 instead
				self.servos.append(Robotis_Servo_X(self.dyn,servo_id,series))
			print "Adding servo id "+repr(servo_id)
			time.sleep(self.pause)
		for servo in self.servos:
			servo.kill_cont_turn()		#make sure position mode limits are enabled

			time.sleep(self.pause)		#in case eeprom delay is what is causing the issues
			if series == "RX":
				servo.apply_speed(1)
			time.sleep(self.pause)
			servo.apply_max_torque(self.max_torque)
		self.modes = [True]*num_servos		#default assignment (shouldn't have servos in torque mode during normal operation)

		if len(self.motorDir)!=num_servos or len(self.motorMin)!=num_servos or len(self.motorMax)!=num_servos:
			print "[ERR] Servo number mismatch, resetting motor limits"
			self.motorDir = [1]*num_servos
			self.motorMin = [self.amnt_release]*num_servos
			self.motorMax = [self.amnt_close]*num_servos

		if num_servos == 4: #This is the model_O and we want to prevent gear shear
			for i in range(num_servos):
				enc = self.servos[i].read_encoder()
				if series == "RX":
					if (self.motorDir[i] ==1 and enc > 512) or (self.motorDir[i] == -1 and enc < 512):
						print "------------FAILSAFE-------------"
						print "Failsafe is incorporated to prevent gear shear in Model O"
						print "Motor encoder postion: ", enc
						input = raw_input("Your encoder position for motor index " + str(i) + " may cause the motor to move backwards and break gears. We recommend you resetting the fingers to prevent gear shear, proceed? [ENTER]")
					else:
						print "Motor directions not set..."
				#These would then be the MX and XM motors
				elif (self.motorDir[i] ==1 and enc > 2048) or (self.motorDir[i] == -1 and enc < 2048):
					print "------------FAILSAFE-------------"
					print "Failsafe is incorporated to prevent gear shear in Model O"
					print "Motor encoder postion: ", enc
					input = raw_input("As an XM Motor, we can automatically fix this issue for motor " + str(i) + ", proceed? [ENTER]")
					self.servos[i].enable_extended_position_control_mode()
					self.servos[i].move_to_encoder(self.servos[i].settings['max_encoder']+100)
					time.sleep(self.pause)
					self.servos[i].disable_extended_position_control_mode()
					time.sleep(self.pause)
					print "Fixed servo from ID: "+repr(servo_ids[i])
			#Finally, limit the abduction_torque if desired
			if self.abduction_limit !=1:
				self.servos[0].enable_current_position_control_mode(self.abduction_limit)

		time.sleep(self.pause)
		self.reset()
		print "Initialization Complete."

	def reset(self):	#returns everything to zeroed positions, different from release
		print "[ERR] reset() not implemented"
		return False

	def release(self):	#opens the finger components, doesn't necessarily move non-finger servos
		print "[ERR] release() not implemented\n"
		return False
	#close functions are normalized regardless of mode such that the operating range [0,1.0] makes sense
	def close(self,amnt=0.5):
		print "[ERR] close() not implemented\n"
		return False
	#tval: torque applied to servos, dpos: delta position overshoot beyond current
	def close_torque(self,tval=0.2,dpos=1.0):
		print "[ERR] close_torque() not implemented\n"
		return False
	#difference between close_torque should just be the particular servos that are actuated
	def _close_torques(self,tval=0.2,dpos=1.0,idxs=None):
		if idxs is None:	#effect on all servos if id's not specified
			idxs = range(len(self.servos))
		hp,he = self.readHand()
		for idx in idxs:
			self.torqueMotor(idx,tval,hp[idx]+dpos)
		i=0
		while i<15:		#some arbitrary limit on torque closing
			for idx in idxs:
				if not self.servos[idx].is_moving():
					amnt,enc = self.readMotor(idx)
					self.moveMotor(idx,amnt)	#exit out of torque mode
			time.sleep(self.pause)
			i+=1
		self.hold()


	#move servo according to amnt, not encoder value, scaled between designated min/max values
	def moveMotor(self,index,amnt):
		if amnt < 0. or amnt > 1.0:
			print "[WARNING] motion out of bounds, capping to [0,1]. Index: "+repr(index)+", Cmd:"+repr(amnt)
		amnt = min(max(amnt,0.),1.0)
		if (index < 0 or index >= len(self.servos)):
			print "[ERR] invalid motor index "+repr(index)
		else:
			servo = self.servos[index]
			#print "Now moving servo "+repr(index)
			if index == 0:
				offset = 0.0
			if index == 1:
				offset = 0.0

			global act_out0, act_out1, act0, act1, enc0, enc1

			if self.motorDir[index]>0:	#normal case

				act0, enc0 = self.readMotor(0)
				act_out0.append(act0)
				act1, enc1 = self.readMotor(1)
				act_out1.append(act1)

				servo.move_to_encoder(int(servo.settings["max_encoder"]*(self.motorMin[index] + amnt*(self.motorMax[index]-self.motorMin[index]))))
				act0, enc0 = self.readMotor(0)
				act_out0.append(act0)
				act1, enc1 = self.readMotor(1)
				act_out1.append(act1)

			else:				#reverse
				act0, enc0 = self.readMotor(0)
				act_out0.append(act0)
				act1, enc1 = self.readMotor(1)
				act_out1.append(act1)

				servo.move_to_encoder(int(servo.settings["max_encoder"]*(self.motorMax[index] - amnt*(self.motorMax[index]-self.motorMin[index]))))
				act0, enc0 = self.readMotor(0)
				act_out0.append(act0)
				act1, enc1 = self.readMotor(1)
				act_out1.append(act1)

			if not self.modes[index]:	#restore position-control mode if necessary - want to register new encoder target first before re-applying system torque
				self.modes[index] = True
				servo.apply_max_torque(self.max_torque)

	def getCurrDir(self):
		global currdir, take_no, act_out0, act_out1
		act_out0 = []
		act_out1 = []
		print 'Current directory: '
		currdir = raw_input()
		print 'Take number: '
		take_no = raw_input()


	def torqueMotor(self,index,val,pos_val=None):
		val = min(1.0,max(val,0))	#by design, can exceed default max torque value
		self.modes[index] = False	#swap to torque mode in record keeping
		s = self.servos[index]
		if pos_val is None:
			enc = s.settings['max_encoder']
		else:
			pos_val = min(max(0,pos_val),1.0) #new target position is always max to force saturation of servo torque
			enc = int((pos_val * (self.motorMax[index]-self.motorMin[index])+self.motorMin[index]) * s.settings['max_encoder'])

		s.apply_max_torque(val)
		s.move_to_encoder(enc)

	def moveHand(self,vals):
		if len(vals)!=len(self.servos):
			print "[ERR] Motor number mismatch"
		else:
			for i in xrange(len(vals)):
				self.moveMotor(i,vals[i])

	#returns motor position amnt, between designated min and max values
	def readMotor(self,index):
		servo = self.servos[index]
		enc = servo.read_encoder()
		if self.motorDir[index]>0:
			val = (enc/float(servo.settings["max_encoder"])-self.motorMin[index]) / (self.motorMax[index]-self.motorMin[index])
		else:
			val = (self.motorMax[index]-enc/float(servo.settings["max_encoder"])) / (self.motorMax[index]-self.motorMin[index])
		return val,enc


	def readLoads(self):
		for servo in self.servos:
			print "---"
			print "Servo ID: "+repr(servo.servo_id)
			print "Load: "+repr(servo.read_load())

	def readMotorMins(self):
		index=0
		for servo in self.servos:
			print "---"
			print "Servo ID: "+repr(servo.servo_id)
			print "Motor Mins: "+ repr(self.motorMin[index])
			index=index+1

	def readHand(self):
		amnts = np.array([0.]*len(self.servos))
		encs = np.array([0]*len(self.servos))

		for i in xrange(len(self.servos)):
			amnt,enc = self.readMotor(i)
			amnts[i] = amnt
			encs[i] = enc

		return amnts, encs

	#takes the current location and sets either the min or max
	def setMotorMin(self):
		amnts,encs = self.readHand()
		self.motorMin = (encs/float(self.servos[0].settings['max_encoder'])).tolist()
	def setMotorMax(self):
		amnts,encs = self.readHand()
		self.motorMax = (encs/float(self.servos[0].settings['max_encoder'])).tolist()

	#setting the max torque (shortcut for torque-based closing motions)
	def setMaxTorque(self,val=0.4):
		val = max(min(1.0,val),0.1)
		for servo in self.servos:
			servo.apply_max_torque(val)
			time.sleep(self.pause)	#helps mitigate eeprom delay issues?
		self.max_torque = val

	def setServoSpeed(self,val=1.0):
		val = max(min(1.0,val),0.1)
		for servo in self.servos:
			servo.apply_speed(val)
			time.sleep(self.pause)
		self.servo_speed = val

	#moves to the current encoder value and locks servos in place to minimize current draw
	def preventAllLoadErrors(self,offset_scale = 0):
		for i in range(len(self.servos)):
			self.preventLoadError(i,offset_scale)
	def preventLoadError(self,i,offset_scale = 0):
		if abs(self.servos[i].read_load()) > 80:	#arbitrary load threshold
			value = offset_scale*10 + self.servos[i].read_encoder()	#should never be negative
			if value < self.servos[i].settings['max_encoder']:
				self.servos[i].move_to_encoder(value)
			else:
				if value < 0:
					self.servos[i].move_to_encoder(0)
				else:
					self.servos[i].move_to_encoder(self.servos[i].settings['max_encoder'])
	#move to the current position for each motor - fast switch from torque mode and alternative to preventAllLoadErrors
	def hold(self):
		for i in range(len(self.servos)):
			amnt, enc = self.readMotor(i)
			self.moveMotor(i,amnt)

	def diagnostics(self):
		for servo in self.servos:
			print "---"
			print "Servo ID: "+repr(servo.servo_id)
			print "Load: "+repr(servo.read_load())
			print "Temperature: "+repr(servo.read_temperature())
			print "Target Encoder: "+repr(servo.read_target_encoder())
			print "Current Encoder: "+repr(servo.read_encoder())

#------------------------------------------------------#

#Different hand types

#------------------------------------------------------#

class GR2(OpenHand):
	motorDir = [1,1]
	motorMin = [0.05,0.05]	#should always be symmetric here?
	motorMax = [0.7,0.7]

	HOLD_TORQUE = 0.2
	OVERSHOOT = 0.15

	pause = 0.2

	def __init__(self,port="/dev/ttyUSB0",s1=1,s2=2,dyn_model="MX"):
		OpenHand.__init__(self,port,[s1,s2],dyn_model)

	def reset(self):
		self.release()
	def release(self):
		self.moveMotor(0,self.amnt_release)
		self.moveMotor(1,self.amnt_release)
	def close(self,amnt=0.3):
		self.moveMotor(0,amnt)
		self.moveMotor(1,amnt)
	#replacement for preventAllLoadErrors() due to servo state constraints
	def hold(self):
		for i in xrange(2):
			amnt,enc = self.readMotor(i)
			self.moveMotor(i,amnt)	#accounts for possible transition from torque mode
	#tval: torque value
	#dpos: delta in position from current (may force operation into compliance region)
	def close_torque(self,tval=None,dpos=1.0):
		if tval is None:
			tval = self.HOLD_TORQUE
		self._close_torques(tval,dpos)

	#demo motion that moves object back and forth with shift (assumes symmetry in operation)
	def sweep(self,val):
		amnts,encs = self.readHand()	#record starting pose to return to
		self.shift(0,val)
		time.sleep(self.pause)
		self.shift(1,val)
		time.sleep(self.pause*3)
		self.shift(0,amnts[0])
		time.sleep(self.pause)
		self.shift(0,amnts[1])
		time.sleep(self.pause)
		self.moveHand(amnts)

	#assumes operation starts with grasp w/ full contact
		#either pushing or relaxing a finger towards a desired position
	def shift(self,index,val):
		other_index = (index+1)%2
		vals,encs = self.readHand()

		if val<vals[index]:
			self.torqueMotor(index,0.)
			self.moveMotor(other_index,vals[other_index]+(vals[index]-val))
		else:
			self.torqueMotor(other_index,0.)
			self.moveMotor(index,val)

		i=0
		while i<10:		#arbitrary step count to 10 as system settles
			s_val,s_enc = self.readMotor(index)
			s_val_err = abs(s_val-val)
			print "Shifting error: "+repr(round(s_val_err,4))
			if s_val_err<0.01:
				break
			time.sleep(self.pause)
			i+=1

		print "Final shift error: "+repr(round(s_val_err,4))
		self.hold()
		return s_val_err


class Model_O(OpenHand):
	servo_speed = 1.0
	max_torque = 0.4
	amnt_close = 0.5 #default close position
	max_close = 0.7 #max motor movement from open to close for individual hand

	modes = [True,True,True,True] #True if in position control

	HOLD_TORQUE = 0.2
	OVERSHOOT = 0.15

	#RESET THE MOTOR MINS FOR FASTER INITIALIZATION
	motorDir = [1,1,-1,1] # one finger is opposite due to placement on the openhand base
	motorMin = [0.0,0.22,0.13,0.27]
	adduct_amount = 0.37
	motorMax = [motorMin[0]+adduct_amount,motorMin[1]+max_close,motorMin[2]+max_close,motorMin[3]+max_close] #RX and MX motors may use an offset of 0.48 instead of 0.40

	HAND_HEIGHT = 0.14
	WRIST_OFFSET = -np.pi/4

	def __init__(self,port="/dev/ttyUSB0",s1=2,s2=1,s3=3,s4=4,dyn_model="RX",s1_min=motorMin[0],s2_min=motorMin[1],s3_min=motorMin[2],s4_min=motorMin[3], abduction_limit =1):
		#s1: adduction/abduction motor for spread
		#s2: forward-driving finger
		#s3: reverse-driving finger
		#s4: thumb

		mot_offset = [s1_min,s2_min,s3_min,s4_min]

		if(mot_offset != self.motorMin):  #update motor mins if initialized to different values
			print 'Setting new motor minimums... '
			self.motorMin = mot_offset
			self.motorMax = [self.motorMin[0]+self.adduct_amount,self.motorMin[1]+self.max_close,self.motorMin[2]+self.max_close,self.motorMin[3]+self.max_close]

		self.abduction_limit = abduction_limit
		OpenHand.__init__(self,port,[s1,s2,s3,s4],dyn_model)

	def reset(self):
		self.release()
		time.sleep(0.5)
		self.moveMotor(0,0.)	#moves fingers into lateral pinch mode with fingers orthogonal to thumb

	def release(self):
		self.moveMotor(1,0.)
		self.moveMotor(2,0.)
		self.moveMotor(3,0.)

	def open(self):
		self.moveMotor(1,0.)
		self.moveMotor(2,0.)
		self.moveMotor(3,0.)

	def close(self,amnt=0.5):
		self.moveMotor(1,amnt)
		self.moveMotor(2,amnt)
		self.moveMotor(3,amnt)

	def change_motor_min(self,index, val):
		if (index < 0 or index >= len(self.servos)):
			print "[ERR] invalid motor index "+repr(index)
		else:
			if (index >0):
				self.motorMin[index]=val
				self.motorMax[index]=val+self.max_close
			else:	#case of the adduction motor
				self.motorMin[index]=val
				self.motorMax[index]=val+0.5
			self.reset()
			print 'Index changed successfully...'

	#abduct/adduct fingers - if no param given, move to power grasp
	def adduct(self,amnt=1):
		self.moveMotor(0,amnt)

    	#abduct fingers - then pinch close
	def pinch_close(self,amnt=0.4):
		adduct_loc, enc = self.readHand()
		if(adduct_loc[0] > 0.05):
			self.reset()
                	time.sleep(1.5) #pause for 1.5 seconds to allow reset
		self.moveMotor(1,amnt)
		self.moveMotor(2,amnt)

	#adduct fingers - then power close
	def power_close(self,amnt=0.6):
		adduct_loc, enc = self.readHand()
		if(adduct_loc[0] < 0.95):
			self.release()
                	time.sleep(1) #pause for 1 second to allow release
                	self.adduct(1)  #move fingers facing thumb
                	time.sleep(1)
		self.close(amnt)

    	#Example why torque control is required for fingertip manipulation
	def pinch_object_move(self,delta_amnt=0.03, left=True, down = False): #These cannot be the same or else we will move diagonal
		adduct_loc, enc = self.readHand()
		if(adduct_loc[0] > 0.05):
			print '[ERR] Hand is not in a pinch grasp'
              		return
		else:
			locs, enc = self.readHand()
			if left==True and down == False:		#move left
				self.moveMotor(1,locs[1]+delta_amnt)
				self.moveMotor(2,locs[2]-delta_amnt)
			elif left==False and down == False: 		#move right
				self.moveMotor(1,locs[1]-delta_amnt)
				self.moveMotor(2,locs[2]+delta_amnt)
			elif left==False and down == True: 		#move down
				self.moveMotor(1,locs[1]+delta_amnt)
				self.moveMotor(2,locs[2]+delta_amnt)
			else: 						#move up
				self.moveMotor(1,locs[1]-delta_amnt)
				self.moveMotor(2,locs[2]-delta_amnt)

	#replacement for preventAllLoadErrors() due to servo state constraints
	def hold(self):
		for i in [1,2]:
			amnt,enc = self.readMotor(i)
			self.modes[i] = True
			self.servos[i].apply_max_torque(self.max_torque)
			self.moveMotor(i,amnt+0.025)	#accounts for possible transition from torque mode

	#tval: torque value
	#dpos: delta in position from current (may force operation into compliance region)
	def close_torque(self,tval=None,dpos=1.0):
		if tval is None:
			tval = self.HOLD_TORQUE
		self._close_torques(tval,dpos)

	#Sets the motor into a torque mode so that fingertip manipulation is possible
	def torqueMotor(self,index,val,pos_val=None):
		val = min(1.0,max(val,0))	#by design, can exceed default max torque value
		self.modes[index] = False   #turn into torque mode
		s = self.servos[index]
		if pos_val is None:
			enc = int(s.read_encoder()+self.OVERSHOOT * s.settings['max_encoder'])
		else:
			pos_val = min(max(0,pos_val),1.0)
			enc = int((pos_val * (self.motorMax[index].modes-self.motorMin[index])+self.motorMin[index]) * s.settings['max_encoder'])

		s.apply_max_torque(val)

    	#assumes operation starts with grasp w/ full contact
	#either pushing or relaxing a finger
	#shifts object to the edge of the workspace
	def shift(self,index,val, wait_range=None):
		if index != 1 and index !=2:
			print "[ERR] Can only shift using power grasp with opposing fingers 1 and 2"
			return

		other_index=1
		if (index == 1):
			other_index=2
		vals,encs = self.readHand()

		if val<vals[index]:
			self.torqueMotor(index,0.03)
			self.moveMotor(other_index,vals[other_index]+(self.motorDir[index])*(vals[index]-val))
		else:
			self.torqueMotor(other_index,0.03)
			self.moveMotor(index,val)

		if wait_range == None:
			wait_range = 7

		for i in range(wait_range):
			s_val,s_enc = self.readMotor(index)
			s_val_err = abs(s_val-val)
			if s_val_err<0.005:
				break
			time.sleep(self.pause)

		print "Final shift error: "+repr(round(abs(val-s_val),4))
		self.hold()
		return s_val_err

	#demo motion that moves object back and forth with shift (assumes symmetry in operation)
	def sweep(self,val=None):
		adduct_loc, enc = self.readHand()
		if(adduct_loc[0] > 0.05):
			print '[ERR] Hand must be in pinch_close mode before sweep'
			return

		amnts,encs = self.readHand() #record starting pose to return to
		if val == None:
			val=amnts[1]+0.10
		self.shift(1,val,7)
		time.sleep(0.5)
		self.shift(2,val,14)
		time.sleep(0.5)
		self.shift(1,val-0.05,12)
		time.sleep(0.5)
		self.shift(2,val-0.05,12)
		time.sleep(0.5)
		self.shift(1,amnts[1],10)
		time.sleep(self.pause)
		print "Sweep Completed.."

	#jiggling the fingers closed
	def close_jiggle(self,amnt=0.5,da=0.05,nsteps=5,pause=0.25):
		amnt_start,amnt_enc = self.readHand()
		s_amnt = amnt_start[0]
		if s_amnt<0.5:
			print "[WARNING] Fingers may be spread too far apart for closing motion"
                        print "Consider moving fingers to adduction value of greater than 0.5"

		amnt_goal = np.array([amnt,amnt,amnt,s_amnt])
		da_arr = np.array([da,-da,0.,0.])
		for i in xrange(nsteps):
                        amnt_curr=self.readHand()
			amnt_arr = amnt_start+(amnt_goal-amnt_curr)*float(i)/nsteps+da_arr*(-1)**i

			self.moveHand(amnt_arr)
			time.sleep(pause)
		self.moveHand(amnt_goal)


class Model_T42(OpenHand):
	servo_speed = 0.25

	modes = [True, True] #True if in position control

	max_close = 0.75
	motorDir = [1,1]
	motorMin = [0.05,0.05]
	motorMax = [motorMin[0]+max_close,motorMin[1]+max_close]

	HAND_HEIGHT = 0.08
	WRIST_OFFSET = -5*np.pi/12

	def __init__(self,port="/dev/ttyUSB0",s1=1,s2=2,dyn_model="RX", s1_min = motorMin[0], s2_min = motorMin[1]):
		#s1: "forefinger"
		#s2: "thumb"
		mot_offset = [s1_min,s2_min]

		if(mot_offset != self.motorMin):  #update motor mins if initialized to different values
			print 'Setting new motor minimums... '
			self.motorMin = mot_offset
			self.motorMax = [self.motorMin[0]+self.max_close,self.motorMin[1]+self.max_close]

		OpenHand.__init__(self,port,[s1,s2],dyn_model)

	#default OpenHand commands:
	def reset(self):
		self.release()

	def close(self,amnt=0.45):	#position-based closing mechanism
		self.moveMotor(0,amnt)
		self.moveMotor(1,amnt)

	def followTrajectory(self):	#position-based closing mechanism
		act1inputs = np.array([0.6777,0.6854,0.6820,0.6762,0.6705,0.6531,0.6363,0.6256,0.6162,0.6053,0.6012,0.5963,0.5935,0.5905,0.5888,0.5864,0.5846,0.5815,0.5782,0.5729,0.5654,0.5555,0.5467,0.5381,0.5317,0.5283,0.5274,0.5272,0.5271,0.5232,0.5179,0.5077,0.4962,0.4845,0.4764,0.4695,0.4669,0.4654,0.4624,0.4589,0.4556,0.4527,0.4502,0.4493,0.4491,0.4490,0.4490,0.4490,0.4490,0.4490])
		act2inputs = np.array([0.6629,0.6726,0.6733,0.6751,0.6788,0.6784,0.6780,0.6791,0.6796,0.6753,0.6741,0.6728,0.6722,0.6717,0.6716,0.6717,0.6722,0.6726,0.6738,0.6751,0.6756,0.6754,0.6758,0.6758,0.6784,0.6818,0.6855,0.6858,0.6858,0.6827,0.6792,0.6762,0.6761,0.6759,0.6728,0.6675,0.6617,0.6560,0.6522,0.6514,0.6524,0.6533,0.6535,0.6520,0.6504,0.6497,0.6492,0.6491,0.6490,0.6490])

		self.moveMotor(0,0.15)
		self.moveMotor(1,0.31)

		time.sleep(1)

		for i in range(len(act1inputs)):
			time.sleep(0.25)
			print "thumb: %.2f" % Decimal(act1inputs[i]*(0.50-0.15)+0.05)
			print "2-link: %.2f" % Decimal(act2inputs[i]*(0.65-0.10)+0.05)
			self.moveMotor(1,act1inputs[i]*(0.50-0.15)+0.) #1 link

			time.sleep(0.05)
			self.moveMotor(0,act2inputs[i]*(0.65-0.10)+0.05) #2 link

			#gaitHand(self,vals,num=5,overshoot=0.5,release=True)

	def shootOutRight(self,shift_amnt=0.4,close_amt=0.25):
		self.release()
		time.sleep(1)
		self.moveMotor(0,shift_amnt)
		time.sleep(1)
		self.close(close_amt-0.10)
		time.sleep(1)
		self.close(close_amt)

	def shootOutLeft(self,shift_amnt=0.37,close_amt=0.3):
		self.release()
		time.sleep(1)
		self.moveMotor(1,shift_amnt)
		time.sleep(1)
		self.close(close_amt)

	def close_pause(self, amnt=0.45):  # position-based closing mechanism
		self.moveMotor(0, amnt)
		self.moveMotor(1, amnt)
		time.sleep(self.pause)

	def release(self):
		self.moveMotor(0,self.amnt_release)
		self.moveMotor(1,self.amnt_release)

	#model-specific OpenHand commands:
	def flip_init(self):
		self.moveMotor(0,self.amnt_release)
		self.moveMotor(1,self.amnt_close)

	def move_right(self,val,close_amnt):
		#self.getCurrDir()
		for i in range(0, 1):
			time.sleep(self.pause * 2) #time to settle
			amnt, enc = self.readMotor(1)
			self.moveMotor(0,amnt + val)
			self.moveMotor(1,amnt - val)
			time.sleep(self.pause * 2)
			self.close(close_amnt)
		#self.saveActVals()

	def gait_right(self, val, num, close_amnt):
		self.getCurrDir()
		for i in range(0, 1):
			time.sleep(self.pause * 2)  # time to settle
			amnt, enc = self.readMotor(1)
			inc = val/num
			for i in range(0,num):
				self.moveMotor(1, amnt - i*inc)
				time.sleep(self.pause)
				self.moveMotor(0, amnt + i * inc)
				time.sleep(self.pause)
			time.sleep(self.pause * 2)
			self.close(close_amnt)
		self.saveActVals()

	def move_left(self,val,close_amnt):
		self.getCurrDir()
		for i in range(0, 1):
			time.sleep(self.pause * 2)
			amnt, enc = self.readMotor(1)
			self.moveMotor(1, amnt + val)
			self.moveMotor(0, amnt - val)
			time.sleep(self.pause * 2)
			self.close(close_amnt)
		self.saveActVals()

	def gait_left(self, val, num,close_amnt):
		self.getCurrDir()
		for i in range(0, 1):
			time.sleep(self.pause * 2)  # time to settle
			amnt, enc = self.readMotor(1)
			inc = val / num
			for i in range(0, num):
				self.moveMotor(1, amnt + i * inc)
				time.sleep(self.pause)
				self.moveMotor(0, amnt - i * inc)
				time.sleep(self.pause)
			time.sleep(self.pause * 2)
			self.close(close_amnt)
		self.saveActVals()

	def pull_in(self,val1,val2,close_amnt):
		self.getCurrDir()
		for i in range(0, 1):
			time.sleep(self.pause * 2)
			amnt, enc = self.readMotor(1)
			self.moveMotor(1, amnt - val2)
			self.moveMotor(0,amnt + val1)
			#time.sleep(self.pause * 2)
			time.sleep(self.pause * 2)
			self.close(close_amnt)
		self.saveActVals()

	def gait_in(self, val1, val2, num,close_amnt):
		self.getCurrDir()
		for i in range(0,1):
			time.sleep(self.pause * 2)
			amnt, enc = self.readMotor(1)
			inc2 = val2 / num
			inc1 = val1 / num
			for i in range(0, num):
				self.moveMotor(1, amnt - inc2 * i)
				time.sleep(self.pause)
				self.moveMotor(0, amnt + inc1 * i)
				time.sleep(self.pause)
			# time.sleep(self.pause * 2)
			time.sleep(self.pause * 2)
			self.close(close_amnt)
		self.saveActVals()


	def move_out(self): #questionable if this could even work
		time.sleep(self.pause * 2)
		amnt, enc = self.readMotor(1)
		for i in range (1,11):

			#move_right
			time.sleep(self.pause)  # time to settle
			self.moveMotor(0, amnt + 0.01*i)
			self.moveMotor(1, amnt - 0.01*i)
			#move_left
			time.sleep(self.pause)
			self.moveMotor(0, amnt - 0.01*i)
			time.sleep(self.pause)
			self.moveMotor(1, amnt + 0.01*i)
		time.sleep(self.pause * 2)

	def flip_close(self):
		self.moveMotor(0,self.amnt_close)
		self.moveMotor(1,self.amnt_close)


	# demo motion that moves object back and forth with shift (assumes symmetry in operation)
	def sweep(self, val):
		amnts, encs = self.readHand()  # record starting pose to return to
		self.shift(0, val)
		time.sleep(self.pause)
		self.shift(1, val)
		time.sleep(self.pause * 3)
		self.shift(0, amnts[0])
		time.sleep(self.pause)
		self.shift(0, amnts[1])
		time.sleep(self.pause)
		self.moveHand(amnts)


	# assumes operation starts with grasp w/ full contact
	# either pushing or relaxing a finger towards a desired position
	def shift(self, index, val):
		other_index = (index + 1) % 2
		vals, encs = self.readHand()

		if val < vals[index]:
			self.torqueMotor(index, 0.)
			self.moveMotor(other_index, vals[other_index] + (vals[index] - val))
		else:
			self.torqueMotor(other_index, 0.)
			self.moveMotor(index, val)

		i = 0
		while i < 10:  # arbitrary step count to 10 as system settles
			s_val, s_enc = self.readMotor(index)
			s_val_err = abs(s_val - val)
			print "Shifting error: " + repr(round(s_val_err, 4))
			if s_val_err < 0.01:
				break
			time.sleep(self.pause)
			i += 1

		print "Final shift error: " + repr(round(s_val_err, 4))
		self.hold()
		return s_val_err

class Model_T(OpenHand):
	max_torque = 0.4
	limit_close = 10		#counter for a standard close (to prevent servo stuck in closing mode in event of tendon failure)

	modes = [True]

	max_close = 0.75
	motorDir = [1]
	motorMin = [0.05]
	motorMax = [motorMin[0]+max_close]

	HAND_HEIGHT = 0.14
	WRIST_OFFSET = -np.pi/4

	def __init__(self,port="/dev/ttyUSB0",s1=1,dyn_model="MX", s1_min = motorMin[0]):

		mot_offset = [s1_min]
		if(mot_offset != self.motorMin):  #update motor mins if initialized to different values
			print 'Setting new motor minimums... '
			self.motorMin = mot_offset
			self.motorMax = [self.motorMin[0]+self.max_close]

		OpenHand.__init__(self,port,[s1],dyn_model)

	def reset(self):
		self.moveMotor(0,self.amnt_release)

	#may exceed encoder limit, especially depending on the servo used
	def close_torque(self,amnt=0.5):
		self.servos[0].enable_torque_mode()
		self.servos[0].apply_torque(amnt*self.max_torque)
		time.sleep(self.pause*2)

		i,sp = 0,1.
		while i<self.limit_close and sp>0:
			sp = self.servos[0].read_speed()
			print "close (speed): "+repr(sp)
			i += 1
			time.sleep(self.pause)

		self.servos[0].disable_torque_mode()
		self.preventLoadError(0)
		return True

	def close_wheel(self,amnt=0.5,speed=0.2):	#closing through wheel mode
		#set torque output to max, use wheel speed to modulate closing force
		self.servos[0].disable_torque_mode()
		self.servos[0].init_cont_turn()
		self.servos[0].apply_speed(speed)
		time.sleep(self.pause*2)

		i,sp = 0,1.
		while i<self.limit_close and sp>0:
			sp = self.servos[0].read_speed()
			print "close (speed): "+repr(sp)
			i += 1
			time.sleep(self.pause)

		self.servos[0].kill_cont_turn()
		self.preventLoadError(0)
		return True

	def close(self,amnt=0.2):			#closing through position mode and torque limit
		#set target position to furthest limit, but change servo torque limit
		self.servos[0].apply_max_torque(amnt)
		time.sleep(self.pause)
		self.servos[0].move_to_encoder(self.servos[0].settings['max_encoder']-1)
		time.sleep(self.pause)

		i,sp = 0,1.
		while i<self.limit_close and sp>0:
			sp = self.servos[0].read_speed()
			print "close (speed): "+repr(sp)
			i += 1
			time.sleep(self.pause)

		self.preventLoadError(0)
		self.servos[0].apply_max_torque(self.max_torque)
		return True

	def release(self):					#should work for all previous close cases
		self.servos[0].enable_torque_mode()

		self.servos[0].apply_torque(0.)			#allow natural compliance to loosen grasp for tight grip cases
		time.sleep(self.pause*2)

		self.servos[0].disable_torque_mode()

		self.servos[0].kill_cont_turn()			#back to position mode
		self.servos[0].apply_speed(self.servo_speed)	#check in case it was in wheel mode
		self.moveMotor(0,self.amnt_release)
