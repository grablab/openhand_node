Yale OpenHand Node (ROS and Python Control)
==============
![Yale Openhand](banner4.png)
## Getting Started

Welcome to the Yale Grab Lab OpenHand Node Repository! All 3D files are openly available on the [OpenHand Hardware Github](https://github.com/grablab/openhand-hardware). The code provided in this repository supports the Model O, Model T42, and Model T. Additional models will be supported in the future, but please reach out if you need support. Additional documentation is available on the [OpenHand Website](http://www.eng.yale.edu/grablab/openhand/) with instruction manuals for fabrication and assembly. This repository supports Python2 and Python3, and Protocol1 (RX,MX actuators) and Protocol2 (X-series actuators) from Dynamixel. 

More information about the lab and present/previous work is available on the [Grab Lab Website](https://www.eng.yale.edu/grablab/).

## Requirements
Python 2 or Python 3 is required for simple control of the hand with a USB2Dynamixel controller interface ([U2D2](http://www.robotis.us/u2d2/) controller). The provided code was built and tested on Python 2.7 and 3.6. Serial communication is provided through the PySerial 3.x library and value storage utilizes the Numpy 1.X library, both of which you may have to install separately (see instructions later in the document). A power supply of 12V (max ~ 1.2A required) should be connected to power the Dynamixel motors through the [hub](http://www.trossenrobotics.com/6-port-rx-power-hub) or similar. *NOTE: The USB2Dynamixel(U2D2) device cannot power the motors. Please be sure to use the hub provided*
  
The main body of the code is formulated for simple implementaion in ROS (tested on Kinetic already created as a ```catkin_ws```). Simple control just through Python is possible by navigating to the ```src/openhand_node``` folder. The main files in this folder are the ```hands.py```, ```lib_robotis_mod.py```, and ```registerDict.py```. We will proceed by first describing simple control through Python.  

## Python Control
Python (works with both Python2 and Python3) code is used for easy control of the Dynamixel-based OpenHand designs. These Python objects depend on a modified version of the 'lib_robotis.py' library from Georgia Tech (included in our repository). The library has been updated to properly control both MX/RX (Protocol1) and X-series (Protocol2) servos, as well as accounting for possible header miscues as suggested by the pydynamixel library. Each hand object has pre-tested settings for servo bounds and torque output that we routinely use in our experiments. These values may vary for different hardware implementations or assembly processes. 

Before you begin, you will need to have a working (preferably updated) version of either Python 2 or Python 3. This installation is available at [python.org](https://www.python.org/) for your designated operating system, and should include the `pip` installation as well. You will also be required to have the ```numpy```, ```scipy```, and ```pyserial``` libraries installed. Once you have your preferred version of python installed, in your terminal (or command prom
pt) window, you should easily be able to install these libraries with the following commands on any directory level:

```python -m pip install pyserial```   
```python -m pip install numpy```   
```python -m pip install scipy```   

USB port names are required for serial communication with the motors. This naming convention is different on every operating system, but can typically be found using the following commands (terminal for MacOS/Linux and command prompt for Windows). At this time, the hand should be powered through the USB hub via the wall adapter and the U2D2 controller should be plugged into your computer via USB:

#### Linux
`$ ls /dev/ttyUSB*`  
`/dev/ttyUSB0`  
#### Mac
`$ ls /dev/tty.usb*`  
`/dev/tty.usbserial-AI03QF0H`  
#### Windows
`$ mode`   
`COM3`   


Please note that an additional [driver](http://www.ftdichip.com/Drivers/D2XX.htm) may be required for serial communication in Windows. Also note single quotations around the name of the port will typically be required later for running these programs. 

#### Finding Dynamixels Command: 

Record the port name presented in the terminal window. You will now also need to know the address of each Dynamixel you intend to control. Addresses can be changed using the [Dynamixel Wizard](http://www.robotis.us/roboplus1/), in Windows or by using the command line tool presented later on. For the Model O case, we should expect to see 4 unique addresses (for the Model T42 there should be 2 unique addresses) when looking for the motor addresses. We will use the ```lib_robotis_mod.py``` program to find these addresses. Navigate to the ```src/openhand_node/``` folder and run the following command in a terminal window:
 
Basic (write ```python3``` if not using Python 2, depending on your system configuration, and you may also need to use the ```sudo``` command before ```python``` or ```python3```):  
`$ python lib_robotis_mod.py -d <YOUR_COM_PORT> --scan`  __OR__  
`$ sudo python lib_robotis_mod.py -d <YOUR_COM_PORT> --scan` 

Example:  
`$ python lib_robotis_mod.py -d '/dev/ttyUSB0' --scan`   __OR__  
`$ python lib_robotis_mod.py -d 'COM9' --scan`

NOTE: This process can often be difficult to cancel out of in Windows (use ```ctrl + \``` in unix-based systems). Once dynamixels are found, you may close out of and restart the terminal without any changes to the configuration. If you have multiple dynamixels (daisy chain) connected and you are missing an ID for one or more motors, it is likely that there are motors in your system that share the same (address) ID, or an ID of 0 is used (invalid for Protocol 2). In this event, you will have to change the ID of matching motors so that they are unique in your system. While connecting just one motor to the serial module, you can run the following command:

#### Setting New ID for Dynamixel Motors:  

Basic:  
`$ python lib_robotis_mod.py -d <YOUR_COM_PORT>--setID <old_id> <new_id>`

Example:  
`$ python lib_robotis_mod.py -d '/dev/ttyUSB0' --setID 2 3`  
`$ python lib_robotis_mod.py -d 'COM9' --setID 2 3`  

With recorded port names and dynamixel addresses, you can now control your OpenHand. Pressing ```ctrl + d``` will exit the Python program. There may be times that you may need to exit the program using ```ctrl + \``` depending on the most recent command. 

#### Validating Your Motor Assignments  
  
To visually validate that you are correctly talking to your motors, we have added another function that will move all motors in the chain. To use this command, you can run the following:

`$ python lib_robotis_mod.py -d '/dev/ttyUSB0' --moveTest "1 2 3 4"`

If all motors are connected properly (note the string of integers can be as big or as little as desired), you should see all motors move and a confirmation echo in the terminal. 

### Basic Usage:  

`$ python -i hands.py`

```python
T = Model_O([port name], [abduction servo[1] id], [right forward servo[2] id],[left reverse servo[3] id],[thumb servo[4] id], [Dynamixel series ("RX", "MX", "XM")])  
T.close([desired tendon length for close (0.0-1.0)])
T.reset()
T.release()  
```

OR   


`$ python -i hands.py`

```python
T = Model_T42([port name], [left finger servo id], [right finger servo id], [Dynamixel series ("RX", "MX","XM")])  
T.close([desired tendon length for close (0.0-1.0)])
T.reset()
T.release()  
```

### Example Usage:

`$ python -i hands.py`

```python
 T = Model_O("/dev/ttyUSB0",2,1,3,4,"RX")  
 T.close(0.6)  
 T.release()  
 T.moveMotor(0,0.6)  
 T.moveMotor(1,0.6)  
 T.release()
 T.adduct()
 T.close(0.5)
 T.power_close(0.6)
```    

NOTE: Once initializing your openhand, the motor addresses are no longer used for control - these are only used for initialization. The motors will now be reference by index (zero-based) according to the order you specified in the object initialization. That is, for the Model O:{Index 0 = adduction, Index 1 = right finger, Index 2= left finger, Index 3= thumb). This configuration is specified when the Y is in the correct orientation. 

### Tuning:  
Tuning your OpenHand is one of the most important steps during the set up process. Due to the set screw gap in the dynamixel pulleys, it is often very difficult to tune the OpenHand perfectly without changing offsets in the code. We have implemented a simple function to find the correct offsets for each of the motors. This step may require a few iterations. 

#### Finding the correct offset   
We will proceed by assuming we are tensioning the Model O hand. It is important to recognize the correct order of IDs for initialization in the python command. The adduction motor should always be the first in the list, with the right finger next, left follows, and finally the thumb. We seek to set up the hand such that home position is with the tendons taught, but not moving the fingers, and the fingers to be perpendicular to the thumb. You are able to 'reset' or 'home' your hand if these offsets are correct which means we need to change the the minimum actuator value (which is implemented as a range from 0-1). You can find these offsets by iterating through values for each motor until the correct offset is recognized. The last string of characters signifies the types of actuators, either "MX", "RX", or "XM". For example:  

`$ python -i hands.py`

```python
 T = Model_O("/dev/ttyUSB0", 2, 1, 3, 4, "RX")  
 T.change_motor_min(1, 0.13)
 #user recognizes that the finger tendon is not taught. Note that the user should
 #be looking at the finger tendon and not the proximal reset tendon. 
 T.change_motor_min(1, 0.16)
 #tendon is now taught - offset is 0.16
```
For this initialization, the user notices that the tendon is tight without moving the finger. This value can be recorded for later use and implementation. Repeat the same command for each of the motors. Note that the abduction motor should be 'homed' when the two fingers are perpendicular to the thumb. When all offsets have been found, you can either change the initial offsets in the ```hands.py``` code (approximately line 331 and preferred), change each of them upon startup using the same ```change_motor_min()``` function call (example 1 below), or you can initialize each of them during your Model_O object call (example 2 below). The two examples are analogous to one another. 

##### Example 1
`$python -i hands.py`  
```python
T=Model_O('/dev/ttyUSB0', 2, 1, 3, 4, "RX")
T.change_motor_min(0,0.0)
T.change_motor_min(1,0.2)
T.change_motor_min(2,0.16)
T.change_motor_min(3,0.18)
```  
##### Example 2
`$python -i hands.py`  
```python
T=Model_O('/dev/ttyUSB0', 2, 1, 3, 4, "RX", 0.0, 0.2, 0.16, 0.18)
```  
The last four numbers are the motorMins of motors for index 0-3, respectively. These relate to the motor index, not the motor address.  

#### Available Methods
Functions available for the Model T, Model T42, and Model O differ according to the topology. Search through the ```hands.py```file to see which functions are supported. 

For example, here are some of the functions available for the Model O:

| Function Call                                  | Description                                                                         |
|------------------------------------------------|-------------------------------------------------------------------------------------|
| <obj_name>.reset( )                            | Moves fingers back to zero position                                         |                                              
| <obj_name>.release( )                          | Opens fingers, does not reset other non-finger servos                       |       
| <obj_name>.close(amt)                          | Closes the fingers by some specified amount ranging from 0.0 to 1.0         |               
| <obj_name>.moveMotor(motor_index, amt)         | Moves a motor (according to the index and not the address) by some specified amount |
| <obj_name>.adduct(amt)                         | Changes adduction/abduction location - default 0.5                          |
| <obj_name>.pinch_close(amt)                    | Setup pinch grasp with opposing fingers - default 0.4                       |
| <obj_name>.power_close(amnt)                   | Setup power grasp with fingers parallel to thumb - default 0.6              |
| <obj_name>.change_motor_min(motor_index, amt)  | Changes motor position minimum for correct reset() and closing              |
| <obj_name>.shift(motor_index, amt)             | Shifts the motor specified to a position beyond its current one. If the position is not beyond its current position, it will drop the object. This funciton uses torque control to maintain a stable grasp and assumes that the hand is already in a stable pinch_close() position.               |
| <obj_name>.sweep(amt)                          | Sweeps a pinch_close() object back and forth. This function may be tuned according to the size and shape of the object - default moves 0.1 beyond current position             |
| <obj_name>.readMotor(motor_index)              | Returns the position of the motor                                           |    
| <obj_name>.readMotorMins()                     | Returns the motor min value, used for resetting motor to initial position   |                                          
| <obj_name>.readLoads()                         | Returns the motor loads for the entire hand                                 |                   
| <obj_name>.readHand( )                         | Returns the positions of each motor in the hand                             |                        
| <obj_name>.diagnostics( )                      | Presents load, temperature, and encoder location for each motor in the hand |         


  

## ROS Node

A ROS catkin_ws is provided for further extension and functionality of the hands. This code was built using ROS Kinetic and was test with Python 2. You can launch the node with the associated ```openhand.launch``` file. Paramaters associated with the system must be updated according to the ```params.yaml``` file. 

Functions to the hand are provided through serviced in ```openhandNode.py```. After starting the node (```roslaunch openhand_node openhandNode.py```), run ```rosservice list``` to see what functions are inherently provided for your model. 


## Thank you  
Thank you for for your interest in the Yale Openhand Project. If you have any questions or interests in any of our other openhand designs and projects, please consider contributing to the opensource software. As always, feel free to contact maintainers of the project from the [OpenHand Contacts Page](https://www.eng.yale.edu/grablab/openhand/#contact). 

