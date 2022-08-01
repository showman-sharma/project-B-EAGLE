# RF communication code taken from https://github.com/BirchJD/RPi_433MHz/blob/master/Pi433MHzTx.py

import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
import pigpio #for reading GPIO inputs
import os
import sys
import time
import datetime

# GPIO Pin connected to 433MHz receiver.
GPIO_RX_PIN = 26
# GPIO Pin connected to 433MHz transmitter.
GPIO_TX_PIN = 19

# GPIO level to switch transmitter off.
TX_OFF_LEVEL = 1
# GPIO level to switch transmitter on.
TX_ON_LEVEL = 0
# Period to signify end of Tx message.
TX_END_PERIOD = 0.01
# Single level period, one period is a binary 0, two periods are a binary 1. 
TX_LEVEL_PERIOD = 0.002
# Start bits transmitted to signify start of transmission.
TX_START_BITS = 1

# Data encryption key.
ENCRYPTION_KEY = [ 0xC5, 0x07, 0x8C, 0xA9, 0xBD, 0x8B, 0x48, 0xEF, 0x88, 0xE1, 0x94, 0xDB, 0x63, 0x77, 0x95, 0x59 ]
# Data packet identifier.
PACKET_SIGNATURE = [ 0x63, 0xF9, 0x5C, 0x1B ]



# Data packet to transmit.
DataPacket = {
   "SIGNATURE": PACKET_SIGNATURE,
   "DATA_LENGTH": 0,
   "DATA": [],
   "CHECKSUM": 0,
}


# Transmit a byte of data from the 433MHz module.
def Tx433Byte(Byte):
   global CurrentTxLevel

   BitMask = (1 << 7)
   for BitCount in range(8):
      # Get the next bit from the byte to transmit.
      Bit = (Byte & BitMask)
      BitMask = int(BitMask / 2)

      # Toggle GPIO level.
      if CurrentTxLevel == TX_OFF_LEVEL:
         CurrentTxLevel = TX_ON_LEVEL
      else:
         CurrentTxLevel = TX_OFF_LEVEL
      RPi.GPIO.output(GPIO_TX_PIN, CurrentTxLevel)
      # Bit 0 level period.
      time.sleep(TX_LEVEL_PERIOD)
      # Bit 1 level additional period.
      if Bit > 0:
         time.sleep(TX_LEVEL_PERIOD)

# A very basic encrypt/decript function, for keeping demonstration code simple. Use a comprehensive function in production code.
def BasicEncryptDecrypt(Data):
   KeyCount = 0
   KeyLen = len(ENCRYPTION_KEY)
   for Count in range(len(Data)):
      Data[Count] ^= ENCRYPTION_KEY[KeyCount]
      if KeyCount >= KeyLen:
         KeyCount = 0

#  /*******************************************/
# /* Configure Raspberry Pi GPIO interfaces. */
#/*******************************************/
RPi.GPIO.setwarnings(False)
RPi.GPIO.setmode(RPi.GPIO.BCM)
RPi.GPIO.setup(GPIO_RX_PIN, RPi.GPIO.IN, pull_up_down=RPi.GPIO.PUD_UP)
RPi.GPIO.setup(GPIO_TX_PIN, RPi.GPIO.OUT, initial=TX_OFF_LEVEL)


# initializing connection to pigpiod
pi = pigpio.pi()

rospy.init_node('flight')
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
#navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
#set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
#set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
#set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
#set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

# setting pins 12,23, 24,25 mode for input for gas sensor input
N = 12; S = 23; E = 24; W = 25; U = 5; D = 6;
pi.set_mode(N, pigpio.INPUT) #North
pi.set_mode(S, pigpio.INPUT) #South
pi.set_mode(E, pigpio.INPUT) #East
pi.set_mode(W, pigpio.INPUT) #West 
pi.set_mode(U, pigpio.INPUT) #Up
pi.set_mode(D, pigpio.INPUT) #Down 



def changeLoc(I,delta,stepSize):
  dIx = (I['E']-I['W'])/(2*delta)
  dIy = (I['N']-I['S'])/(2*delta)
  dIz = (I['U']-I['D'])/(2*delta)
  lenI = sqrt(dIx**2+dIy**2+dIz**2)
  dx = stepSize*dIx/lenI; 
  dy = stepSize*dIy/lenI;
  dz = stepSize*dIz/lenI;
  return (dx),(dy),(dz)

def find_source(delta = 0.2,stepSize = 0.5 ,x=0, y=0, z=1, yaw=float('nan'), yaw_rate=0, speed=0.5, frame_id='body', tolerance=0.2, auto_arm=False):

    I = {'E':pi.read(E),'W':pi.read(W),'N':pi.read(N),'S':pi.read(S),'U':pi.read(U),'D':pi.read(D)}
    trajectory = [[x,y,z]]
    error = stepSize+1
    while error>=stepSize:
      dx,dy,dz = nextLoc(I,delta,stepSize)
      res = navigate(x=dx, y=dy, z=dz, yaw=yaw, yaw_rate=yaw_rate, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
      if not res.success:
        return None


      trajectory.append([dx+trajectory[-1][0],dy + trajectory[-1][1],dz + trajectory[-1][2]])
      try:
        #error is the distance between current point and previous-to-previous point
        error = sqrt((trajectory[-1][0]-trajectory[-3][0])**2+(trajectory[-1][1]-trajectory[-3][1])**2+(trajectory[-1][2]-trajectory[-3][2])**2)
      except:
        pass 
      
      rospy.sleep(10) #waiting for 10 seconds to let the gas reading saturate
        
      I = {'E':pi.read(E),'W':pi.read(W),'N':pi.read(N),'S':pi.read(S),'U':pi.read(U),'D':pi.read(D)}
      
    land()
    return trajectory

trajectory = find_source()

data = '('+str(trajectory[-1][0])+','+str(trajectory[-1][1])+','+str(trajectory[-1][2])+')'

# Place data into data packet and set packet values ready to be sent.
DataPacket["DATA_LENGTH"] = len(data)
#Tokenise and encrypt data to be sent.
DataPacket["DATA"] = list(data)
for Count in range(len(DataPacket["DATA"])):
  DataPacket["DATA"][Count] = ord(DataPacket["DATA"][Count])
BasicEncryptDecrypt(DataPacket["DATA"])
# Calculate checksum of data for transmission validation.
DataPacket["CHECKSUM"] = 0
for Byte in DataPacket["DATA"]:
  DataPacket["CHECKSUM"] ^= Byte

# Display data packet being sent.
sys.stdout.write("\nSENDING PACKET:\n")
sys.stdout.write(str(DataPacket) + "\n\n")

# Switch on 433MHz transmitter.
CurrentTxLevel = TX_ON_LEVEL
RPi.GPIO.output(GPIO_TX_PIN, CurrentTxLevel)
# Wait for the number of start bits.
for Count in range(TX_START_BITS):
  time.sleep(TX_LEVEL_PERIOD)

# Transmit data packet signature.
for Byte in DataPacket["SIGNATURE"]:
  Tx433Byte(Byte)

# Transmit data packet data length.
Tx433Byte(DataPacket["DATA_LENGTH"])

# Transmit data packet encrypted data.
for Byte in DataPacket["DATA"]:
  Tx433Byte(Byte)

# Transmit data packet data checksum.
Tx433Byte(DataPacket["CHECKSUM"])

# Switch off 433MHz transmitter.
CurrentTxLevel = TX_OFF_LEVEL
RPi.GPIO.output(GPIO_TX_PIN, CurrentTxLevel)

# End of transmission period.
time.sleep(TX_END_PERIOD)