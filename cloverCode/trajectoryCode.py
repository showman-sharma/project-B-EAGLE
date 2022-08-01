import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
import pigpio #for reading GPIO inputs


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

