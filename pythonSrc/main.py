import string
import sys
import time
import json
import jsbsim
import math
import pygame
import socket

aileronLimit = 0.3
aileronVal = 0.0
elevatorLimit = 0.3
elevatorVal = 0.0
throttleLimit = 1
throttleVal = 0.0
rudderLimit = 0.3
rudderVal = 0.0
stepVal = 0.0015

RemoteUdpAddress = '127.0.0.1'
RemoteUdpPort = 54000

pygame.init()
screen = pygame.display.set_mode((160, 120))
pygame.display.set_caption('keyboard ctrl')
screen.fill((0, 0, 0))

socketUdp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# socketUdp.bind((RemoteUdpAddress, RemoteUdpPort))

fdm = jsbsim.FGFDMExec(None)  # Use JSBSim default aircraft data.
fdm.load_model('Concorde')
# fdm.load_script('scripts/Concorde_runway_test.xml')

# Velocity Initialization
fdm['ic/vc-kts'] = 0

# Position Initialization
fdm["ic/lat-gc-deg"] = 0
fdm["ic/long-gc-deg"] = 0
fdm["ic/h-sl-ft"] = 0

# Attitude Initialization
fdm["ic/psi-true-deg"] = 0
fdm["ic/theta-deg"] = 0
fdm["ic/phi-deg"] = 0

##########################
## Model Initialization ##

fdm.run_ic()
##########################

# Turning on the Engine
fdm["propulsion/starter_cmd"] = 1
fdm["propulsion/active_engine"] = True
fdm["propulsion/set-running"] = 1

fdm["fcs/throttle-cmd-norm"] = 0.5
fdm["fcs/mixture-cmd-norm"] = 1.0
fdm["propulsion/magneto-cmd"] = 3.0
fdm["propulsion/starter-cmd"] = 1.0
fdm["ap/autopilot-roll-on"] = 1.0
fdm["ap/roll-altitude-mode"] = 0.0
fdm["ap/heading-hold"] = 0.0

# First but not Initial
fdm.run()

while True:
  for event in pygame.event.get():
      if event.type == pygame.QUIT:
          sys.exit()

  time.sleep(0.01)

  scan_wrapper = pygame.key.get_pressed()

  if scan_wrapper[pygame.K_DOWN] and (elevatorVal >= (stepVal-elevatorLimit)):
    elevatorVal -= stepVal

  if scan_wrapper[pygame.K_UP] and (elevatorVal <= (elevatorLimit-stepVal)):
    elevatorVal += stepVal

  if scan_wrapper[pygame.K_LEFT] and (aileronVal >= (stepVal - aileronLimit)):
    aileronVal -= stepVal

  if scan_wrapper[pygame.K_RIGHT] and (aileronVal <= (aileronLimit - stepVal)):
    aileronVal += stepVal

  if scan_wrapper[pygame.K_SPACE] and (throttleVal <= (throttleLimit - stepVal)):
    throttleVal += stepVal

  if scan_wrapper[pygame.K_LCTRL] and (throttleVal >= stepVal):
    throttleVal -= stepVal

  if scan_wrapper[pygame.K_d] and (rudderVal >= (stepVal - rudderLimit)):
    rudderVal -= stepVal

  if scan_wrapper[pygame.K_a] and (rudderVal <= (rudderLimit-stepVal)):
    rudderVal += stepVal

  ##
  # "fcs/elevator-cmd-norm"  # 升降舵
  # "fcs/rudder-cmd-norm"  # 方向舵
  # "fcs/throttle-cmd-norm"  # 油门
  # "fcs/flap-cmd-norm"  # 襟翼
  # "fcs/speedbrake-cmd-norm"  # 减速板
  # "fcs/spoiler-cmd-norm"  # 扰流片

  fdm["fcs/aileron-cmd-norm"] = aileronVal
  fdm["fcs/elevator-cmd-norm"] = elevatorVal
  fdm["fcs/throttle-cmd-norm[1]"] = throttleVal
  fdm["fcs/throttle-cmd-norm[2]"] = throttleVal
  fdm["fcs/throttle-cmd-norm[3]"] = throttleVal
  fdm["fcs/throttle-cmd-norm[4]"] = throttleVal
  # fdm["fcs/rudder-cmd-norm"] = rudderVal

  fdm["gear/gear-pos-norm"] = 1 #设置起落架（0-关，1-开）

  fdm.run()
  lat = 111320 * fdm['position/lat-geod-deg']
  long = 40075000 * fdm['position/long-gc-deg'] * math.cos(fdm['position/lat-geod-deg'] * (math.pi / 180.0)) / 360
  alt = fdm['position/h-sl-ft']

  # 角度
  # "attitude/psi-deg"  # Yaw 偏航角
  # "attitude/theta-deg"  # Pitch 俯仰角
  # "attitude/phi-deg"  # Roll 翻滚角

  pitch = fdm["attitude/theta-deg"]# Pitch angle initial condition in degrees
  roll = fdm["attitude/phi-deg"]# Roll angle initial condition in degrees
  yaw = fdm["attitude/psi-deg"]# Heading angle initial condition in degrees


  throttleVal_str = "[%f,%f,%f,%f]" %(fdm['fcs/throttle-cmd-norm[1]'], fdm['fcs/throttle-cmd-norm[2]'], fdm['fcs/throttle-cmd-norm[3]'], fdm['fcs/throttle-cmd-norm[4]'])
  control_str = "Aileron:%f\r\nElevator:%f\r\nThrottle:%s\r\nRudder:%f" % (aileronVal, elevatorVal, throttleVal_str, rudderVal)

  jsontstr = json.dumps({"x": lat, "y": long, "z": alt, "roll": roll, "yaw": yaw, "pitch": pitch, "controlStr": control_str})
  socketUdp.sendto(jsontstr.encode("utf-8"), (RemoteUdpAddress, RemoteUdpPort))
  #print(jsontstr)

socketUdp.close()
