#!/usr/bin/python3
#
# WatchObservatory.com - Andre Germain Dec 2024 - GPL licensing
# You must credit the above for conception of speckle position sensing for telescopes, and this software
#
# ROI: region of interest, or subset of the sensor for faster download and computation.
# ROI only needs to be so large to encompass sufficient speckle patterns.
#
# With OV9281 3rd party cameras, timeout may occur. Add the following line in rpi_apps.yaml using the command below it:
# "camera_timeout_value_ms":    5000000,
# sudo nano /usr/share/libcamera/pipeline/rpi/vc4/rpi_apps.yaml
#
# single frame ROI (not functional with OV9281)
#  rpicam-still -t 20000 --shutter 500 --analoggain 1 --roi .25,.25,.1,.1
#
# video of ROI
#  mono OV9281: (no ROI available)
#   rpicam-vid -t 20000 --shutter 40 --denoise off --sat 0 --analoggain 1 speckle.h264 --width 1280 --height 800
#
# To control light source brightness, pigpio was used as it has a more solid h/w based PWM as compared to GPIO
# You must start its daemon before running this script: sudo pigpiod

import time
from subprocess import Popen, PIPE
#import RPi.GPIO as GPIO
import pigpio
import numpy as np
import matplotlib.pyplot as plt
import struct
from PIL import Image as im
from picamera2 import Picamera2
from socket import socket, AF_INET, SOCK_DGRAM
from scipy import signal
from scipy import ndimage

debug = 1        # 1 is only graphs, 2 is graphs and jpg

cx = int(1280/2) # center of sensor OV9281 TODO: get from camera modes
cy = int(800/2)  # "
cxs = 128        # ROI pixel size (NOTE: use power of 2 for FFT correlator)
cys = cxs        # "

count = 0
ipn = 5         # ROI pipeline depth
roi = [[[0]*cxs]*cys]*ipn  # ROI pipeline
roib = [[0]*cxs]*cys       # ROI background
roi16 = np.zeros((cxs,cys), dtype=np.int16) # ROI scratch pad 16 bit wide pixels
DarkF = 0       # 'dark frame' as it were, to increase CM estimate quality
IGain = 1       # gain on dark adjusted image to fill 8 bit
x = [0]*ipn     # ROI pipeline center shift in X
y = [0]*ipn     # ROI pipeline center shift in Y
tm = [0]*ipn    # ROI pipeline times
pathx = []      # path for plotting
pathy = []

# precision delay (microseconds)
def Delay(uS):
  t0 = time.perf_counter_ns() + uS * 1000
  while (time.perf_counter_ns() < t0):
    pass

# procedure to download ROI from sensor
def GetROI(Shift2Zero):
  global exp
  global DarkF
  global roib
  global count

  cam.start()
  Delay(exp)
  img = cam.capture_array("main")
  cam.stop()
  roi = img[:,:,1][cx:cx+cxs,cy:cy+cys] - roib

  if (Shift2Zero):
    roi = np.clip((roi.astype(np.int16) - DarkF) * IGain, a_min=0, a_max=255)
    if (debug > 1):
      data = im.fromarray(roi.astype(np.uint8))
      data.save('speckle{0}.jpg'.format(count))

  return roi

# procedure to detect correlation peak and its centroid
def Centroid(img, sel):
  Rd = 1     # centroid radius window
  SumX = 0   # sum of X 'moments'
  SumY = 0   # sum of Y 'moments'
  SumM = 0   # sum of 'masses'
  SumMx = 0  # sum of 'masses' in X cross portion
  SumMy = 0  # sum of 'masses' in Y cross portion
  CMx = 0    # center of mass in X
  CMy = 0    # center of mass in Y

# img = img - np.min(img)   # lower to zero bottom of 2D shape
  w = img.shape             # fetch array size
  a0, b0 = np.unravel_index(img.argmax(), w)  # brightest cell
  if ((a0<Rd) or (a0>=(w[0]-Rd-1)) or (b0<Rd) or (b0>=(w[1]-Rd-1))):
    print('======== incorrect correlation for centroiding ========')
    print(a0, b0, w)
    return(0, 0)

  ctr = img[a0-Rd:a0+Rd+1,b0-Rd:b0+Rd+1]      # extract Rd radius region around brightest cell
  ctr = ctr - np.min(ctr)                     # lower to zero bottom of 2D shape
  if (0):                            # CM across full X & Y pixels
    for x in range(0, 2*Rd+1, 1):    # sweep X axis by Rd either side of brightest cell
      for y in range(0, 2*Rd+1, 1):  # sweep Y axis by Rd either side of brightest cell
        SumM += ctr[x][y]
        SumX += ctr[x][y] * x
        SumY += ctr[x][y] * y
    if (SumM > 0):
      CMx = SumX / SumM + a0 - Rd
      CMy = SumY / SumM + b0 - Rd
  else:                              # CM across X and Y axis (cross) only
    for x in range(0, 2*Rd+1, 1):    # sweep X axis by Rd either side of brightest cell
      SumMx += ctr[x][Rd]
      SumX  += ctr[x][Rd] * x
    for y in range(0, 2*Rd+1, 1):    # sweep Y axis by Rd either side of brightest cell
      SumMy += ctr[Rd][y]
      SumY  += ctr[Rd][y] * y

    if ((SumMx > 0) and (SumMy > 0)):
      CMx = SumX / SumMx + a0 - Rd
      CMy = SumY / SumMy + b0 - Rd

  if ((debug > 0) and (sel > 0)):
    print('----')
    print('cnt: {:2d} a0: {:2d} b0: {:2d} CMx: {:.2f} CMy: {:.2f}'.format(count, a0, b0, CMx, CMy))
#   print(ctr)

  return(CMx, CMy)

# adjust exposure before tracking
def SetExposure():
  global DarkF
  global IGain
  global exp
  global exp0

  for exp in range (7,45,1):                    # exposure range to test (in microseconds)
    cam.set_controls({"ExposureTime": exp})     # set camera exposure
    roi = GetROI(0)                             # snap ROI at set exposure
    hist = ndimage.histogram(roi, min = 0, max = 255, bins = 256) # histogram pixel brightness in 16 bins over 8 bit
    MxIdx = np.max(np.nonzero(hist))            # locate highest non empty bin (threshold to avoid hot pixels)
    MnIdx = np.min(np.nonzero(hist))            # locate lowest  non empty bin (threshold to avoid hot pixels)
    if (MxIdx == 255):
      break
    elif (MxIdx < 240):                         # latch exposure
      exp0 = exp

    if (debug):
      print('auto exposure: time {:.2f} uS index low/high {:2d}/{:2d}'.format(exp, MnIdx, MxIdx))

  #snap final image at target exposure and extract histogram extent
  cam.set_controls({"ExposureTime": exp0})     # set camera exposure
  roi = GetROI(0)                              # snap ROI at set exposure
  hist = ndimage.histogram(roi, min = 0, max = 255, bins = 256) # histogram pixel brightness in 16 bins over 8 bit
  MxIdx = np.max(np.nonzero(hist))             # locate highest non empty bin
  MnIdx = np.min(np.nonzero(hist))             # locate lowest  non empty bin
# DarkF = np.array(hist).argmax()              # lower cutoff (dark) for further imaging - maximum value index
# DarkF = int((MxIdx - MnIdx)/3 + MnIdx)       # lower cutoff (dark) for further imaging - portion of family
  DarkF = MnIdx

  print('auto exposure setting: {:.2f} uS index low/high {:3d}/{:3d}'.format(exp0, MnIdx, MxIdx))
  print('dark threshold: {:3d}'.format(DarkF))

  return int(exp0)

# START OF PROGRAM
# Initialize camera
exp0 = 7         # default exposure in microseconds
exp = exp0       # exposure in microseconds
cam = Picamera2()
modes = cam.sensor_modes
#camconfig = cam.create_still_configuration(main={'size': (cxs,cys)})   # ROI (no effect on OV9281)
camconfig = cam.create_still_configuration(main={'size': (1280,800)})
cam.configure(camconfig)
#cam.camera_configuration()['raw']
# monochrome camera, note: OV9281 3rd party doesn't seem to support ROI (ScalerCrop)
cam.set_controls({"AeEnable": False, "ExposureTime": exp0, "AnalogueGain": 1.0, "NoiseReductionMode": False})

#initialize socket
SERVER_IP = '192.168.2.154'
PORT_NUMBER = 50022
SrvSocketActive = 1
try:
  Socket = socket(AF_INET, SOCK_DGRAM)
  Socket.setblocking(False)  # prevent blocking
except:
  SrvSocketActive = 0

# Initialize ROIs & Correlator
xi = 0          # Integrated center shifts
yi = 0          # Integrated center shifts
x0 = int(cxs/2) # origin
y0 = int(cys/2) # origin
dt = 0
dx = 0
dy = 0
dxl = 0
dyl = 0
dtl = 0
HB = 0
HBSrv = 0
ipp = 0         # previous image pipeline index
ipc = 1         # current  image pipeline index
ipl = 2         # oldest   image pipeline index

# set light source control
p = Popen("sudo pigpiod", stdout=PIPE, stderr=PIPE, shell=True)  # start daemon (should use communicate() iso sleep)
time.sleep(2)
LightOn = 255  # PWM duty cycle 0 to 255 for 0 to 100%
LightOff = 0;
LightPin=12
gpio = pigpio.pi()
gpio.set_mode(LightPin, pigpio.OUTPUT)
gpio.set_PWM_frequency(LightPin,10000000)
gpio.set_PWM_dutycycle(LightPin,LightOff)
#gpio.write(LightPin,0)
#GPIO.setmode(GPIO.BCM)
#GPIO.setup(LightPin, GPIO.OUT)
#pwm = GPIO.PWM(LightPin, 100000)

# switch on light source for auto exposure (roib must be zero for this as roib depends on exposure)
#GPIO.output(LightPin, GPIO.HIGH)   # light source on
#pwm.start(25)
gpio.set_PWM_dutycycle(LightPin,LightOn)
time.sleep(.25)              # wait for light source to stabilize
exp = SetExposure()          # adjust exposure
cam.set_controls({"ExposureTime": exp})

# switch off light source to sample background frame (roib)
#GPIO.output(LightPin, GPIO.LOW)   # light source off
gpio.set_PWM_dutycycle(LightPin,LightOff)
time.sleep(.25)             # wait for light source to stabilize
bn = 10
for n in range (0, bn+1, 1):
  roi16 = np.add(roi16, GetROI(0).astype(np.int16))
roib = roi16 / bn           # average ROI background
DarkF = DarkF - np.min(roib)# need to remove dark frame ROI minimum

# sample correlation peak position for 'background' removal
# note: in final design, needs to be done when there is no motion,
# either detected by this code or told so by the server over socket
# compute correlation background
roi[0] = GetROI(0)
corr0 = signal.correlate(np.array(roi[0]).astype(float), np.array(roi[0]).astype(float), mode='same', method='fft')

# switch on light source for position sensing
#GPIO.output(LightPin, GPIO.HIGH)   # light source on
gpio.set_PWM_dutycycle(LightPin,LightOn)
time.sleep(.25)              # wait for light source to stabilize

# start image pipeline (with 'dark frame' substraction)
for n in range(0, ipn, 1):
  roi[n] = GetROI(1)

t0 = time.perf_counter() # time zero

# START OF REAL TIME LOOP
for n in range(0, 5*2): # approx 5 images per second - debugging/test phase of project
  tm[ipc]  = time.perf_counter() - t0  # image time stamp
  roi[ipc] = GetROI(1)                 # snap a new ROI (as current index)

  # compute correlation of previous and current ROI
  corrc = np.subtract(signal.correlate(np.array(roi[ipp]).astype(float), np.array(roi[ipc]).astype(float), mode='same', method='fft'), 0) #corr0)
  # compute centroid of correlation result to determine shift from origin between current and previous image
  x[ipc], y[ipc] = Centroid(corrc,0)
  dx = x[ipc] - x0        # shift from origin in X axis between previous and current ROI
  dy = y[ipc] - y0        # shift from origin in Y axis between previous and current ROI
  xi = xi + dx            # integrated shifts in X axis (motion in X)
  yi = yi + dy            # integrated shifts in Y axis (motion in Y)
  dt = tm[ipc] - tm[ipp]  # elapsed time between previous and current ROI

  # compute correlation of oldest and current RIO and floor to lowest value
  corrl = np.subtract(signal.correlate(np.array(roi[ipl]).astype(float), np.array(roi[ipc]).astype(float), mode='same', method='fft'), 0) #corr0)
  # compute centroid of correlation result to determine shift from origin between current and oldest image
  x[ipl], y[ipl] = Centroid(corrl,1)
  dxl = (x[ipl] - x0) #/ (ipn-1)        # shift from origin in X axis between oldest and current ROI
  dyl = (y[ipl] - y0) #/ (ipn-1)        # shift from origin in Y axis between oldest and current ROI
  dtl = (tm[ipc] - tm[ipl]) #/ (ipn-1)  # elapsed time between oldest and current ROI

  if (debug): # and (OriginFound != 0)):
    pathx.append(xi)
    pathy.append(yi)
#   print ('dt:{:6.3f} dx:{:6.3f} dy:{:6.3f} xi:{:6.2f} yi:{:6.2f}'.format(dt, dx, dy, xi, yi))
    if (debug > 1):
      data = im.fromarray((corrl/np.max(corrl)*255).astype(np.uint8))
      data.save('corr{0}.jpg'.format(count))
      count += 1

  # slow down process
  time.sleep(.1)

  # Handle server/client traffic
  if (SrvSocketActive):
    # Data to receive
    try:
      buf = Socket.recv(256)
      ChkSum = 0
      for i in buf:
        ChkSum = ChkSum + i
      dataT = struct.unpack('<III', buf)
      ChkSumSrv = dataT[2]
      if (ChkSum == ChkSumSrv * 2):  # server checksum is included in summation!
        (HBSrv, StatusSrv, ChkSumSrv) = dataT
      else:
        print("bad checksum", ChkSum, ChkSumSrv)
    except:
      pass

    HB = HB + 1     # local heart beat
    HB = HB & 0xff

    # Data to send
    Status = 0      # local status, TODO
    ChkSum = 0
    buf = struct.pack('<IIIfffffffff',HB,HBSrv,Status,tm[ipc],dt,dtl,dx,dxl,dy,dyl,xi,yi)
#   buf = struct.pack('<IIIfffffffff',HB,HBSrv,Status,tm[ipc],dt,dtl,dx,dxl,dy,dyl,float(exp/1000),yi)
    for i in buf:
      ChkSum = ChkSum + i
    buf = buf[:len(buf)] + struct.pack('<I',ChkSum)
    Socket.sendto(buf,(SERVER_IP,PORT_NUMBER))

  # advance pipeline image indexes for next iteration
  ipp = ipc       # previous pipeline index
  ipc = ipc + 1   # current  pipeline index
  if (ipc >= ipn): ipc = 0
  ipl = ipc + 1   # oldest pipeline index
  if (ipl >= ipn): ipl = 0

gpio.set_PWM_dutycycle(LightPin,LightOff)
gpio.stop()
p = Popen("sudo killall pigpiod", stdout=PIPE, stderr=PIPE, shell=True)

#GPIO.output(LightPin, GPIO.LOW)   # shutoff light source
#GPIO.cleanup()

# numpy debug graphs
if (debug):
  if (1):
    plt.subplots_adjust(hspace=.5)

    plt.subplot(3,2,1)
    plt.title("previous ROI")
    plt.imshow(roi[ipl], cmap='gray', origin='lower')

    plt.subplot(3,2,2)
    plt.title("current ROI")
    plt.imshow(roi[ipc], cmap='gray', origin='lower')

    plt.subplot(3,2,3)
    plt.title("correlation")
    plt.imshow(corrl, cmap='gray', origin='lower')

    plt.subplot(3,2,4)
    plt.title("intensity histogram")
    plt.yscale('log')
    plt.plot(ndimage.histogram(roi[ipc],min=0,max=255,bins=256))

#   plt.subplot(3,2,5)
#   plt.title("correlation histogram")
#   plt.yscale('log')
#   maxc = np.amax(corrc)
#   hcorr = ndimage.histogram(corrc,min=0,max=maxc,bins=100)
#   plt.plot([i*maxc/100 for i in range(len(hcorr))], hcorr)

    plt.subplot(3,2,6)
    plt.title("speckle motion")
    plt.scatter(pathx,pathy,s=1)
    plt.axis((-200,200,-200,200))

    X = np.arange(0,cxs,1)
    Y = np.arange(0,cys,1)
    X, Y = np.meshgrid(X, Y)
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    ax.plot_surface(X, Y, roi[ipl], cmap='Purples')
    ax.set_xlim(0,cxs)
    ax.set_ylim(0,cys)
    ax.set_zlim(0,255)

    X = np.arange(0,cxs,1)
    Y = np.arange(0,cys,1)
    X, Y = np.meshgrid(X, Y)
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    ax.plot_surface(X, Y, roi16, cmap='Purples')
    ax.set_xlim(0,cxs)
    ax.set_ylim(0,cys)
    ax.set_zlim(0,np.max(roi16))

    X = np.arange(50,78,1)
    Y = np.arange(50,78,1)
    X, Y = np.meshgrid(X, Y)
    fig, bx = plt.subplots(subplot_kw={"projection": "3d"})
    corrlz = corrl[50:78,50:78]
    bx.plot_surface(X, Y, corrlz, cmap='Purples')
#    bx.set_xlim(40,84)#cxs)
#    bx.set_ylim(40,84)#cys)
#    bx.set_zlim(0,np.max(corrlz))

#    X = np.arange(0,cxs,1)
#    Y = np.arange(0,cys,1)
    X = np.arange(50,78,1)
    Y = np.arange(50,78,1)
    X, Y = np.meshgrid(X, Y)
    fig, cx = plt.subplots(subplot_kw={"projection": "3d"})
    corr0z = corr0[50:78,50:78]
    cx.plot_surface(X, Y, corr0z, cmap='Purples')
#    cx.set_xlim(40,84)#cxs)
#    cx.set_ylim(40,84)#cys)
#    cx.set_zlim(0,np.max(corrlz))

    plt.show()

#sys.exit()