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
import RPi.GPIO as GPIO
import numpy as np
import matplotlib.pyplot as plt
import struct
from PIL import Image as im
from picamera2 import Picamera2
from socket import socket, AF_INET, SOCK_DGRAM
from scipy import signal
from scipy import ndimage

debug = 1        # 1 is only graphs, 2 is graphs and jpg

SpecklePosFlatFrame = 'SpecklePosFlatFrame.npy'  # flat fiel file name

cx = int(1280/2) # center of sensor OV9281 TODO: get from camera modes
cy = int(800/2)  # "
cxs = 128        # ROI pixel size (NOTE: use power of 2 for FFT correlator)
cys = cxs        # "

count = 0
ipn = 5         # ROI pipeline depth
roi   = [np.zeros((cxs,cys), dtype=float)] * ipn  # ROI pipeline
roib  = np.zeros((cxs,cys), dtype=float)          # ROI background
roiff = np.ones((cxs,cys), dtype=float)           # ROI flat field
roiff_= np.ones((cxs,cys), dtype=float)           # ROI flat field accumlator
roix  = np.zeros((cxs,cys), dtype=float)          # ROI scratch pad
DarkF = 0       # 'dark field' as it were, to increase CM estimate quality
x = [0]*ipn     # ROI pipeline center shift in X
y = [0]*ipn     # ROI pipeline center shift in Y
tm = [0]*ipn    # ROI pipeline times
ROIFFCnt = 0    # ROI flat field accumulation counter
pathx = []      # path for plotting
pathy = []

# precision delay (microseconds)
def Delay(uS):
  tm = time.perf_counter_ns() + uS * 1000
  while (time.perf_counter_ns() < tm):
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
  roi = img[:,:,1][cx:cx+cxs,cy:cy+cys].astype(float) - roib

  if (Shift2Zero):
    roi = np.clip((roi - DarkF), a_min=0, a_max=255)
    if (debug > 1):
      data = im.fromarray(roi.astype(np.uint8))
      data.save('speckle{0}.jpg'.format(count))

  return roi

# procedure to detect correlation peak and its centroid
def Centroid(roi, sel):
  Rd = 1     # centroid radius window
  SumX = 0   # sum of X 'moments'
  SumY = 0   # sum of Y 'moments'
  SumM = 0   # sum of 'masses'
  SumMx = 0  # sum of 'masses' in X cross portion
  SumMy = 0  # sum of 'masses' in Y cross portion
  CMx = 0    # center of mass in X
  CMy = 0    # center of mass in Y

  w = roi.shape             # fetch array size
  a0, b0 = np.unravel_index(roi.argmax(), w)  # brightest cell
  if ((a0<Rd) or (a0>=(w[0]-Rd-1)) or (b0<Rd) or (b0>=(w[1]-Rd-1))):
    print('======== incorrect correlation for centroiding ========')
    print(a0, b0, w)
    return(0, 0)

  ctr = roi[a0-Rd:a0+Rd+1,b0-Rd:b0+Rd+1]      # extract Rd radius region around brightest cell
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

  return(CMx, CMy)

# adjust exposure before tracking
def SetExposure():
  global DarkF
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

  # snap final image at target exposure and extract histogram for bias
  cam.set_controls({"ExposureTime": exp0})     # set camera exposure
  print('auto exposure setting: {:.2f} uS index low/high {:3d}/{:3d}'.format(exp0, MnIdx, MxIdx))
  return int(exp0)

# compute bias via histogram
def GetBias(roi):
  hist = ndimage.histogram(roi, min = 0, max = 255, bins = 256) # histogram pixel brightness in 16 bins over 8 bit
  MxIdx = np.max(np.nonzero(hist))             # locate highest non empty bin
  MnIdx = np.min(np.nonzero(hist))             # locate lowest  non empty bin
# bias =  np.array(hist).argmax()              # lower cutoff (dark) for further imaging - maximum value index
  bias  = int((MxIdx - MnIdx)/3 + MnIdx)       # lower cutoff (dark) for further imaging - portion of family
# bias  = MnIdx                                # lower cutoff (dark) for further imaging
# bias  = 0
  return int(bias)

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
x0 = int(cxs/2) # ROI origin in sensor frame
y0 = int(cys/2) # ROI origin in sensor frame
dt = 0          # delta x change between previous and current ROIs
dx = 0          # delta y change between previous and current ROIs
dy = 0          # delta time between previous and current ROIs
dxl = 0         # delta x change between ROI n and n-m
dyl = 0         # delta y change between ROI n and n-m
dtl = 0         # delta time between ROI n and n-m
HB = 0          # local socket data checksum
HBSrv = 0       # server socket data checksum
RateSrv = 0     # server motor rate
ipp = 0         # previous image pipeline index
ipc = 1         # current  image pipeline index
ipl = 2         # oldest   image pipeline index

# set light source control
LightPin=12
GPIO.setmode(GPIO.BCM)
GPIO.setup(LightPin, GPIO.OUT)

# switch on light source for auto exposure (roib must be zero for this as roib depends on exposure)
GPIO.output(LightPin, GPIO.HIGH)   # light source on
time.sleep(.25)              # wait for light source to stabilize
exp = SetExposure()          # adjust exposure
cam.set_controls({"ExposureTime": exp})

# switch off light source to sample background frame (roib)
GPIO.output(LightPin, GPIO.LOW)   # light source off
time.sleep(.25)             # wait for light source to stabilize
bn = 50
for n in range (0, bn+1, 1):
  roix = np.add(roix, GetROI(0))
roib = roix / bn           # average ROI background
#DarkF = DarkF - np.min(roib)# need to remove ROI minimum from DarkF

# load flat frame data
try:
  roiff = np.load(SpecklePosFlatFrame)
  print('Flat Frame data from disk X / Y', roiff.shape)
except:
  print('Flat Frame failure to load data')

# sample correlation peak position for 'background' removal
# note: in final design, needs to be done when there is no motion,
# either detected by this code or told so by the server over socket
# compute correlation background
#roi[0] = GetROI(0)
#corr0 = signal.correlate(roi[0], roi[0], mode='same', method='fft')

# switch on light source for position sensing
GPIO.output(LightPin, GPIO.HIGH)   # light source on
time.sleep(.25)              # wait for light source to stabilize

# repeat exposure settings after backgroud frame was computed
DarkF = GetBias(GetROI(0))
print('dark threshold: {:3d}'.format(DarkF))

# start image pipeline (with 'dark frame' substraction)
for n in range(0, ipn, 1):
  roi[n] = GetROI(1)

# zero time
t0 = time.perf_counter();

# START OF REAL TIME LOOP
for n in range(0, 4*480): # approx 5 images per second - debugging/test phase of project
  tm[ipc]  = time.perf_counter() - t0  # image time stamp
  roi[ipc] = GetROI(1)                 # snap a new ROI (as current index)

  # flat frame - must do this on ROI before these are flat framed!
  if (RateSrv != 0):   # TODO, logic on HBSrv
    if (ROIFFCnt < 1000):
      ROIFFCnt = ROIFFCnt + 1
      roiff_ = roiff_ + roi[ipc]
  roiraw = roi[ipc]   # for graphing (debug)
  roi[ipc] = np.multiply(roi[ipc], roiff)  # apply flat field

  # compute correlation of previous and current ROI
  corrc = np.subtract(signal.correlate(roi[ipp], roi[ipc], mode='same', method='fft'), 0)
  # compute centroid of correlation result to determine shift from origin between current and previous image
  x[ipc], y[ipc] = Centroid(corrc,0)
  dx = x[ipc] - x0        # shift from origin in X axis between previous and current ROI
  dy = y[ipc] - y0        # shift from origin in Y axis between previous and current ROI
  xi = xi + dx            # integrated shifts in X axis (motion in X)
  yi = yi + dy            # integrated shifts in Y axis (motion in Y)
  dt = tm[ipc] - tm[ipp]  # elapsed time between previous and current ROI

  # compute correlation of oldest and current RIO and floor to lowest value
  corrl = np.subtract(signal.correlate(roi[ipl], roi[ipc], mode='same', method='fft'), 0)
  # compute centroid of correlation result to determine shift from origin between current and oldest image
  x[ipl], y[ipl] = Centroid(corrl,1)
  dxl = (x[ipl] - x0) / (ipn-1)        # shift from origin in X axis between oldest and current ROI
  dyl = (y[ipl] - y0) / (ipn-1)        # shift from origin in Y axis between oldest and current ROI
  dtl = (tm[ipc] - tm[ipl]) / (ipn-1)  # elapsed time between oldest and current ROI

  if (debug):
    pathx.append(xi)
    pathy.append(yi)
    print ('dt:{:6.3f} dx:{:6.3f} dy:{:6.3f} xi:{:6.2f} yi:{:6.2f}'.format(dt, dx, dy, xi, yi))
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
      buf = bytearray(Socket.recv(32))
      dataT = struct.unpack('<IIII', buf)
      ChkSum = sum(buf[0:12])
      ChkSumSrv = dataT[3]
      if (ChkSum == ChkSumSrv):
        (HBSrv, StatusSrv, RateSrv, ChkSumSrv) = dataT
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

  # waste time to achieve better determinism
  TimeCorr = 0.25 - (time.perf_counter() - t0 - tm[ipc])
  if (TimeCorr > 0):
    Delay(TimeCorr * 1000000)   # pad to 0.25 sec

  # advance pipeline image indexes for next iteration
  ipp = ipc       # previous pipeline index
  ipc = ipc + 1   # current  pipeline index
  if (ipc >= ipn): ipc = 0
  ipl = ipc + 1   # oldest pipeline index
  if (ipl >= ipn): ipl = 0

# average flat frames (need to move this in loop above)
if (ROIFFCnt > 0):
  roiff = roiff_ / ROIFFCnt
  avgpix = np.mean(roiff)
  if ((avgpix > 0) and (np.min(roiff) > 0)):
    roiff = avgpix / roiff
    print('Flat Frame: accm frames / mean pix', ROIFFCnt, avgpix)
    np.save(SpecklePosFlatFrame, roiff)
  else:
    print('====== Flat Frame fail: mean or one of more pixels zero')
else:
  print('====== Flat Frame fail: no flat frames accumulated;')

GPIO.output(LightPin, GPIO.LOW)   # shutoff light source
GPIO.cleanup()

# numpy debug graphs
if (debug):
  if (1):
    plt.subplots_adjust(hspace=.5)

    plt.subplot(2,2,1)
    plt.title("previous ROI")
    plt.imshow(roi[ipl], cmap='gray', origin='lower')

    plt.subplot(2,2,2)
    plt.title("correlation")
    plt.imshow(corrl, cmap='gray', origin='lower')

    plt.subplot(2,2,3)
    plt.title("intensity histogram")
    plt.yscale('log')
    plt.plot(ndimage.histogram(roi[ipc],min=0,max=255,bins=256))

    plt.subplot(2,2,4)
    plt.title("speckle motion")
    plt.scatter(pathx,pathy,s=1)
    plt.axis((-400,400,-400,400))

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
    fig, bx = plt.subplots(subplot_kw={"projection": "3d"})
    bx.plot_surface(X, Y, roiraw, cmap='Purples')
    bx.set_xlim(0,cxs)
    bx.set_ylim(0,cys)
    bx.set_zlim(0,255)

    X = np.arange(0,cxs,1)
    Y = np.arange(0,cys,1)
    X, Y = np.meshgrid(X, Y)
    fig, cx = plt.subplots(subplot_kw={"projection": "3d"})
    cx.plot_surface(X, Y, roiff, cmap='Purples')
    cx.set_xlim(0,cxs)
    cx.set_ylim(0,cys)
    cx.set_zlim(0,np.max(roiff))

    X = np.arange(50,78,1)
    Y = np.arange(50,78,1)
    X, Y = np.meshgrid(X, Y)
    fig, dx = plt.subplots(subplot_kw={"projection": "3d"})
    corrlz = corrl[50:78,50:78]
    dx.plot_surface(X, Y, corrlz, cmap='Purples')

#   X = np.arange(50,78,1)
#   Y = np.arange(50,78,1)
#   X, Y = np.meshgrid(X, Y)
#   fig, ex = plt.subplots(subplot_kw={"projection": "3d"})
#   corr0z = corr0[50:78,50:78]
#   ex.plot_surface(X, Y, corr0z, cmap='Purples')

    plt.show()

#sys.exit()