#!/usr/bin/python3
#
# WatchObservatory.com - Andre Germain Feb 2025 - GPL licensing
# You must credit the above for conception of speckle position sensing for telescopes, and this software
#
# ROI: region of interest, or subset of the sensor for faster download and computation.
# ROI only needs to be so large to encompass sufficient speckle patterns.
#
# Requirements:
#  O/S Bookworm or later
#  libcamera2
#  Rpi4 or Zero2W
#
# With OV9281 3rd party camera:
#  a) set default driver by adding dtoverlay=ov9281 in config.txt with the following,
#     sudo nano /boot/config.txt
#  b) Timeout may occur. Add the following line in rpi_apps.yaml using the command below it:
#     "camera_timeout_value_ms":    5000000,
#     sudo nano /usr/share/libcamera/pipeline/rpi/vc4/rpi_apps.yaml
#
# single frame ROI
#  rpicam-still -t 20000 --shutter 500 --analoggain 1 --roi .25,.25,.1,.1
#  rpicam-still --raw --shutter 7 --timeout 20000 --width 1280 --height 800
#
# video of ROI
#  mono OV9281:
#   rpicam-vid -t 20000 --shutter 40 --denoise off --sat 0 --analoggain 1 speckle.h264 --width 1280 --height 800
#   rpicam-vid -t 240s --width 1280 --height 800 --shutter 10 -o raw.h264 --nopr
#
# light source is controlled via GPIO library

import time
from subprocess import Popen, PIPE
import RPi.GPIO as GPIO
import math
import numpy as np
import matplotlib.pyplot as plt
import struct
from PIL import Image as im
from picamera2 import Picamera2
from socket import socket, AF_INET, SOCK_DGRAM
from scipy import signal
from scipy import ndimage
from skimage.registration import phase_cross_correlation
from skimage.registration._phase_cross_correlation import _upsampled_dft
from scipy.ndimage import fourier_shift
from skimage.registration import optical_flow_tvl1, optical_flow_ilk

import os

os.environ['LIBCAMERA_LOG_LEVELS'] = '4'          # silence libcamera2

debug = 1                                         # 1 is only graphs, 2 is graphs and jpg
RunFramesToDo = 4*2400                            # for loop frames to run (4/sec), will remove in final version
TimeBetweenSamples = .25                          # time in seconds betwen samples, dependent on processor power and imager
count = 0
SpecklePosFlatFrame = 'SpecklePosFlatFrame.npy'   # flat fiel file name
ipn = 5                                           # ROI pipeline depth
cxs = 64                                          # ROI pixel size (NOTE: use power of 2 for FFT correlator)
cys = cxs                                         # " (must same as cxs for fft)
roi   = [np.zeros((cxs,cys), dtype=float)] * ipn  # ROI pipeline
roib  = np.zeros((cxs,cys), dtype=float)          # ROI background
roiff = np.ones((cxs,cys), dtype=float)           # ROI flat field
roiff_= np.ones((cxs,cys), dtype=float)           # ROI flat field accumulator
roix  = np.zeros((cxs,cys), dtype=float)          # ROI scratch pad
DarkF = 0                                         # 'dark field' as it were, to increase CM estimate quality
x = [0]*ipn                                       # ROI pipeline center shift in X
y = [0]*ipn                                       # ROI pipeline center shift in Y
tm = [0]*ipn                                      # ROI pipeline times
ROIFFCnt = 0                                      # ROI flat field accumulation counter
ns2ms  = 1/1000000                                # milliseconds in a nanosecond
ns2sec = 1/1000000000                             # seconds in a nanoseconds
us2sec = 1/1000000                                # seconds in a microsecond
us2ns  = 1000                                     # nanoseconds in a microsecond
sec2us = 1000000                                  # microseconds in a second

# precision delay (microseconds)
def Delay(uS):
  tm = time.perf_counter_ns() + uS * us2ns
  while (time.perf_counter_ns() < tm):
    pass

# procedure to download ROI from sensor
def GetROI(Shift2Zero):
  global DarkF
  global roib
  global count

# img10b = (img[:, 0::5] << 2) | (img[:, 1::5] >> 6)
# when camera does not support ROI, so full frame ROI array extract
# roi = img[:,:,1][cxo:cxo+cxs,cyo:cyo+cys].astype(float) - roib  # main
# roi = img10b[cxo:cxo+cxs,cyo:cyo+cys].astype(float) - roib  #raw
# roi = img[cxo:cxo+cxs,cyo:cyo+cys].astype(float) - roib  #raw

  # stack images to improve SNR
  img1 = cam.capture_array("main")[:,:,1].astype(float)    # snap ROI for client
  img2 = cam.capture_array("main")[:,:,1].astype(float)    # snap ROI for client
  img3 = cam.capture_array("main")[:,:,1].astype(float)    # snap ROI for client
# img4 = cam.capture_array("main")[:,:,1].astype(float)    # snap ROI for client
# roi = (img1 + img2 + img3 + img4)/4 - roib               # average stack and remove background
  roi = (img1 + img2 + img3)/3 - roib                      # average stack and remove background
  timestamp = cam.capture_metadata()["SensorTimestamp"]

  if (Shift2Zero): roi = np.clip((roi - DarkF), a_min=0, a_max=255)

  return roi, timestamp

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

  for exp in range (10,60,5):                   # exposure range to test (in microseconds)
    if (exp == 67):
      continue
    cam.set_controls({"ExposureTime": exp})     # set camera exposure
    roi,_ = GetROI(0)                           # snap ROI at set exposure
    hist = ndimage.histogram(roi, min = 0, max = 255, bins = 256) # histogram pixel brightness in 16 bins over 8 bit
    MxIdx = np.max(np.nonzero(hist))            # locate highest non empty bin (threshold to avoid hot pixels)
    MnIdx = np.min(np.nonzero(hist))            # locate lowest  non empty bin (threshold to avoid hot pixels)
    time.sleep(.1)
    if (MxIdx == 255):
       print('break:   {:.2f} uS index low/high {:3d}/{:3d}'.format(exp, MnIdx, MxIdx))
       break
    elif (MxIdx < 240):                         # latch exposure
      exp0 = exp
      print('less240: {:.2f} uS index low/high {:3d}/{:3d}'.format(exp, MnIdx, MxIdx))
    else:
      print('240-255: {:.2f} uS index low/high {:3d}/{:3d}'.format(exp, MnIdx, MxIdx))

  # snap final image at target exposure and extract histogram for bias
  cam.set_controls({"ExposureTime": exp0})     # set camera exposure
  print('auto exposure: {:.2f} uS index low/high {:3d}/{:3d}'.format(exp0, MnIdx, MxIdx))
  return int(exp0)

# compute bias via histogram
def GetBias(roi):
  hist = ndimage.histogram(roi, min = 0, max = 255, bins = 256) # histogram pixel brightness in 16 bins over 8 bit
  MxIdx = np.max(np.nonzero(hist))             # locate highest non empty bin
  MnIdx = np.min(np.nonzero(hist))             # locate lowest  non empty bin
  bias  = int((MxIdx - MnIdx)/3 + MnIdx)       # lower cutoff (dark) for further imaging - portion of family
  return int(bias)

###############################################################################################
# START OF PROGRAM

# Initialize camera
exp0 = 7         # default exposure in microseconds
exp = exp0       # exposure in microseconds
cam = Picamera2()
modes = cam.sensor_modes
cx,cy = cam.camera_properties['PixelArraySize']
print("Camera native resolution: ",cx,"x",cy)
cx  = 1280       # X pixel count TODO: get from camera
cy  = 800        # Y pixle count
cxo = int(cx/2)  # center of sensor
cyo = int(cy/2)  # "
#cam.still_configuration.enable_raw()
#cam.still_configuration.main.size = (cxs, cys)
#cam.still_configuration.queue = False;
#cam.configure(cam.create_still_configuration())
camconfig = cam.create_still_configuration(main={'size': (cxs,cys)}, queue=False)   # size same as ROI
#camconfig = cam.create_still_configuration(raw={'size': (cxs,cys),'format': 'R8'}, queue=False)
cam.configure(camconfig)
crop = (cxo, cyo, cxs, cys)
cam.set_controls({"AeEnable": False, "ExposureTime": exp0, "AnalogueGain": 1.0, "NoiseReductionMode": False, "ScalerCrop": crop})
cam.start()

# load flat frame data
try:
  roiff = np.load(SpecklePosFlatFrame)
  print('Flat Frame data from disk X x Y', roiff.shape)
except:
  print('Flat Frame failure to load data')

if (roiff.shape != (cxs,cys)):
  print('Flat frame data from disk not compatible with ROI, creating new one')
  np.resize(roiff, (cxs,cys))
  roiff = np.ones((cxs,cys))

#initialize socket
SERVER_IP = '192.168.2.154'
PORT_NUMBER = 50022
SrvSocketActive = 1
try:
  Socket = socket(AF_INET, SOCK_DGRAM)
  Socket.setblocking(False)  # prevent blocking
except:
  print("ERROR: ethernet socket failure <<<<<<<<<<<<")
  SrvSocketActive = 0

# Initialize ROIs & Correlator
xi = 0          # Integrated center shifts
yi = 0          # Integrated center shifts
x0 = int(cxs/2) # ROI origin in sensor frame
y0 = int(cys/2) # ROI origin in sensor frame
dt = 0          # delta x change between previous and current ROIs (seconds)
dtl = 0         # delta time between ROI n and n-m (seconds)
dx = 0          # delta y change between previous and current ROIs
dy = 0          # delta time between previous and current ROIs
dxl = 0         # delta x change between ROI n and n-m
dyl = 0         # delta y change between ROI n and n-m
HB = 0          # local socket data checksum
HBSrv = 0       # server socket data checksum
RateSrv = 0     # server motor rate
ipp = 0         # previous image pipeline index
ipc = 1         # current  image pipeline index
ipl = 2         # oldest   image pipeline index

# ROI windows for fft
#win1d = signal.windows.tukey(cxs, alpha=.2, sym=True)  # TODO, should make with cxs and cys as they may not be the same
#win2d = np.sqrt(np.outer(win1d, win1d))

# set light source control
LightPin=12
GPIO.setmode(GPIO.BCM)
GPIO.setup(LightPin, GPIO.OUT)

# switch on light source for auto exposure (roib must be zero for this as roib depends on exposure)
GPIO.output(LightPin, GPIO.HIGH) # light source on
time.sleep(.25)                  # wait for light source to stabilize
exp = SetExposure()              # adjust exposure

# switch off light source to sample background frame (roib)
GPIO.output(LightPin, GPIO.LOW)  # light source off
time.sleep(.25)                  # wait for light source to stabilize
bn = 25
for n in range (0, bn+1, 1):
  roix = np.add(roix, GetROI(0)[0])
roib = roix / bn                 # average ROI background

# switch on light source for position sensing
GPIO.output(LightPin, GPIO.HIGH) # light source on
time.sleep(.25)                  # wait for light source to stabilize

# compute bias now with background frame
DarkF = GetBias(GetROI(1)[0])
print('dark threshold: {:3d}'.format(DarkF))

# start image pipeline with 'dark frame' substraction
for n in range(0, ipn, 1):
  roi[n],tm[n] = GetROI(1)

# zero time
t0 = time.perf_counter_ns();

#################################################################################
# START OF 'REAL' TIME
for n in range(0, RunFramesToDo):
  roi[ipc],tm[ipc] = GetROI(1)  # snap 'current' ROI

  # flat frame - do this on ROI before these are flat framed!
  if (RateSrv != 0):   # TODO, logic on HBSrv
    if (ROIFFCnt < 1000):
      ROIFFCnt += 1
      roiff_ += roi[ipc]
    elif (ROIFFCnt == 1000):
      # average flat frames
      roiff = roiff_ / ROIFFCnt
      avgpix = np.mean(roiff)
      ROIFFCnt = 1001  # save only once!
      if ((avgpix > 0) and (np.min(roiff) > 0)):
        roiff = avgpix / roiff
        print('Flat Frame: accm frames / mean pix', ROIFFCnt, avgpix)
        np.save(SpecklePosFlatFrame, roiff)
      else:
        print('====== Flat Frame fail: mean, one or more pixels zero')

# roi[ipc] = np.multiply(roi[ipc], roiff)  # apply flat field - DO NOT MOVE above flat frame accumulator!

  # METHOD 1: compute correlation (previous, current), (oldest, current) ROI
  # corrc = np.subtract(signal.correlate(roi[ipp], roi[ipc], mode='same', method='fft'), 0)
  # corrl = np.subtract(signal.correlate(roi[ipl], roi[ipc], mode='same', method='fft'), 0)
  # compute centroid of correlations result to determine shift from origin between image pair
  # x[ipc], y[ipc] = Centroid(corrc,0)
  # x[ipl], y[ipl] = Centroid(corrl,1)
  # dx = x[ipc] - x0                   # shift from origin in X axis between previous and current ROI
  # dy = y[ipc] - y0                   # shift from origin in Y axis between previous and current ROI
  # dxl = (x[ipl] - x0)                # shift from origin in X axis between oldest and current ROI
  # dyl = (y[ipl] - y0)                # shift from origin in Y axis between oldest and current ROI
  # dtl = (tm[ipc] - tm[ipl]) * ns2sec # elapsed time between oldest and current ROI

  # METHOD 2: compute phase shift in frequency domain and return to spatial domain for linear shift, both axes
  # fftp = np.fft.rfft2(np.multiply(roi[ipp],window2d))  # fft of previous ROI
  # fftc = np.fft.rfft2(np.multiply(roi[ipc],window2d))  # fft of current  ROI
  # fftp = np.fft.rfft2(roi[ipl])           # fft of previous ROI
  # fftc = np.fft.rfft2(roi[ipc])           # fft of current  ROI
  # R = np.multiply(fftp, np.conj(fftc))    # complex cross product of ffts (conjugate on second)
  # r = np.fft.fftshift(np.fft.irfft2(R))   # shift zero freq to center
  # dxf, dyf = Centroid(np.abs(r),0)        # centroid shifted peak
  # dxf -= int(cxs/2)                       # shift from zero
  # dyf -= int(cys/2)

  # METHOD 3: phase cross correlationmask = corrupted_pixels
  # shift, error, diffphase = phase_cross_correlation(roi[ipl], roi[ipc], normalization='phase', upsample_factor=100)

  # METHOD 4: optical flow (vectors)
  Vx, Vy = optical_flow_ilk(roi[ipl], roi[ipc], radius=4, gaussian=False, prefilter=False)
  dx = np.mean(Vx)
  dy = np.mean(Vy)
  Vmag = math.sqrt(dx**2 + dy**2)

  # set shift, integrated positions and time deltas
  xi = xi + dx                       # integrated shifts in X axis (motion in X)
  yi = yi + dy                       # integrated shifts in Y axis (motion in Y)
  dt = (tm[ipc] - tm[ipl]) * ns2sec  # elapsed time between previous and current ROI

  if (debug): print ('dt:{:6.3f} dx:{:6.3f} dy:{:6.3f}'.format(dt, dx, dy))

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
    buf = struct.pack('<IIIffffff',HB,HBSrv,Status,tm[ipc],dt,dx,dy,xi,yi)
    for i in buf:
      ChkSum = ChkSum + i
    buf = buf[:len(buf)] + struct.pack('<I',ChkSum)
    Socket.sendto(buf,(SERVER_IP,PORT_NUMBER))

  # advance pipeline image indexes for next iteration
  ipp  = ipc               # previous pipeline index
  ipc += 1                 # current  pipeline index
  if (ipc >= ipn): ipc = 0 # fold back to start if at end of pipeline
  ipl += 1                 # oldest pipeline index
  if (ipl >= ipn): ipl = 0 # fold back to start if at end of pipeline

  if (debug): count += 1

  # waste time to achieve better determinism (pad to 0.25 sec)
  TimeCorr = TimeBetweenSamples - (time.perf_counter_ns() - t0) * ns2sec
  if (TimeCorr > 0):
    Delay(TimeCorr * sec2us)

# print('Corr:{:6.3f} it:{:6.3f}'.format(TimeCorr, (time.perf_counter_ns() - t0) * ns2sec))
  t0 = time.perf_counter_ns()  # end of frame time

GPIO.output(LightPin, GPIO.LOW)   # shutoff light source
GPIO.cleanup()
cam.stop()

# numpy debug graphs
#      data = im.fromarray(roi.astype(np.uint8)*2)
#      data.save('speckle{0}.jpg'.format(count))
if (debug):
  plt.subplots_adjust(hspace=.5)

  plt.subplot(2,2,1)
  plt.title("Current ROI")
  plt.imshow(roi[ipc], cmap='gray', origin='lower')

  plt.subplot(2, 2, 2)
  plt.title("Optical flow")
  nvec = 10  # Number of vectors to be displayed along each image dimension
  norm = np.sqrt(Vx**2 + Vy**2)
  nl, nc = roi[ipc].shape
  step = max(nl // nvec, nc // nvec)
  y, x = np.mgrid[:nl:step, :nc:step]
  u_ = Vy[::step, ::step]
  v_ = Vx[::step, ::step]
  plt.quiver(x, y, u_, v_, cmap='gray', color='y', units='dots', angles='xy', scale_units='xy', lw=5)
  plt.imshow(norm)

  plt.subplot(2,2,3)
  plt.title("intensity histogram")
  plt.yscale('log')
  plt.plot(ndimage.histogram(roi[ipc],min=0,max=255,bins=256))

  Vm = np.sqrt(np.square(Vx) + np.square(Vy))
  print('Mean vector magnitude: {:3f}'.format(np.mean(Vm)))
  VRange = 5   # pixel range on X ax
  VBins = 100
  VmHist = ndimage.histogram(Vm,min=0,max=VRange,bins=VBins)
  plt.subplot(2,2,4)
  xaxis = np.arange(0, VRange, step=VRange/VBins)
  plt.title("Vm histogram")
  plt.scatter(xaxis, VmHist,s=1)
  plt.plot(xaxis,VmHist)

  if (0):   # current ROI
    X = np.arange(0,cxs,1)
    Y = np.arange(0,cys,1)
    X, Y = np.meshgrid(X, Y)
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    ax.plot_surface(X, Y, roi[ipc], cmap='Purples')
    ax.set_xlim(0,cxs)
    ax.set_ylim(0,cys)
    ax.set_zlim(0,255)

  if (0):   # correlation
    px1 = int(cxs/2) - 10
    px2 = px1 + 20
    py1 = int(cys/2) - 10
    py2 = py1 + 20
    x = np.arange(px1,px2,1)
    y = np.arange(py1,py2,1)
    X, Y = np.meshgrid(x, y)
    fig, bx = plt.subplots(subplot_kw={"projection": "3d"})
    bx.plot_surface(X, Y, corrc[px1:px2,py1:py2], cmap='Purples')
    bx.set_xlim(px1,px2)
    bx.set_ylim(py1,py2)

  plt.show()
  #plt.savefig('speckle{0}.jpg'.format(n))
  #plt.clf()

#sys.exit()