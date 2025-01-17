#!/usr/bin/python3
#
# WatchObservatory.com - Andre Germain Jan 2025 - GPL licensing
# You must credit the above for conception of speckle position sensing for telescopes, and this software
#
# ROI: region of interest, or subset of the sensor for faster download and computation.
# ROI only needs to be so large to encompass sufficient speckle patterns.
#
# With OV9281 3rd party cameras, timeout may occur. Add the following line in rpi_apps.yaml using the command below it:
# "camera_timeout_value_ms":    5000000,
# sudo nano /usr/share/libcamera/pipeline/rpi/vc4/rpi_apps.yaml
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
import numpy as np
import matplotlib.pyplot as plt
import struct
from PIL import Image as im
from picamera2 import Picamera2
from socket import socket, AF_INET, SOCK_DGRAM
from scipy import signal
from scipy import ndimage
import os
os.environ['LIBCAMERA_LOG_LEVELS'] = '4'   # silence libcamera2

debug = 1        # 1 is only graphs, 2 is graphs and jpg

SpecklePosFlatFrame = 'SpecklePosFlatFrame.npy'  # flat fiel file name

# OV9281 camera
cx  = 1280       # X pixel count TODO: get from camera
cy  = 800        # Y pixle count
cxo = int(cx/2)  # center of sensor
cyo = int(cy/2)  # "
cxs = 128        # ROI pixel size (NOTE: use power of 2 for FFT correlator)
cys = 128        # "

count = 0
ipn = 5                                           # ROI pipeline depth
stn = 5                                           # stacking depth
R     = np.zeros((cxs,cys), dtype=float)          # FFT cross product freq domain
r     = np.zeros((cxs,cys), dtype=float)          # FFT inverse cross prouct spatial domain
roi   = [np.zeros((cxs,cys), dtype=float)] * ipn  # ROI pipeline
rois  = [np.zeros((cxs,cys), dtype=float)] * stn  # ROI stack
roib  = np.zeros((cxs,cys), dtype=float)          # ROI background
roiff = np.ones((cxs,cys), dtype=float)           # ROI flat field
roiff_= np.ones((cxs,cys), dtype=float)           # ROI flat field accumulator
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
  global rois
  global stn
  global count

# t0 = time.perf_counter_ns()
#  img = cam.capture_array("main") #.astype(float)
# Delay(exp)   # NO NEED since capture_array is blocking by default
# t1 = time.perf_counter_ns()
# print((t1-t0)/1000000)
# img10b = (img[:, 0::5] << 2) | (img[:, 1::5] >> 6)
# print(img.shape)
# print(img10b.shape)
# roi = img[:,:,1].astype(float) - roib  # main
# roi = img[:,:,1][cxo:cxo+cxs,cyo:cyo+cys].astype(float) - roib  # main
# roi = img10b[cxo:cxo+cxs,cyo:cyo+cys].astype(float) - roib  #raw
# roi = img[cxo:cxo+cxs,cyo:cyo+cys].astype(float) - roib  #raw

  # average ROIs for SNR
  ts0 = time.perf_counter_ns()
  for m in range(0, stn):  # burst capture images for SNR
    img = cam.capture_array("main")
    rois[m] = img[:,:,1].astype(float) - roib  # main
  ts1 = time.perf_counter_ns()
  print("stacking time (ms): ",int((ts1-ts0)/1000000))

  # stack images and average
  roi = rois[0]
  for m in range(1, stn):
    roi = np.add(roi, rois[m])
  roi /= stn

  if (Shift2Zero):
    roi = np.clip((roi - DarkF), a_min=0, a_max=255)
    if (debug > 1):
      data = im.fromarray(roi.astype(np.uint8))
      data.save('speckle{0}.jpg'.format(count))

  return roi

# procedure to detect correlation peak and its centroid
def Centroid(roi, sel):
  Rd = 2     # centroid radius window
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

  for exp in range (7,45,2):                    # exposure range to test (in microseconds)
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
if (0):
  for n in range (1,6): print(modes[n])
camconfig = cam.create_still_configuration(main={'size': (cxs,cys)}, queue=False)   # size same as ROI
#camconfig = cam.create_still_configuration(main={'size': (1280,800)}, queue=False)
#camconfig = cam.create_still_configuration(raw={'size': (1280,800), 'format' : 'R10_CSI2P'}, queue=False)
#camconfig = cam.create_still_configuration(raw={'size': (cxs,cys), 'format' : 'R8'}, queue=False)
cam.configure(camconfig)
crop = (cxo, cyo, cxs, cys)
cam.set_controls({"AeEnable": False, "ExposureTime": exp0, "AnalogueGain": 1.0, "NoiseReductionMode": False, "ScalerCrop": crop})
#print(cam.camera_controls['ScalerCrop'])
#print("ROI: ",cxs,"/",cys)
#print(cam.camera_configuration())
#stride = camconfig['raw']['stride']
#print("raw pixel byte width ", stride)
cam.start()

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

# ROI windows for fft
window1d = signal.windows.tukey(cxs, alpha=.2, sym=True)  # TODO, should make with cxs and cys as they may not be the same
window2d = np.sqrt(np.outer(window1d, window1d))

# set light source control
LightPin=12
GPIO.setmode(GPIO.BCM)
GPIO.setup(LightPin, GPIO.OUT)

# switch on light source for auto exposure (roib must be zero for this as roib depends on exposure)
GPIO.output(LightPin, GPIO.HIGH)   # light source on
time.sleep(.25)              # wait for light source to stabilize
exp = SetExposure()          # adjust exposure
#exp = 500000 # test
cam.set_controls({"ExposureTime": exp})

# switch off light source to sample background frame (roib)
GPIO.output(LightPin, GPIO.LOW)   # light source off
time.sleep(.25)             # wait for light source to stabilize
bn = 25
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
for n in range(0, 4*5): # 4 images per second - debugging/test phase of project
  tm[ipc]  = time.perf_counter() - t0  # image time stamp

  roi[ipc] = GetROI(1)

  # flat frame - must do this on ROI before these are flat framed!
  if (RateSrv != 0):   # TODO, logic on HBSrv
    if (ROIFFCnt < 1000):
      ROIFFCnt = ROIFFCnt + 1
      roiff_ = roiff_ + roi[ipc]

  roi[ipc] = np.multiply(roi[ipc], roiff)  # apply flat field - DO NOT MOVE above flat frame accumulator!

  # compute correlation (previous, current), (oldest, current) ROI
  corrc = np.subtract(signal.correlate(roi[ipp], roi[ipc], mode='same', method='fft'), 0)
  corrl = np.subtract(signal.correlate(roi[ipl], roi[ipc], mode='same', method='fft'), 0)
  # compute phase shift in frequency domain and return to spatial domain for linear shift, both axes
  if (0):  # fft method
#   fftp = np.fft.rfft2(np.multiply(roi[ipp],window2d))  # fft of previous ROI
#   fftc = np.fft.rfft2(np.multiply(roi[ipc],window2d))  # fft of current  ROI
    fftp = np.fft.rfft2(roi[ipl])           # fft of previous ROI
    fftc = np.fft.rfft2(roi[ipc])           # fft of current  ROI
    R = np.multiply(fftp, np.conj(fftc))    # complex cross product of ffts (conjugate on second)
    r = np.fft.fftshift(np.fft.irfft2(R))   # shift zero freq to center
    dxf, dyf = Centroid(np.abs(r),0)        # centroid shifted peak
    dxf -= int(cxs/2)                       # shift from zero
    dyf -= int(cys/2)
    dxf /= (ipn-1)
    dyf /= (ipn-1)

  # compute centroid of correlations result to determine shift from origin between current and previous image
  x[ipc], y[ipc] = Centroid(corrc,0)
  x[ipl], y[ipl] = Centroid(corrl,1)

  # se t shift, integrated positions and time deltas
  dx = x[ipc] - x0        # shift from origin in X axis between previous and current ROI
  dy = y[ipc] - y0        # shift from origin in Y axis between previous and current ROI
  xi = xi + dx            # integrated shifts in X axis (motion in X)
  yi = yi + dy            # integrated shifts in Y axis (motion in Y)
  dt = tm[ipc] - tm[ipp]  # elapsed time between previous and current ROI
  dxl = (x[ipl] - x0) / (ipn-1)        # shift from origin in X axis between oldest and current ROI
  dyl = (y[ipl] - y0) / (ipn-1)        # shift from origin in Y axis between oldest and current ROI
  dtl = (tm[ipc] - tm[ipl]) / (ipn-1)  # elapsed time between oldest and current ROI

  if (0):   # 3d surface plot to jpeg
    X = np.arange(50,78,1)
    Y = np.arange(50,78,1)
    X, Y = np.meshgrid(X, Y)
    fig, ex = plt.subplots(subplot_kw={"projection": "3d"})
    arr = corrl[50:78,50:78]  # r[50:78,50:78]
    ex.plot_surface(X, Y, arr, cmap='Purples')
#   ex.set_zlim(15000000,25000000)
    plt.savefig('peak{0}.jpg'.format(count))
    plt.close(fig)

  if (debug):
    pathx.append(xi)
    pathy.append(yi)
    print ('dt:{:6.3f} dx:{:6.3f} dy:{:6.3f} xi:{:6.2f} yi:{:6.2f}'.format(dt, dx, dy, xi, yi))
    if (debug > 1):
      data = im.fromarray((corrl/np.max(corrl)*255).astype(np.uint8))
      data.save('corr{0}.jpg'.format(count))

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
#   buf = struct.pack('<IIIfffffffff',HB,HBSrv,Status,tm[ipc],dt,dtl,dxf,dxl,dyf,dyl,xi,yi)
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

  if (debug):
    count += 1

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
cam.stop()

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

    if (0):   # current ROI
      X = np.arange(0,cxs,1)
      Y = np.arange(0,cys,1)
      X, Y = np.meshgrid(X, Y)
      fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
      ax.plot_surface(X, Y, roi[ipc], cmap='Purples')
      ax.set_xlim(0,cxs)
      ax.set_ylim(0,cys)
      ax.set_zlim(0,255)

    x = np.arange(50,78,1)
    y = np.arange(50,78,1)
    X, Y = np.meshgrid(x, y)
    fig, bx = plt.subplots(subplot_kw={"projection": "3d"})
    bx.plot_surface(X, Y, corrl[50:78,50:78], cmap='Purples')
    bx.set_xlim(50,78)
    bx.set_ylim(50,78)

    if (0): # fft peak 3d surface plot
      x = np.arange(50,78,1) #len(r),1)
      y = np.arange(50,78,1) #len(r),1)
      X, Y = np.meshgrid(x, y)
      fig = plt.figure()
      bx = fig.add_subplot(111, projection='3d')
      bx.plot_surface(X, Y, np.abs(r[50:78,50:78]), cmap='Purples')
      bx.set_xlim(50,78)
      bx.set_ylim(50,78)

    plt.show()

#sys.exit()