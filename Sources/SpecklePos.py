#!/usr/bin/python3
# WatchObservatories - Andre Germain Dec 2024 - GPL licensing
# You must credit the above for conception of speckle position sensing for telescopes, and this software
#
# ROI: region of interest, or subset of the sensor for faster download and computation. ROI only needs
# to be so large to encompass sufficient speckle patterns.
#
# single frame ROI
# rpicam-still -t 20000 --shutter 500 --analoggain 2 --roi .25,.25,.1,.1
# video of ROI
# rpicam-vid -t 20000 --shutter 2000 --awbgains 1,0 False --denoise off --sat 0 --analoggain 2 --roi .5,.5,.0625,.0625 --o speckle.h264 --width 204 --height 154

import sys
import io
import time
import numpy as np
from PIL import Image as im
from picamera2 import Picamera2
from scipy import signal
from scipy import datasets
from scipy import ndimage
import matplotlib.pyplot as plt
from socket import socket, AF_INET, SOCK_DGRAM
import struct

debug=0         # 1 is only graphs, 2 is graphs and jpg

cx=int(3280/2)  # center of sensor IMX219 todo: get from camera modes
cy=int(2464/2)
cxs=128         # ROI pixel size
cys=cxs
exp=25*1000     # microseconds, todo: adjust exposure from initial image flux

count = 0
xi  = 0         # Integrated center shifts
yi  = 0         # Integrated center shifts
ipn = 4         # ROI pipeline depth
roi = [[[0]*cxs]*cys]*ipn  # ROI pipeline
x   = [0]*ipn   # ROI pipeline center shift in X
y   = [0]*ipn   # ROI pipeline center shift in Y
tm  = [0]*ipn   # ROI pipeline times
pathx = []      # path for plotting
pathy = []

# procedure to download ROI from sensor
def GetROI():   # id is for debug
  cam.start()
  img=cam.capture_array("main")
  roi = img[:,:,1]  # extract red (todo: figure index from camera properties)
  if (debug > 1):
    data = im.fromarray(roi)
    data.save('speckle{0}.jpg'.format(count))
  return roi

# procedure to detect correlation peak and its centroid
Rd=12    # centroid radius window
def Centroid(img, wx, wy):
  SumX = 0;
  SumY = 0;
  Sum  = 0;
  a, b = np.unravel_index(img.argmax(), img.shape)
  for bo in range(-Rd, Rd, 1):
    y = b + bo;
    for ao in range(-Rd, Rd, 1):
      x = a + ao
      if ((x>=0) and (x<wx) and (y>=0) and (y<wy)):
        Sum  += img[x,y]
        SumX += img[x,y] * x
        SumY += img[x,y] * y
  if (Sum):
    return(SumX/Sum, SumY/Sum)
  else:
    return(0, 0)

# START OF PROGRAM
# Initialize camera
cam = Picamera2()
modes = cam.sensor_modes
camconfig = cam.create_still_configuration(main={'size': (cxs,cys)})
cam.configure(camconfig)   # below, no automatic image adjustments allowed
cam.set_controls({"AwbEnable": False, "AeEnable": False, "ExposureTime": exp, "AnalogueGain": 1.0, "NoiseReductionMode": False, "ScalerCrop":(cx,cy,cxs,cys)})

#initialize socket
#CLIENT_IP = '192.168.2.206'
SERVER_IP = '192.168.2.154'
PORT_NUMBER = 50022
SrvSocketActive = 1
try:
  Socket = socket(AF_INET, SOCK_DGRAM)
  Socket.setblocking(False)  # prevent blocking
except:
  SrvSocketActive = 0

# Initialize ROIs & Correlator
xi = 0
yi = 0
x0 = 0
y0 = 0
dt = 0
dx = 0
dy = 0
dxl = 0
dyl = 0
dtl = 0
HB = 0
HBSrv = 0
ipp = 0              # previous image pipeline index
ipc = 1              # current  image pipeline index
ipl = 2              # oldest   image pipeline index

roi[0] = GetROI()    # quick start ROIs (will be the 'previous' at start of iterative code section

# adjust exposure before tracking
sum = 0
cnt = 0
for n in range (5,40):                                 # exposure range to test (in milliseconds)
  exp = n * 1000                                       # exposure in microseconds
  cam.set_controls({"ExposureTime": exp})              # set camera exposure
  roi[0] = GetROI()                                    # snap ROI at set exposure
  hist=ndimage.histogram(roi[0],min=0,max=255,bins=16) # histogram pixel brightness in 16 bins over 8 bit
  MxSignalIndex = np.max(np.nonzero(hist))             # locate highest non empty bin
  if (MxSignalIndex == 10):                            # add images with target highest bin
    sum = sum + exp                                    # sum suitable exposures
    cnt = cnt + 1                                      # track suitable exposure count
  time.sleep(.01)
  if (debug):
    print('auto exposure: time {:.2f} ms index {:2d}'.format(exp/1000, MxSignalIndex))

if (cnt):                  # if suitable exposures detected, average exposure sums, otherwise use default
  exp = int(sum/cnt)
else:
  exp = 25000

if (debug):
  print('final exposure setting: {:.2f}'.format(exp/1000))

# iterative code section
t0 = time.perf_counter()               # time zero

OriginFound = 0   # force origin detection
OriginAvg = 0     # origina averaging counter

cam.set_controls({"ExposureTime": exp})
roi[0] = GetROI()                      # start image pipeline

for n in range(0, 6*400):              # about 6 images per second - debugging/test phase of project
  exp = exp + 100 * 0  # debug for exposure sweeps
# cam.set_controls({"ExposureTime": exp})
  tm[ipc]  = time.perf_counter() - t0  # image time stamp
  roi[ipc] = GetROI()                  # snap a new ROI (as current index)

  # compute correlation of previous and current ROI
  corr=signal.correlate(np.array(roi[ipp]).astype(int), np.array(roi[ipc]).astype(int), mode='same',method='fft')

  # compute centroid of correlation result to determine shift from origin between current and previous image
  x[ipc], y[ipc] = Centroid(corr,cxs,cys)

  # sample correlation peak position when no motion for origin latching
  # note: in final design, averaging for origin needs to be done when there is no motion,
  # either detected by this code or told so by the server over socket
  if (OriginFound == 0):
    OriginAvg = OriginAvg + 1
    x0 = x0 + x[ipc]
    y0 = y0 + y[ipc]
    if (OriginAvg == 10):
      OriginFound = 1
      x0 = x0 / OriginAvg
      y0 = y0 / OriginAvg
      xi = 0
      yi = 0
      if (debug):
        print ('origin x: {:.2f} y: {:.2f}'.format(x0, y0))
  else:
    dx = x[ipc] - x0        # shift from origin in X axis between previous and current ROI
    dy = y[ipc] - y0        # shift from origin in Y axis between previous and current ROI
    xi = xi + dx            # integrated shifts in X axis (motion in X)
    yi = yi + dy            # integrated shifts in Y axis (motion in Y)
    dt = tm[ipc] - tm[ipp]  # elapsed time between previous and current ROI

    # compute correlation of oldest and current RIO
    corr=signal.correlate(np.array(roi[ipl]).astype(int), np.array(roi[ipc]).astype(int), mode='same',method='fft')

    # compute centroid of correlation result to determine shift from origin between current and oldest image
    x[ipl], y[ipl] = Centroid(corr,cxs,cys)

    dxl = (x[ipl] - x0) / (ipn-1)        # shift from origin in X axis between oldest and current ROI
    dyl = (y[ipl] - y0) / (ipn-1)        # shift from origin in X axis between oldest and current ROI
    dtl = (tm[ipc] - tm[ipl]) / (ipn-1)  # elapsed time between oldest and current ROI

    if (debug and (OriginFound != 0)):
      pathx.append(xi)
      pathy.append(yi)
      print ('{:6.3f} {:.2f} {:.2f} {:6.3f} {:6.3f} {:6.2f} {:6.2f}'.format(dt, x[ipc], y[ipc], dx, dy, xi, yi))
      if (debug > 1):
        data = im.fromarray((corr/np.max(corr)*255).astype(np.uint8))
        data.save('corr{0}.jpg'.format(count))
        count += 1
  # slow down process
  if (debug < 2):
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

# numpy debug graphs
if (debug):
  plt.subplots_adjust(hspace=.5)

  plt.subplot(3,2,1)
  plt.title("previous ROI")
  plt.imshow(roi[ipp], cmap='gray')

  plt.subplot(3,2,2)
  plt.title("current ROI")
  plt.imshow(roi[ipc], cmap='gray')

  plt.subplot(3,2,3)
  plt.title("correlation")
  plt.imshow(corr, cmap='gray')

  plt.subplot(3,2,4)
  plt.title("intensity histogram")
  plt.yscale('log')
  plt.plot(ndimage.histogram(roi[ipc],min=0,max=255,bins=256))

  plt.subplot(3,2,5)
  plt.title("correlation histogram")
  plt.yscale('log')
  maxc = np.amax(corr)
  hcorr = ndimage.histogram(corr,min=0,max=maxc,bins=100)
  plt.plot([i*maxc/100 for i in range(len(hcorr))], hcorr)

  plt.subplot(3,2,6)
  plt.title("speckle motion")
  plt.scatter(pathx,pathy,s=1)
  plt.axis((-400,400,-350,50))

  plt.show()

#sys.exit()