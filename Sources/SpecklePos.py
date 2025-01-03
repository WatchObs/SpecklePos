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
#  colour IMX219:
#   rpicam-vid -t 20000 --shutter 2000 --awbgains 1,0 False --denoise off --sat 0 --analoggain 2 --roi .5,.5,.0625,.0625 --o speckle.h264 --width 204 --height 154
#  mono OV9281: (no ROI available)
#   rpicam-vid -t 20000 --shutter 40 --denoise off --sat 0 --analoggain 1 speckle.h264 --width 1280 --height 800

import time
import numpy as np
import matplotlib.pyplot as plt
import struct
from PIL import Image as im
from picamera2 import Picamera2
from socket import socket, AF_INET, SOCK_DGRAM
from scipy import signal
from scipy import ndimage

debug = 1         # 1 is only graphs, 2 is graphs and jpg

cx = int(1280/2) # center of sensor OV9281 TODO: get from camera modes
cy = int(800/2)  # "
cxs = 128        # ROI pixel size
cys = cxs        # "
exp0 = 7         # default exposure in microseconds
exp = exp0       # exposure in microseconds

count = 0
xi = 0          # Integrated center shifts
yi = 0          # Integrated center shifts
ipn = 5         # ROI pipeline depth
roi = [[[0]*cxs]*cys]*ipn  # ROI pipeline
DarkF = 0       # 'dark frame' as it were, to increase CM estimate quality
x = [0]*ipn     # ROI pipeline center shift in X
y = [0]*ipn     # ROI pipeline center shift in Y
tm = [0]*ipn    # ROI pipeline times
pathx = []      # path for plotting
pathy = []

# precision delay (microseconds)
def Delay(uS):
  t0 = time.perf_counter() + uS / 1000000
  while (time.perf_counter() < t0):
    pass

# procedure to download ROI from sensor
def GetROI(Shift2Zero):   # id is for debug
  global exp
  global DarkF
  cam.start()
  Delay(exp)
  img=cam.capture_array("main")
  cam.stop()
  roi = img[:,:,1][cx:cx+cxs,cy:cy+cys]
  if (debug > 1):
    roidf = np.clip((roi.astype(np.int16) - DarkF),a_min=0, a_max=255)
    data = im.fromarray(roidf.astype(np.uint8))
    data.save('speckle{0}.jpg'.format(count))

  if (Shift2Zero):
    return np.clip((roi.astype(np.int16) - DarkF),a_min=0, a_max=255)
  else:
    return roi

# procedure to detect correlation peak and its centroid
Rd=12    # centroid radius window
def Centroid(img, wx, wy):
  SumX = 0   # sum of X 'moments'
  SumY = 0   # sum of Y 'moments'
  SumM = 0   # sum of 'masses'
  a, b = np.unravel_index(img.argmax(), img.shape)
  for bo in range(-Rd, Rd, 1):
    y = b + bo;
    for ao in range(-Rd, Rd, 1):
      x = a + ao
      if ((x >= 0) and (x < wx) and (y >= 0) and (y < wy)):
        SumM += img[x,y]
        SumX += img[x,y] * x
        SumY += img[x,y] * y
  if (SumM):
    return(SumX/SumM, SumY/SumM)
  else:
    return(0, 0)

# adjust exposure before tracking
def SetExposure():
  global exp
  global DarkF
  sum = 0                                                # sum of exposures that are within targeted flux
  cnt = 0                                                # count of exposures that are within targeted flux
  for n in range (7,45,3):                               # exposure range to test (in microseconds)
    exp = n                                              # exposure in microseconds
    cam.set_controls({"ExposureTime": exp})              # set camera exposure
    roi = GetROI(0)                                      # snap ROI at set exposure
    hist=ndimage.histogram(roi, min = 0, max = 255, bins = 256) # histogram pixel brightness in 16 bins over 8 bit
    MxSignalIndex = np.max(np.nonzero(hist))             # locate highest non empty bin (threshold to avoid hot pixels)
    MnSignalIndex = np.min(np.nonzero(hist))             # locate lowest  non empty bin (threshold to avoid hot pixels)
    if ((MxSignalIndex > 160) and (MxSignalIndex < 240)):# add images with target highest bin
      sum = sum + exp                                    # sum suitable exposures
      cnt = cnt + 1                                      # track suitable exposure count
    if (debug):
      print('auto exposure: time {:.2f} uS index low/high {:2d}/{:2d}'.format(exp, MnSignalIndex, MxSignalIndex))

  if (cnt):                  # if suitable exposures detected, average exposure sums, otherwise use default
    exp = int(sum/cnt)
  else:
    exp = exp0

  #snap final image at target exposure and extract histogram extent
  cam.set_controls({"ExposureTime": exp})              # set camera exposure
  roi = GetROI(0)                                      # snap ROI at set exposure
  hist=ndimage.histogram(roi, min = 0, max = 255, bins = 256) # histogram pixel brightness in 16 bins over 8 bit
# print(hist)
  MxSignalIndex = np.max(np.nonzero(hist))             # locate highest non empty bin (threshold to avoid hot pixels)
  MnSignalIndex = np.min(np.nonzero(hist))             # locate lowest  non empty bin (threshold to avoid hot pixels)
# DarkF = int((MxSignalIndex + MnSignalIndex)/2)       # lower cutoff (dark frame) for further imaging - middle of family
  DarkF = np.array(hist).argmax()                      # lower cutoff (dark frame) for further imaging - maximum value index

  if (debug):
    print('final exposure setting: {:.2f} uS index low/high {:3d}/{:3d}'.format(exp, MnSignalIndex, MxSignalIndex))
    print('dark frame threshold: {:3d}'.format(DarkF))

  return int(exp)

# START OF PROGRAM
# Initialize camera
cam = Picamera2()
modes = cam.sensor_modes
#camconfig = cam.create_still_configuration(main={'size': (cxs,cys)})   # ROI (no effect on OV9281, ok on IMX219)
camconfig = cam.create_still_configuration(main={'size': (1280,800)})
cam.configure(camconfig)
#cam.camera_configuration()['raw']
# colour camera
#cam.set_controls({"AwbEnable": False, "AeEnable": False, "ExposureTime": exp0, "AnalogueGain": 1.0, "NoiseReductionMode": False, "ScalerCrop":(cx,cy,cxs,cys)})
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

exp = SetExposure()  # adjust exposure
cam.set_controls({"ExposureTime": exp})

# iterative code section
t0 = time.perf_counter() # time zero
OriginFound = 0          # force origin detection
OriginAvg = 0            # origin averaging counter

roi[ipp] = GetROI(1)     # start image pipeline (with 'dark frame' substraction)

for n in range(0, 5*100): # about 5 images per second - debugging/test phase of project
  tm[ipc]  = time.perf_counter() - t0  # image time stamp
  roi[ipc] = GetROI(1)                 # snap a new ROI (as current index)

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
    dyl = (y[ipl] - y0) / (ipn-1)        # shift from origin in Y axis between oldest and current ROI
    dtl = (tm[ipc] - tm[ipl]) / (ipn-1)  # elapsed time between oldest and current ROI

    if (debug and (OriginFound != 0)):
      pathx.append(xi)
      pathy.append(yi)
      print ('dt:{:6.3f} dx:{:6.3f} dy:{:6.3f} xi:{:6.2f} yi:{:6.2f}'.format(dt, dx, dy, xi, yi))
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
  plt.axis((-200,200,-200,200))

  plt.show()

#sys.exit()