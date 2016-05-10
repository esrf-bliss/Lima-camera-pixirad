# -*- coding: utf-8 -*-
"""
Éditeur de Spyder

Ceci est un script temporaire.
"""



import sys
import time
import socket # For hostaname


if (socket.gethostname() == "linfalcon"):
    sys.path.append('/home/watier/LimaGitLab/Lima_test/install')
if (socket.gethostname() == "lid00limabois"):
    sys.path.append('/users/watier/lima/install')
if (socket.gethostname() == "lbm05pixirad"):
    sys.path.append('/users/watier/Git/lima/install')


from Lima import Core
from Lima import Pixirad as PixiradAcq

import matplotlib.pyplot as plt
import numpy as np
#
#

#Be wild:
#Core.Processlib.PoolThreadMgr.get().setNumberOfThread()
Core.Processlib.PoolThreadMgr.get().setNumberOfThread(20)


def debugtrace():
  db = Core.DebParams()
  db.setModuleFlagsNameList(["Camera"])
  db.setTypeFlagsNameList(['Trace'])
  db.setTypeFlagsNameList(['Trace','Funct'])

def debugchucknorris():
  db = Core.DebParams()
  db.setModuleFlagsNameList(["Camera", "Control"]) 
  db.setTypeFlagsNameList(['Trace'])
  db.setTypeFlagsNameList(['Trace','Funct'])


def debugwarning():
  db = Core.DebParams()
  db.setModuleFlagsNameList(["Camera"])
  db.setTypeFlagsNameList(['Warning'])
  db.setTypeFlagsNameList(['Warning','Funct'])


def debugoff():
  db = Core.DebParams()
  db.setModuleFlagsNameList([])
  db.setTypeFlagsNameList([])
  
debugtrace()

print "\n\n\n\n ======= INIT ======== \n"

#debugchucknorris()


#camera = PixiradAcq.Camera("172.24.8.135", 6666)
camera = PixiradAcq.Camera("192.168.0.1", 2222)

camera.init()  ### Verifier INIT TODO

#time.sleep(3)

#print "\n\n\n\n ======= RESET ======== \n"

#camera.reset()

#print "\n\n\n\n ======= ATTENTE REBOOT DETECTEUR ======== \n"
#time.sleep(30)

print "\n\n\n\n ======= INTERFACE ======== \n"

camera_interface = PixiradAcq.Interface(camera)

#camera_interface.setNbiMode(camera.SensorConfigNBI_OFF)
camera_interface.setNbiMode(camera.SensorConfigNBI_ON)
#camera_interface.setDeadTimeFreeMode(camera.DeadTimeFreeModeON)
camera_interface.setDeadTimeFreeMode(camera.DeadTimeFreeModeOFF)
camera_interface.setRunConfigMode(camera.DATA)


camera_interface.setHybridMode(camera.CDTE)

#camera_interface.setSensorConfigBuild(camera.PX1)
camera_interface.setSensorConfigBuild(camera.PX8)

camera_interface.setColorMode(camera.COLMODE_1COL0)

camera.setCoolingTemperatureSetpoint(-50)



camera.setLowThreshold0(10)
camera.setHighThreshold0(60)
camera.setLowThreshold1(10)
camera.setHighThreshold1(60)

camera.setHighVoltageBiais(2100)
camera.setHVBiasModePower(1)


camera.setHighVoltageDelayBeforeOn(3)

camera.setHVRefreshPeriod(1000);
#camera.setHVBiasModePower(camera.STDHV);


camera_interface.setTrsfMode(camera.UNMOD)
#camera.setTrsfMode(camera.UNMOD)

#beforePrepare = time.time()


#print "\n\n\n\n ======= PREPARE UDP Tuning  ======== \n"
#camera_interface.setNCyclesUdpDelay(0)
#camera_interface.setNCyclesUdpDelay(10000) # 
#camera_interface.setNCyclesUdpDelay(100)
#camera_interface.setNCyclesUdpDelay(500)


#time.sleep(1)



# FOrce image generation on the PX8

#camera.setSeedModeForDebugOnlyInOneChipWithPX8(True)
#camera.setWhichModuleOutOfEightOnPX8(0)

#camera.setWhichModuleOutOfEightOnPX8(5)



print "\n\n\n\n ======= CONTROL ======== \n"
control = Core.CtControl(camera_interface)

print control.Status()

control.buffer().setMaxMemory(70)

print "\n\n\n\n ======= ACQUISITION OBJECT ======== \n"

acq = control.acquisition()

saving = control.saving()

def newsave(newsaving, basename):
  pars=newsaving.getParameters()
  #pars.directory='/users/watier/test_du_stagiaire'
  #pars.directory='/data/visitor/md829/id17/Christopher_head_1'
  pars.directory='/tmp'
  pars.prefix=basename
  pars.suffix='.edf'
  #pars.suffix='.raw'
  pars.fileFormat=Core.CtSaving.EDF
  #pars.fileFormat=Core.CtSaving.RAW
  pars.savingMode=Core.CtSaving.AutoFrame
  newsaving.setParameters(pars)
  
  

  
#newsave(saving , time.strftime('TestPX8_on_ID17_firsttest_%Y_%m_%d-%H_%M_%S'))

#newsave(saving , time.strftime('test_stagiaire_%Y_%m_%d-%H_%M_%S'))


def externalTrigger():
  acq.setTriggerMode(Core.ExtTrigMult)

def internalTrigger():
  acq.setTriggerMode(Core.IntTrig)



#print "\n\n\n\n ========== ROI ==========\n "
##
##image = control.image()
##maroi = Core.Roi(0,0,100,100)
##image.setRoi(maroi)


print control.Status()

newsave(saving , time.strftime('TestPX8_change_filename_with_newsave_%Y_%m_%d-%H_%M_%S'))


acq.setAcqExpoTime(1)
acq.setAcqNbFrames(1)


#afterfirstimage = time.time()+1 # Update when acq really start on the detector level.



def id_17_acq():
  
  debugoff();
  
  acq.setLatencyTime(0); 
  camera.setLowThreshold0(0); 
  camera.setLowThreshold1(0); 
  camera.setHighThreshold0(99);  
  camera.setHighThreshold1(99); 
  acq.setAcqNbFrames(1); 
  acq.setAcqExpoTime(0.001); 
  control.prepareAcq(); 
  control.startAcq()






def defineMaxFramerateForVideo(nbimages, duree, delaiBetweenFrames):
  
    print "\n\n\n\n ======= PREPARE  ======== \n"
    acq.setAcqExpoTime(duree)
    acq.setAcqNbFrames(nbimages)
    acq.setLatencyTime(delaiBetweenFrames)  
    beforePrepare = time.time()
    
    
    control.prepareAcq()
    
    print "\n\n\n\n ======= START  ======== \n"
    beforeStart = time.time()
    control.startAcq()
    
    
    while not (control.getStatus().AcquisitionStatus == Core.AcqReady) :
      #print "\n\n\n\n ======= WAIT  ======== \n\n\n\n"
      time.sleep(0.1)

    allImagesReceived = time.time()
    print "PrepareTime = "+str(beforeStart - beforePrepare)     
    print "FirstImageLagTime = "+str(firstImageReceived - beforePrepare)     
    print "Expo time estimated : "+str(duree*nbimages+nbimages*delaiBetweenFrames)
    print "TotalTime : "+str(allImagesReceived-beforeStart)
    print "TotalTime reception : " +str(allImagesReceived-firstImageReceived)
    print "fps:"+str(nbimages/(allImagesReceived-beforeStart))
    print "fps reception:"+str(nbimages/(allImagesReceived-firstImageReceived))
    
    return(nbimages/(allImagesReceived-firstImageReceived))


def frange(x, y, jump):
  while x < y:
    yield x
    x += jump
    








    
framerates_delai = []
framerates_Coldtf_DtfON = []
fpsvoulu = []

def defineMaxFramerateForVideoAuto(minfps, maxfps, incrementfps, nbframes ):
  global framerates, fpsvoulu
  
  
      
  print "fps for COLMODE_DTF DTFON" 
  
  camera_interface.setColorMode(camera.COLMODE_DTF)  
  camera_interface.setDeadTimeFreeMode(camera.DeadTimeFreeModeON)  
  
  for i in frange(minfps,maxfps, incrementfps):
      fpsvoulu.append(i)
      framerates_Coldtf_DtfON.append(defineMaxFramerateForVideo(nbframes, 1.0/i, 0))
    
  for i in frange(minfps,maxfps, incrementfps):
    framerates_delai.append(defineMaxFramerateForVideo(nbframes, 0.001, 1.0/i))
    
    
  plt.plot(fpsvoulu, fpsvoulu)


  plt.scatter(fpsvoulu, framerates_delai)
  plt.plot(fpsvoulu, framerates_delai)

  plt.scatter(fpsvoulu, framerates_Coldtf_DtfON)
  plt.plot(fpsvoulu, framerates_Coldtf_DtfON)

  plt.show()




fpsobs = []
fpsvoulu = []
cycles = []

def stability(fps, nbexpo, nbcycles):
  global fpsobs, fpsvoulu
  
  
      
  print "fps stability for COLMODE_DTF DTFON" 
  
  camera_interface.setColorMode(camera.COLMODE_DTF)  
  camera_interface.setDeadTimeFreeMode(camera.DeadTimeFreeModeON)  
  
  for i in frange(1,nbcycles, 1):
      cycles.append(i)
      fpsvoulu.append(fps)
      fpsobs.append(defineMaxFramerateForVideo(nbexpo, 1.0/fps, 0))
        
    
  plt.plot(cycles, fpsvoulu)


  plt.scatter(cycles, fpsobs)
  plt.plot(cycles, fpsobs)


  plt.show()

#externalTrigger()
internalTrigger()

camera.setLowThreshold0(1); camera.setLowThreshold1(1); camera.setHighThreshold0(96);  camera.setHighThreshold1(96);

def noAccumulation():
  acq.setAcqMode(Core.Single)

def accumulation(tempsExpoMax, tempsTotal, nbframes):
  
  acq.setAcqMode(Core.Accumulation)
  acq.setAccMaxExpoTime(tempsExpoMax)
  acq.setAcqExpoTime(tempsTotal); 
  acq.setAcqNbFrames(nbframes)
  control.prepareAcq(); 
  control.startAcq()


#def hvscan(duree):
  
  
  #camera.setLowThreshold0(0)
  #camera.setHighThreshold0(60)
  
  #for i in range(0,400,10):
    ##time.sleep(10)

    #print "\n\n\n\n ======= SETTERS  ======== \n"
    #camera.setHighVoltageBiais(i)
    

    #print "\n\n\n\n ======= PREPARE  ======== \n"
    #control.prepareAcq()
    #print "\n\n\n\n ======= START  ======== \n"
    #control.startAcq()
    
    #while not (control.getStatus().AcquisitionStatus == Core.AcqReady) :
      #print "\n\n\n\n ======= WAIT  ======== \n\n\n\n"
      #time.sleep(0.2)
    
  
scannumber = 0
  
def energyscan():
  global scannumber
  scannumber =scannumber+1
  #accumulation(0.1,5,1)
  
  acq.setAcqExpoTime(3); 
  acq.setAcqNbFrames(1)

  for i in range(0,60):
    #time.sleep(10)

    print "\n\n\n\n ======= SETTERS  ======== \n"

    camera.setLowThreshold0(i)
    #camera.setHighThreshold0(i+1)
    camera.setHighThreshold0(60)

    camera.setLowThreshold1(i)
    #camera.setHighThreshold0(i+1)
    camera.setHighThreshold1(60)

    #newsave(saving , str(scannumber)+"_rat_skull_energyScan_"+str(i)+"-"+str(i+1) )
    newsave(saving ,  time.strftime('energy_scan_source_cedric_%Y_%m_%d-%H_%M_%S__low_')+str(i)+"_high_"+str(60) )
    
    print "\n\n\n\n ======= PREPARE  ======== \n"
    control.prepareAcq()
    print "\n\n\n\n ======= START  ======== \n"
    control.startAcq()
    
    while not (control.getStatus().AcquisitionStatus == Core.AcqReady) :
      print "\n\n\n\n ======= WAIT  ======== \n\n\n\n"
      time.sleep(0.2)
      
      
      
      
      
      
def scanModules():
  
  
  acq.setAcqExpoTime(3); 
  acq.setAcqNbFrames(1)
  
  for i in range(0,7):
    camera.setWhichModuleOutOfEightOnPX8(i)
    newsave(saving , time.strftime('module_scan_source_cedric_%Y_%m_%d-%H_%M_%S__module_')+str(i))
    
    print "\n\n\n\n ======= PREPARE  ======== \n"
    control.prepareAcq()
    print "\n\n\n\n ======= START  ======== \n"
    control.startAcq()
    
    while not (control.getStatus().AcquisitionStatus == Core.AcqReady) :
      print "\n\n\n\n ======= WAIT  ======== \n\n\n\n"
      time.sleep(0.2)
      
  
    
def exposcan(min, max, increment):  
  
  camera_interface.setColorMode(camera.COLMODE_DTF)  
  camera_interface.setDeadTimeFreeMode(camera.DeadTimeFreeModeON)  
  
  
  for i in frange(min, max, increment):
    #time.sleep(10)

    print "\n\n\n\n ======= SETTERS  ======== \n"

    acq.setAcqExpoTime(i)

    print "\n\n\n\n ======= PREPARE  ======== \n"
    control.prepareAcq()
    print "\n\n\n\n ======= START  ======== \n"
    control.startAcq()
    
    while not (control.getStatus().AcquisitionStatus == Core.AcqReady) :
      #print "\n\n\n\n ======= WAIT  ======== \n\n\n\n"
      time.sleep(0.2)
    
def roi():
  
  print "\n\n\n\n ========== ROI ==========\n "

  image = control.image()
  maroi = Core.Roi(0,0,100,100)
  image.setRoi(maroi)



def resetLatency():
  global latency
  latency = np.array(0)
  latency = np.delete(latency,0)

resetLatency()

print "\n\n\n\n  ======= CALLBACK ======= \n "
class MyCbk(Core.CtControl.ImageStatusCallback) :
	def imageStatusChanged(self,img_status) :

	    global latency
	    global firstImageReceived
	    #global ic
	    if(img_status.LastImageAcquired==0):
	      firstImageReceived = time.time()
	    
	    latency = np.append(latency, img_status)
	    try:
	      im = control.ReadImage()
	      print str(time.time()),' Reconstruction latency: ', img_status.LastImageAcquired - img_status.LastImageSaved, " img timestamp: ", im.timestamp, " frame: ",im.frameNumber
	    except Exception, e:
	      pass
	    
	    
	    #latency = np.append(latency,im.timestamp)
	    # latency = np.append(latency,[[time.time(), img_status.LastImageAcquired, img_status.LastImageSaved]])
	    #print 'ic', type(ic), ic.shape


#Cbk = MyCbk()
#control.registerImageStatusCallback(Cbk)


def plot():
  latency.shape = (latency.shape[0]/3, 3)



    
  
  #print "\n\n\n\n ======= TEST  ======== \n"
  #print (control.Status().AcquisitionStatus  != Core.AcqReady)
  #print control.getStatus()
  #print (control.Status().AcquisitionStatus  != Core.AcqReady)
  
  #while (not (control.Status().AcquisitionStatus  == Core.AcqReady)):
    #print "\n\n\n\n ======= WAIT  ======== \n\n\n"      
    #print control.getStatus()
    #print control.Status().AcquisitionStatus
    #time.sleep(0.1)



#camera_interface.setHighVoltageBiais(400)

#
#print "\n\n\n\n  ======= Test camera set/get ======= \n "
#
#camera.getRunConfigMode()
#camera.setRunConfigMode(camera.SUMMED_DATA)
#camera.getRunConfigMode()
#
#print "\n\n\n\n  ======= Test interface set/get ======= \n "
#
#camera_interface.getRunConfigMode()
#camera_interface.setRunConfigMode(camera.DATA)
#camera_interface.getRunConfigMode()



#camera.AUTOHV                         camera.HVMode                         camera.getCoolingTemperatureSetpoint  camera.setCoolingMode
#camera.AUTOHV_LC                      camera.HV_OFF                         camera.getDeadTimeFreeMode            camera.setCoolingTemperatureSetpoint
#camera.CDTE                           camera.HV_ON                          camera.getDelayBetweenFrames          camera.setDeadTimeFreeMode
#camera.COLLECT_FLAT                   camera.PII                            camera.getFakeMode                    camera.setDelayBetweenFrames
#camera.COLMODE_1COL0                  camera.PIII                           camera.getHVBiasMode                  camera.setFakeMode
#camera.COLMODE_1COL1                  camera.PX1                            camera.getHVBiasModePower             camera.setHVBiasMode
#camera.COLMODE_2COL                   camera.PX2                            camera.getHVRefreshPeriod             camera.setHVBiasModePower
#camera.COLMODE_4COL                   camera.PX8                            camera.getHighThreshold0              camera.setHVRefreshPeriod
#camera.COLMODE_DTF                    camera.RunConfigMode                  camera.getHighThreshold1              camera.setHighThreshold0
#camera.COOLING_OFF                    camera.STDHV                          camera.getHighVoltageBiais            camera.setHighThreshold1
#camera.COOLING_ON                     camera.SUMMED_COLLECT_FLAT            camera.getHighVoltageDelayBeforeOn    camera.setHighVoltageBiais
#camera.ColorMode                      camera.SUMMED_DATA                    camera.getHybridMode                  camera.setHighVoltageDelayBeforeOn
#camera.CoolingMode                    camera.SensorConfigASIC               camera.getLowThreshold0               camera.setHybridMode
#camera.DATA                           camera.SensorConfigBuild              camera.getLowThreshold1               camera.setLowThreshold0
#camera.DeadTimeFreeMode               camera.SensorConfigHybrid             camera.getNbiMode                     camera.setLowThreshold1
#camera.DeadTimeFreeModeOFF            camera.SensorConfigNBI                camera.getRunConfigMode               camera.setNbiMode
#camera.DeadTimeFreeModeON             camera.SensorConfigNBI_OFF            camera.init                           camera.setRunConfigMode
#camera.FakeMode                       camera.SensorConfigNBI_ON             camera.prepareAcq                     camera.startAcq
#camera.FakeOFF                        camera.getAsicMode                    camera.reset                          camera.stopAcq
#camera.FakeON                         camera.getColorMode                   camera.setAsicMode                    
#camera.GAAS                           camera.getConfigBuild                 camera.setColorMode                   
#camera.HVBiaisPower                   camera.getCoolingMode                 camera.setConfigBuild      


#camera_interface.setAsicMode                    camera_interface.setFakeMode                    camera_interface.setHighVoltageDelayBeforeOn
#camera_interface.setColorMode                   camera_interface.setHVBiasMode                  camera_interface.setHybridMode
#camera_interface.setConfigBuild                 camera_interface.setHVBiasModePower             camera_interface.setLowThreshold0
#camera_interface.setCoolingMode                 camera_interface.setHVRefreshPeriod             camera_interface.setLowThreshold1
#camera_interface.setCoolingTemperatureSetpoint  camera_interface.setHighThreshold0              camera_interface.setNbiMode
#camera_interface.setDeadTimeFreeMode            camera_interface.setHighThreshold1              camera_interface.setRunConfigMode
#camera_interface.setDelayBetweenFrames          camera_interface.setHighVoltageBiais            


#camera_interface.setAsicMode(camera.PII)              
#camera_interface.setFakeMode(camera.FakeOFF)
#camera_interface.setHighVoltageDelayBeforeOn(0.001) # 1 sec
#camera_interface.setColorMode(camera.COLMODE_1COL0)
#camera_interface.setHVBiasMode(camera.AUTOHV)
#camera_interface.setHybridMode(camera.CDTE)
#camera_interface.setConfigBuild(camera.PX1)
#camera_interface.setHVBiasModePower(camera.HV_ON)
#camera_interface.setCoolingMode(camera.COOLING_ON)
#camera_interface.setHVRefreshPeriod(0)
#camera_interface.setLowThreshold0(1)
#camera_interface.setHighThreshold0(100)
#camera_interface.setLowThreshold1(1)
#camera_interface.setHighThreshold1(100)
#camera_interface.setCoolingTemperatureSetpoint(20.0)
#camera_interface.setNbiMode(camera.SensorConfigNBI_OFF)
#camera_interface.setDeadTimeFreeMode(camera.DeadTimeFreeModeOFF)
#camera_interface.setRunConfigMode(camera.DATA)
#camera_interface.setDelayBetweenFrames(0)
#camera_interface.setHighVoltageBiais (400)











#print "\n\n\n\n ========== ACQ ==========\n "
#acq = control.acquisition()
#saving = control.saving()

#acq.setAcqExpoTime(1)
#acq.setAcqNbFrames(1) ### Crash after 6203 img with 100Mbp

#print control.Status()

##pars=saving.getParameters()
##pars.directory='/tmp/test_lima'
##pars.prefix='test1_'
##pars.suffix='.edf'
##pars.fileFormat=Core.CtSaving.EDF
##pars.savingMode=Core.CtSaving.AutoFrame
##saving.setParameters(pars)

#print "\n\n\n\n ========== ROI ==========\n "
##
##image = control.image()
##maroi = Core.Roi(0,0,100,100)
##image.setRoi(maroi)


#print "\n\n\n\n ========== DTF MODE AND OTHER PIXIRAD SUBTILITIES ==========\n "
##













#
#nbrun = 0
#while True:    

#    pars=saving.getParameters()
#    pars.directory='/tmp/test_lima'
#    pars.prefix='test_'+str(nbrun)+'_'
#    pars.suffix='.edf'
#    pars.fileFormat=Core.CtSaving.EDF
#    pars.savingMode=Core.CtSaving.AutoFrame
#    saving.setParameters(pars)
    
    #### 24 img /s => 0.041 s / img     23 img/s =>0.04347
    #### 23 img/s => 80% 100Mbp
#    acq.setAcqExpoTime(0.04)  # 87% TB100
 #   acq.setAcqExpoTime(0.035)  # 97% TB100
#    acq.setAcqExpoTime(0.01)  
#    acq.setAcqNbFrames(1000) ### Crash after 6203 img
#    acq.setAcqNbFrames(2147483647)  # should be enough

#    print "\n\n\n\n ======= PREPARE ======== \n"
#    control.prepareAcq()
#    
#    print "\n\n\n\n ======= START  ======== \n"    
#    control.startAcq()
#    while (control.getStatus().ImageCounters.LastImageAcquired< acq.getAcqNbFrames()-1):
#        print "WAITING END OF RUN ----     "+str(control.getStatus().ImageCounters.LastImageAcquired) +" / " +str(acq.getAcqNbFrames()-1)
#        time.sleep(0.3)
#        
#    nbrun = nbrun +1
    

#
#
#while True:    
#    
#    print "\n\n\n\n ======= PREPARE ======== \n"
#    afterfirstimage = time.time()+1 # Update when acq really start on the detector level.
#    
#    beforePrepare = time.time()
#    control.prepareAcq()
#    
#    print "\n\n\n\n ======= START  ======== \n"
#    
#    control.startAcq()
#    
#    prev = -1
#    
#    total = 0
#    
#    while True:
#        if(total == 1):afterfirstimage = time.time()
#        now = control.getStatus().ImageCounters.LastImageAcquired
#    
#        print str(total)+ "  total: " +str(now) + " rate (with prep): "+str(now/(time.time() - beforePrepare))   +" img/s ,  rate (no prep): "+str(now/(time.time() - afterfirstimage ))   +" img/s ,  time lost due to prepare "+str(afterfirstimage - beforePrepare) + "    rate instant: "+str(now-prev)+" img/s"
#        total = total +1
#        prev = now
#        time.sleep(1)
#        if(now == acq.getAcqNbFrames()-1): break
#    
#
#
#
#
#
#
#
###### SPEED ??? ######  DTF
##
#camera_interface.setDeadTimeFreeMode(camera.DeadTimeFreeModeON)
##
#camera_interface.setColorMode(camera.COLMODE_DTF)
##
#camera_interface.setNbiMode(camera.SensorConfigNBI_ON)

#time.sleep(3)
##
#print "\n\n\n\n ======= PREPARE ======== \n"

#acq.setAcqExpoTime(0.001)
#acq.setAcqNbFrames(1000)

#afterfirstimage = time.time()+1 # Update when acq really start on the detector level.

#beforePrepare = time.time()
#control.prepareAcq()
##time.sleep(5)
#print "\n\n\n\n ======= START  ======== \n"

#control.startAcq()

#prev = -1

#total = 0

#while True:
   #if(total == 1):afterfirstimage = time.time()
   #now = control.getStatus().ImageCounters.LastImageAcquired

   #print str(total)+ "  total: " +str(now) + " rate (with prep): "+str(now/(time.time() - beforePrepare))   +" img/s ,  rate (no prep): "+str(now/(time.time() - afterfirstimage ))   +" img/s ,  time lost due to prepare "+str(afterfirstimage - beforePrepare) + "    rate instant: "+str(now-prev)+" img/s"
   #total = total +1
   #prev = now
   #time.sleep(1)
   #if(now == acq.getAcqNbFrames()-1): break







### TEST NEW SERVER FROM MASSIMO FOR PIX8

#time.sleep(20)

#acq.setAcqExpoTime(0.001)
#acq.setAcqNbFrames(1000)

#while True:
    #beforePrepare = time.time()
    #control.prepareAcq()
    #beforeAcq = time.time()
    #control.startAcq()
    
    #while (control.getStatus().ImageCounters.LastImageAcquired!=0):
        #pass

    #afterAcq = time.time()
    #print "duree prepare" + str(beforeAcq- beforePrepare)+ " duree acq " + str(afterAcq-beforeAcq)    
    ###time.sleep(1)

#acq.setAcqExpoTime(1)
#acq.setAcqNbFrames(1)

#print control.Status()

#print "\n\n\n\n =======    PREPARE     ======== \n"
#control.prepareAcq()
#print "\n\n\n\n =======     START      ======== \n"
#control.startAcq()
#print "\n\n\n\n =======  END OF START  ======== \n"

##while (control.getStatus().ImageCounters.LastImageAcquired!=0):
    ##pass
#time.sleep(5)

#acq.setAcqExpoTime(1.5)
#acq.setAcqNbFrames(1)
#print "\n\n\n\n =======    PREPARE     ======== \n"
#print control.Status()
#control.prepareAcq()
#print "\n\n\n\n =======     START      ======== \n"
#print control.Status()
#control.startAcq()
#print "\n\n\n\n =======  END OF START  ======== \n"
#print control.Status()


#print "\n\n\n\n =======  PAUSE ======== \n"

#time.sleep(20)

#print "\n\n\n\n =======  EXIT ======== \n"


#while (control.getStatus().ImageCounters.LastImageAcquired!=0):
    #pass
    #time.sleep(1)
    #print control.Status()

##fig = plt.figure()
#ax = fig.add_subplot(111)
#cax = ax.imshow(ic, interpolation='nearest', origin='lower')
#ax.set_title('pixel 2D, parallel readout')
#cbar = fig.colorbar(cax)
#plt.show()


#
##time.sleep(20000)
#print "\n\n\n\n ======= FORCE  STOP  DURING AQUISITION======== \n"
#
##
#control.stopAcq()
#
#
#
#time.sleep(5)
#
#print "\n\n\n\n ======= PREPARE 2 ======== \n"
#
#
#control.prepareAcq()
#time.sleep(5)
#print "\n\n\n\n ======= START 2  ======== \n"
#
#control.startAcq()



#time.sleep(999999)

