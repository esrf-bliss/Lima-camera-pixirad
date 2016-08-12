.. _camera-pixirad:

PIXIRAD (PX1 and PX8) camera plugin
-----------------------------------


.. image:: Comp-PIXIRAD8.png
   :scale: 50 %
.. image:: PIXIRAD1_photos.png

Introduction
````````````
PIXIRAD Imaging Counters s.r.l. is an INFN Spin-off company introducing an innovative, high quality X-ray imaging sensor with intrinsic digital characteristics. It is based on Chromatic Photon Counting technology and represents a radical leap forward compared to the standard methods currently on the market.

The PIXIRAD imaging sensors are able to count individually the incident X-ray photons and to separate them in real time according to their energy (two color images per exposure).

- Global count rate > 200 GHz
- Energy range 1-100 keV
- Energy resolution better than 2 keV (FWHM) @20 keV

Installation & Module configuration
````````````````````````````````````
-  follow first the steps for the linux installation :ref:`linux_installation`

The minimum configuration file is *config.inc* :

.. code-block:: sh

  COMPILE_CORE=1
  COMPILE_SIMULATOR=0
  COMPILE_SPS_IMAGE=1
  COMPILE_ESPIA=0
  COMPILE_FRELON=0
  COMPILE_MAXIPIX=0
  COMPILE_PILATUS=0
  COMPILE_POINTGREY=0
  COMPILE_BASLER=0
  COMPILE_PIXIRAD=1
  COMPILE_CBF_SAVING=0
  export COMPILE_CORE COMPILE_SPS_IMAGE COMPILE_SIMULATOR \
         COMPILE_ESPIA COMPILE_FRELON COMPILE_MAXIPIX COMPILE_PILATUS \
         COMPILE_POINTGREY COMPILE_PIXIRAD COMPILE_BASLER COMPILE_CBF_SAVING

-  start the compilation :ref:`linux_compilation`

-  finally for the Tango server installation :ref:`tango_installation`


Initialisation and Capabilities
````````````````````````````````
Implementing a new plugin for new detector is driven by the LIMA framework but
the developer has some freedoms to choose which standard and specific features will be make available. This section is supposed to give you the correct information regarding how the camera is exported within the LIMA framework.


Camera initialisation
......................

The camera has to be initialized using the Pixirad::Camera class. The default constructor does accept parameters:

Std capabilities
................

This plugin has been implement in respect of the mandatory capabilites but with some limitations which are due to the camera and SDK features.  We only provide here extra information for a better understanding of the capabilities.

* HwDetInfo

TODO
* HwSync

 - The minimum latency time is 1 ms.

 - The supported trigger modes are depending of the chosen frame mode. There are:

   - IntTrig
   - ExtTrigMult

Optional capabilities
........................

* HwReconstruction

TODO

Specific control parameters
.............................

Some specific parameters are available within the camera hardware interface. Those parameters should be used carefully, please refer to the camera SDK (or user's guide) documentation for further information.


.. code-block:: cpp

        void autocalibration();
	
        void setHighThreshold0(float t);
        void getHighThreshold0(float& t) ;
        void setLowThreshold0(float t);
        void getLowThreshold0(float& t) ;
        void setHighThreshold1(float t);
        void getHighThreshold1(float& t) ;
        void setLowThreshold1(float t);
        void getLowThreshold1(float& t) ;
        void setDeadTimeFreeMode(Camera::DeadTimeFreeMode dtf) ;
        void getDeadTimeFreeMode(Camera::DeadTimeFreeMode &dtf) ;
        void setNbiMode(Camera::SensorConfigNBI nbi) ;
        void getNbiMode(Camera::SensorConfigNBI &nbi) ;
        void setAsicMode(Camera::SensorConfigASIC asic);
        void getAsicMode(Camera::SensorConfigASIC &asic);
        void setHybridMode(Camera::SensorConfigHybrid hybrid);
        void getHybridMode(Camera::SensorConfigHybrid &hybrid);
	void setSensorConfigBuild(Camera::SensorConfigBuild build);
	void getSensorConfigBuild(Camera::SensorConfigBuild &build);
        void setRunConfigMode(Camera::RunConfigMode mode);
        void getRunConfigMode(Camera::RunConfigMode &mode);
        void setCoolingTemperatureSetpoint(float t);
        void getCoolingTemperatureSetpoint(float& t) ;
        void setCoolingMode(Camera::CoolingMode mode);
        void getCoolingMode(Camera::CoolingMode &mode);
        void setHighVoltageBiais(float hv);
        void getHighVoltageBiais(float& hv) ;
        void setHVBiasModePower(Camera::HVBiaisPower mode);
        void getHVBiasModePower(Camera::HVBiaisPower &mode);
        void setHVBiasMode(Camera::HVMode mode);
        void getHVBiasMode(Camera::HVMode &mode);
        void setHighVoltageDelayBeforeOn(float sec);
        void getHighVoltageDelayBeforeOn(float& sec) ;
        void setHVRefreshPeriod(int nbOfImages);
        void getHVRefreshPeriod(int& nbOfImages);
        void setDelayBetweenFrames(int delayms);
        void getDelayBetweenFrames(int& delayms);
        void setColorMode(Camera::ColorMode color);
        void getColorMode(Camera::ColorMode &color);
	
	
	void setTrsfMode(Camera::TrsfMode mode);
	void getTrsfMode(Camera::TrsfMode &mode);

// UDP
	void setNCyclesUdpDelay(int nbcycles);
	void getNCyclesUdpDelay(int& nbcycles);
	
	void setSyncOutFunction(Camera::SyncOutFunction mode);
	void getSyncOutFunction(Camera::SyncOutFunction &mode);
	void setSyncOutPol(Camera::Polarity mode);
	void getSyncOutPol(Camera::Polarity &mode);
	void setSyncInPol(Camera::Polarity mode);
	void getSyncInPol(Camera::Polarity &mode);
	
	
	
	// Weather variable extracted from UDP stream, needs get/set
	void getTemperaturePeltierCold(float& information);
	void getTemperaturePeltierHot(float& information);
	void getHighVoltageTension(float& information);
	void getBoxHumidity(float& information);
	void getBoxTemperature(float& information);
	void getPeltierPower(float& information);
	
	void getAlarmTempTooHot(bool& information);
	void getAlarmTempTooHotEnabled(bool& information);
	void getAlarmTempTooCold(bool& information);
	void getAlarmTempTooColdEnabled(bool& information);
	void getAlarmHumidity(bool& information);
	void getAlarmHumidityEnabled(bool& information);


Basic network configuration
````````````````````````````

The camera has 192.168.0.1/24 adress. The detector pc has to be configured likewise.
The recommended option is to have one good quality network interface dedicated to the pixirad, and one for the rest of the world.

- Case one (Recommended), dedicated interface:

 .. code-block:: sh

   auto eth1
   iface eth1 inet static
   address 192.168.0.100
   netmask 255.255.255.0
   mtu 1500


- Case two, one interface, with a router handling two subnetworks:

 Configuration with an alias on interface eth0:

 .. code-block:: sh

   auto eth0:1
   iface eth0:1 inet static
   address 192.168.0.100
   netmask 255.255.255.0
   mtu 1500
               
            
Test examples
``````````````        
            
With python 
............

- Test directly the camera within python:


 .. code-block:: python

   from Lima import Core
   from Lima import Pixirad as PixiradAcq
   



- Set the number of image treatment threads according to the number of CPU available on your mighty machine :

 .. code-block:: python

   Core.Processlib.PoolThreadMgr.get().setNumberOfThread(20)



- Create your camera with its network settings and model (PX8 or PX1)

 .. code-block:: python

   print "\n\n\n\n ======= INIT ======== \n"
   camera = PixiradAcq.Camera("192.168.0.1", 2222, "PX8")
   camera.init() 


 .. code-block:: python

   print "\n\n\n\n ======= INTERFACE ======== \n"
   camera_interface = PixiradAcq.Interface(camera)
   # Set some feature (check manual) 
   # color mode (only 1 col mode supported)
   camera_interface.setColorMode(camera.COLMODE_1COL0)
   # Set point (more than acheavable by the peliter to have full powa):
   camera.setCoolingTemperatureSetpoint(-50) 
   # Set some energy thresholds (check manual, as they will fall in gain level (ranges of energy).
   camera.setLowThreshold0(10)
   camera.setHighThreshold0(60)
   camera.setLowThreshold1(10)
   camera.setHighThreshold1(60)
   # Some high tension management 
   camera.setHighVoltageBiais(2100)
   camera.setHVBiasModePower(1)
   camera.setHighVoltageDelayBeforeOn(3)
   camera.setHVRefreshPeriod(1000);
   # some ethernet interface 
   camera_interface.setTrsfMode(camera.UNMOD)


 .. code-block:: python

   # Get control over things:
   print "\n\n\n\n ======= CONTROL ======== \n"
   control = Core.CtControl(camera_interface)
   # set how much you want lima to buffer memory for treatment.
   control.buffer().setMaxMemory(70)


 .. code-block:: python

   # Get the object with whom you will play :
   print "\n\n\n\n ======= ACQUISITION OBJECT ======== \n"
   acq = control.acquisition()
   # Define trigger:
   acq.setTriggerMode(Core.IntTrig)
   #acq.setTriggerMode(Core.ExtTrigMult)



 .. code-block:: python

   # save somewhere
   saving = control.saving()
   pars=newsaving.getParameters()
   pars.directory='/tmp/test'
   pars.prefix=basename
   pars.suffix='.edf'
   pars.fileFormat=Core.CtSaving.EDF
   pars.savingMode=Core.CtSaving.AutoFrame
   saving.setParameters(pars)

 .. code-block:: python

   # Take images !
   # expo time for one frame :
   acq.setAcqExpoTime(0.01)
   # number of frames:
   acq.setAcqNbFrames(10)
   # get it !
   control.prepareAcq(); 
   control.startAcq()

 .. code-block:: python

   # pretty ones now !
   # Take many (100) images and accumulate them to have better stats and one image written:
   acq.setAcqMode(Core.Accumulation)
   # Max expo time per frame:
   acq.setAccMaxExpoTime(0.01)
   # Total time for the accumulation:
   acq.setAcqExpoTime(1);
   # how many accumulated images:
   acq.setAcqNbFrames(1)
   # get them all and keep one:
   control.prepareAcq(); 
   control.startAcq()




With Tango
..........

- Properties 

 .. code-block:: sh

   initial_model = PX8   // or PX1
   ip_address    = 192.168.0.1
   port_number   = 2222


- PyTango client connection examples:

 .. code-block:: python

   import PyTango
   pixi = PyTango.DeviceProxy("d05/pixirad/pixirad")
   limaccd = PyTango.DeviceProxy("d05/pixirad/pixirad8")
   pixi.cooling_temperature_setpoint = -50
   pixi.high_voltage_biais = 2100
   pixi.dead_time_free_mode = 'DEAD_TIME_FREE_MODE_OFF'
   pixi.color_mode = 'COLMODE_1COL0'
   pixi.low_threshold0 =  1
   pixi.high_threshold0 = 99
   pixi.low_threshold1 =  1
   pixi.high_threshold1 = 99
   #pixi.sensor_config_build = 'PX8'
   pixi.h_v_bias_mode_power = 1
   pixi.trsf_mode = "UNMOD"
   limaccd.buffer_max_memory = 80	
   limaccd.acq_nb_frames = 0
   limaccd.acq_expo_time = 0.01
   limaccd.prepareAcq()
   limaccd.startAcq()


                           
Advanced configuration and optimization  (**optional**)
````````````````````````````````````````````````````````
The camera will send the images as small (1490) udp datagrams, as fast as it can, nearly saturating the bandwidth of the 1Gb ethernet link.
Bad network cards, or high latency systems will result in a loss of part of the image. 
If this happens, several points needs checking. The ethernet card driver might drop packets (and as they are UDP, there won't be any chace to see them). 
The linux kernel UDP buffer might saturate and willingly drop packets (but you knows it at least). In this case, it means that your reading loop (reading from the linux udp buffer) is too slow.


- Using FIFO realtime mode can help.
- Tuning network buffers can help.
- Changing ethernet card can save your skin, and avoid you loosing weeks fine tuning muddy cards.

Realtime mode
.............

In : /etc/security/limits.conf add : 

.. code-block:: sh

   username    -       rtprio  5

In soft :

.. code-block:: cpp

   pthread_t this_thread = pthread_self(); 
   struct sched_param params;     
   params.sched_priority = 5; 
   ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);
   if (ret != 0) { std::cout << "Check /etc/security/limits.conf " << std::endl; }

Kernel tuning
.............
man udp

Change in /etc/sysctl.conf and validate with sysctl -p

.. code-block:: sh

   net.core.rmem_max = 256217728
   net.core.wmem_max = 256217728
   net.ipv4.udp_mem = 131072 262144 524288
   net.ipv4.udp_rmem_min = 65536
   net.core.netdev_max_backlog = 65536 
   net.core.somaxconn = 1024

Network card driver tuning
..........................
.. code-block:: sh

   ethtool -g eth1
   Ring parameters for eth1:
   Pre-set maximums:
   RX:		4096
   RX Mini:	0
   RX Jumbo:	0
   TX:		4096
   Current hardware settings:
   RX:		512         <<<<<< =====
   RX Mini:	0
   RX Jumbo:	0
   TX:		512

Increased with :

.. code-block:: sh

   ethtool -G eth1 rx 4096


Troubleshootings
````````````````
UDP debug tips
...............
  If you suspect drop of UDP datagram due to a too small kernel buffer (the plugin is too slow to treat the buffer, it filled and drop frames)
  
.. code-block:: sh

  cat /proc/net/udp
  
  And check the drop column.
  
.. code-block:: sh

  cat /proc/sys/net/core/rmem_max  

  tells you the buffer size 
  by default : 131071
  
  Enough for 100 images:
  
.. code-block:: sh

  net.core.rmem_max = 507217408


Possible problems with network
...............................
== List of known to work cards ==
= Embedded motherboard card on optiplex 980:
Intel Corporation 82578DM Gigabit Network Connection (rev 05)

== List of non working cards ==
= Intel pro 1000 on PCI card (82541GI) (debian 7 & 9):
Intel Corporation 82541GI Gigabit Ethernet Controller
Intel Corporation 82541PI Gigabit Ethernet Controller (rev 05)




Possible problems with Chillers
...............................
Symptoms : strippy images

The goal is to setup your temperature settings as to have the peltier full time @ max power.
If the peltier is regulating the temperature, stripes appears in the images.
A easy way is to setup a -50C unreachable goal for the detector and let it stabilise to wathever temperature it can reach based on chiller setting.
Chiller is supposed to be set at 16degC. Going bellow needs a hutch humidity well controlled.






