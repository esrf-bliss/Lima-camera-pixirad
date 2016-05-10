//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2015
// European Synchrotron Radiation Facility
// BP 220, Grenoble 38043
// FRANCE
//
// This is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//###########################################################################

#include <sstream>
#include <iostream>
#include <string>
#include <math.h>
#include <climits>
#include <iomanip>
#include <errno.h>

#include "lima/Exceptions.h"
#include "lima/Debug.h"
#include "lima/MiscUtils.h"
#include "PixiradCamera.h"

#include "pixiradDetector.h"

#include <thread>
#include <mutex>
#include <condition_variable>

#include <unistd.h> // sleep

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION  // Silly: warning "Using deprecated NumPy API, disable it by " 



using namespace lima;
using namespace lima::Pixirad;
using namespace std;

lima::Pixirad::Camera::Camera(std::string hostname, int tcpPort) :
  m_hostname(hostname), m_tcpPort(tcpPort) {

	DEB_CONSTRUCTOR();
   
	DEB_TRACE() << "Pixirad attemping to create detector object for : " << DEB_VAR2(m_hostname, m_tcpPort);
    
//        pixiradDetector *m_pixirad = new pixiradDetector(m_hostname, m_tcpPort);
//   	m_bufferCtrlObj = new SoftBufferCtrlObj(); 
	
        m_pixirad = new pixiradDetector(m_hostname, m_tcpPort, m_bufferCtrlObj);
	
	init();

}

lima::Pixirad::Camera::~Camera() {
	DEB_DESTRUCTOR();
        Camera::stopAcq();
	delete m_pixirad;
}

void lima::Pixirad::Camera::init() {
	DEB_MEMBER_FUNCT();
	
        std::string command;
        
	
	m_pixirad->setStatusDetector(HwInterface::StatusType::Config);
	
	
        // This is need to be done only once for each detector, and probably for each new version of the pixirad server interface.
        
        // m_pixirad->m_flagSetFake     = false;
        // m_pixirad->m_flagSensorModelConfig  = false;
        
//         command = string("! Set_Fake "+m_pixirad->m_fake);    
//         DEB_TRACE() << "Sending first command "<<DEB_VAR1(command);	
        // m_pixirad->sendCommand(command);
        
      //  command = string("! Sensor_Model_Config "+m_pixirad->m_sensorConfigAsic + " " +m_pixirad->m_sensorConfigHybrid +" " +m_pixirad->m_sensorConfigBuild );   
      //  m_pixirad->sendCommand(command);
        
        
        
        
        
      //  DEB_TRACE() << "Waiting acknowledgment from detector"<<DEB_VAR2(m_pixirad->m_flagSetFake,m_pixirad->m_flagSensorModelConfig);  
        
        // We wait until detector acknowledge both commands.
        // wait for set fake flag
       // {
       //     std::unique_lock<std::mutex> uniqLock(m_pixirad->m_mutexRX);
      //      m_pixirad->m_condVariableRX.wait(uniqLock, [&]{return m_pixirad->m_flagSetFake;});
//             uniqLock.unlock();
     //   }
        //wait for sensor model config flag.
//         DEB_TRACE() <<DEB_VAR2(m_pixirad->m_flagSetFake,m_pixirad->m_flagSensorModelConfig);  
        
//         {
//             std::unique_lock<std::mutex> uniqLock(m_pixirad->m_mutexRX);
//             m_pixirad->m_condVariableRX.wait(uniqLock, [&]{return m_pixirad->m_flagSensorModelConfig;});
//             uniqLock.unlock();
//         } 
              
              
//         DEB_TRACE() << "Acknowledgment from detector received"<<DEB_VAR2(m_pixirad->m_flagSetFake,m_pixirad->m_flagSensorModelConfig);  
        
        
	
	/*
	

	
	*/
	
	/*
	// Test :
	sprintf(temp_str,"DAQ:! SET_SENSOR_OPERATINGS %d %d %d %d %d %d %d %s %s\n"
	,0
	,0
	,0
	,0
	,0
	,0
	,0
	,"test"
	,"test"
	);
	m_pixirad->pixirad_send_cmd(temp_str, NULL,0,0);
	*/
	
	
	
// 	SYS:! SET_ALARM_MSG_DEST_ADD Ipaddress port
// 	Parameters
// 	port
// 	IPaddress
// 	(N) destination port number;
// 	(S) dot decimal notation;
// 	Description
// 	Sets the Alarm message UDP packet destination IP address and port number
// 	(by default it is set to be broadcasted on port 2225);


// TODO


// SYS:! SET_STATUS_MSG_DEST_ADD Ipaddress port
// Parameters
// port
// IPaddress
// (N) destination port number;
// (S) dot decimal notation;
// Description
// Sets the detector Status message UDP packet destination IP address and port
// number (by default it is set to broadcasted on port 2224);

// TODO



// SYS:! SET_MEAS_DEST_ADD Ipaddress port
// Parameters
// port
// IPaddress
// (N) destination port number;
// (S) dot decimal notation;
// Description
// Sets the detector Data Measurement UDP packets destination IP address and port
// number (by default it is set to broadcasted on port 2223);

// TODO


// DAQ:! SET_SLOT_ID_AUTORESET mode
// Parameters
// mode
// (N) enabled/disabled
// Description
// if (mode != 0), slot_id is resetted every time a LOOP command is issued.
//   Setting mode to “0” makes slot_id incrementing over consecutive acquisitions.
//   Anyway the first acquisition after this command has been issued will start with
//   slot_id = 0.
//   By default slot_id autoreset is enabled


command = string("DAQ:! SET_SLOT_ID_AUTORESET "+ to_string(m_pixirad->m_autoResetMode) );
res = m_pixirad->sendCommand(command,commandAnswerFromDetector, true);	






resetAllAlarms();




//     SYS:! SET_ALARMS Humidity_al_th Tcold_al_th Thot_al_th humidity_al_en
//     Tcold_al_en Thot_al_en Alarm_msg_lvl
//     Description
//     Sets the alarms threshold levels and status. Envinromental conditions are
//     continuosly checked and if they exceed the threshold levels safety actions are
//     automatically taken. Depending on alrm_msg_lvl, an UDP packet message informs
//     on the alarms status. In the following an example of the message:
//     “BOX 1022 ALARMS    
//     THOT_ALARM_STATUS OFF
//     TCOLD_ALARM_STATUS DISABLED
//     HUMIDITY_ALARM_STATUS ON”
//     Alarm status can be “ON”, “OFF” or “DISABLED”. The “ON” status signaled by the
//     message means that alarm limit has been exceeded and the related safety action
//     has been taken. Conversely, “OFF” means that alarm monitoring is activated and
//     limit is not exceeded. When alarm monitoring is disabled, its status is marked
//     “DISABLED”.
//     By default Alarm status message is sent over UDP at port 2225.


	autocalibration();


	m_pixirad->setStatusDetector(HwInterface::StatusType::Ready);
	
}

void lima::Pixirad::Camera::reset() {
	DEB_MEMBER_FUNCT();
	//         m_pixirad->stopSocketThread();
	
	//     SYS:! SYSTEM_RESET [Delay_ms]
	//     Parameters
	//     Delay_ms
	//     (R) Reset Delay in ms after command is received (Optional)
	//     Description
	//     Triggers the Box Reset; if no delay is set, the detector system resets after 500ms.
	
	
	std::string command;
	
	
	resetAllAlarms();
	
	command = string("SYS:! SYSTEM_RESET");
	res = m_pixirad->sendCommand(command,commandAnswerFromDetector, false);
	
	
	
	
}

void lima::Pixirad::Camera::autocalibration() {
	DEB_MEMBER_FUNCT();

/*
DAQ:! AUTOCAL [mode]
Parameters
mode
(S) Autocompensation codes readout option (optional):
“BOTH” to read pre and post calibration codes;
“LAST” to read only post-calibration codes;
“NOCODES” no caòlibration code read-out;
Description
Triggers the Detector Offsets Calibration. If no parameters are specified, bot pre and
	*/

	
	std::string command;
	command = string("DAQ:! AUTOCAL" );
	res = m_pixirad->sendCommand(command,commandAnswerFromDetector, true);	
	DEB_TRACE() << "Autocalibration " <<DEB_VAR1(commandAnswerFromDetector); 
}

void lima::Pixirad::Camera::prepareAcq() {
	DEB_MEMBER_FUNCT();
        
	
 	m_pixirad->prepareAcq(); // initialise local reconstruction buffer and  frame dimension
        
    std::string command;
    
	
    
    
    //     DAQ:! SET_UDP_DELAY ncycles
    // Parameters
    // ncycles
    //  (N) number of delay cycles in measurement data
    // packets transmission
    // Description
    // A 32ns * ncycles delay is inserted in measurement data UDP packets trasmission. It
    // results in a trasmission slowing down.
    // The detector coerces ncycles to 0 - 4.000.000.

    // Test :
    command = string("DAQ:! SET_UDP_DELAY "+ to_string(m_pixirad->m_nCyclesUdpDelay) );	
    res = m_pixirad->sendCommand(command,commandAnswerFromDetector, true);
    DEB_TRACE() << DEB_VAR1(res);
    
    
    
         
    
    
	///////////  ENERGIES and Detector Modes  ////////////////
	
    
//     DAQ:! SET_SENSOR_OPERATINGS HighTh1 LowTh1 HighTh0 LowTh0 VthMax Ref AuFS    Dtf Nbi
//     Description
//     (N) Threshold Set 1 Register 1 Threshold (4th Color)
//     (N) Threshold Set 1 Register 0 Threshold (3rd Color)
//     (N) Threshold Set 1 Register 1 Threshold (2nd Color)
//     (N) Threshold Set 1 Register 0 Threshold (1st Color)
//     (N) Global Threshold Full Scale DAC value
//     (N) Set to “2”
//     (N) Set to “7”
//     (S) “DTF” to configure DeadTimeFree Mode for Detector “NODTF”
//     otherwise
//     (S) “NBI” to configure NBI Mode for Detector “NONBI” otherwise
//     Sets the internal detector configuration and threshold levels. Values in 0-31 range are
//     admitted and translate in number of electrons depending on VthMax value. For Example, if
//     VthMax=1625 and LowTh1=1 discriminator threshold is set to 50e-, LowTh1=2 sets it to
//     100e-, and so on. Please note that the step increases for higher values.
//     Ref and AuFS are detectors internal biasing settings and should be set to “2” and “7”
//     respectively. For a complete correspondence table between Thresholds counts and
//     Threshold Energies please refer to table 3.
    
    
    
	m_pixirad->update_thresolds_from_energies(); // Will fill in VthMax too.
	
	// If we want more than the max returned by update threshold, we keep the max.
	int hvbiais = m_pixirad->m_envConfigHighVoltageBiaisMax;
	if (m_pixirad->m_envConfigHighVoltageBiais <= m_pixirad->m_envConfigHighVoltageBiaisMax){
	   hvbiais = m_pixirad->m_envConfigHighVoltageBiais;
	}
	
	command = string("DAQ:! SET_SENSOR_OPERATINGS "+ to_string(m_pixirad->m_sensorConfigHighThreshold1DAC) +" "+	to_string(m_pixirad->m_sensorConfigLowThreshold1DAC) +" "+	to_string(m_pixirad->m_sensorConfigHighThreshold0DAC) +" "+	to_string(m_pixirad->m_sensorConfigLowThreshold0DAC) +" "+	to_string(hvbiais) +" "+	to_string(m_pixirad->m_sensorConfigRefInternalBiais) +" "+ to_string(m_pixirad->m_sensorConfigAusFSInternalBiais) +" "+	m_pixirad->m_sensorConfigDeadTimeFreeMode +" "+	m_pixirad->m_sensorConfigNBI );
	
	res = m_pixirad->sendCommand(command,commandAnswerFromDetector, true);	
	DEB_TRACE() << DEB_VAR1(res);
	
    	
	
	// 	DAQ:! SET_HV_MNGMT  (BIAIS_management_config)
	
	command = string("DAQ:! SET_HV_MNGMT "+ m_pixirad->m_biaisMode +" "+to_string(m_pixirad->m_tonHvAcquisitionDelay)+" "+to_string(m_pixirad->m_toffHvAcquisitionDelay) +" "+to_string(m_pixirad->m_HvRefreshPeriod)); 
	
	res = m_pixirad->sendCommand(command,commandAnswerFromDetector, true);	
	DEB_TRACE() << DEB_VAR1(res);
	
	
	
	
	
	
// 	DAQ:! INIT CoolT CollStatus HVVal HVStatus
// 	Parameters
// 	CoolT	(N)	Detector	Cooling temperature set (°C)	
// 	CollStatus	(N)	Detector  Coling System Status (“1”: On, “0”:Off)
// 	HVVal	(N)	Detector	Bias Voltage (V)
// 	HVStatus (N)	Detector	Bias Status	(“1”:On ,“0”:Off)
// 	Description
// 	Sets the detector Bias And cooling status;
// 	
	sendCommandForDetectorCooling();
	
	
	
// 	DAQ:! SET_SYNC Sync_in_pol Sync_out_pol Sync_out_function
// 	Sync_in_pol (S) polarity "POS" or "NEG"
// 	Sync_out_pol (S) polarity "POS" or "NEG"
// 	Sync_out_function(S) “SHUTTER”, “RODONE” or “READ”
	
	
	command = string("DAQ:! SET_SYNC "
	+ m_pixirad->m_syncInPolarity +" "
	+ m_pixirad->m_syncOutPolarity  +" "
	+ m_pixirad->m_syncOutFunction  
	); 
	
	res = m_pixirad->sendCommand(command,commandAnswerFromDetector, true);	
	DEB_TRACE() << DEB_VAR1(res);
	
	
	
	
	
	// The PX8 has 8 modules, but you can ask it to use only one of them.   // -1 is 8 modules mode //  [0-7] is module selected.
	
      if (m_pixirad->m_sensorConfigBuild == "PX8" ){
	
	 if (m_pixirad->m_oneChipModeOutOfEight >= 0  or m_pixirad->m_oneChipModeOutOfEight <= 7) {
	    
	      // Do not do this 
	      if(m_pixirad->m_seedModeForDebugOnlyInOneChipWithPX8){
		// What did I say two line above ?
		command = string("DAQ:! SET_DATA_TEST 1"); 	
		res = m_pixirad->sendCommand(command,commandAnswerFromDetector, true);	
		DEB_TRACE() << DEB_VAR1(res);
	      }
	      else {	  
		command = string("DAQ:! SET_DATA_TEST 0"); 	
		res = m_pixirad->sendCommand(command,commandAnswerFromDetector, true);	
		DEB_TRACE() << DEB_VAR1(res);	  
	      }
	      
	      
	    command = string("DAQ:! SET_RO_SCHEMA CHIP" + to_string(m_pixirad->m_oneChipModeOutOfEight)); 	
	    res = m_pixirad->sendCommand(command,commandAnswerFromDetector, true);	
	    DEB_TRACE() << DEB_VAR1(res);
	 }
	 else{
	    // 8 modules mode
	    command = string("DAQ:! SET_RO_SCHEMA DEFAULT");
	    res = m_pixirad->sendCommand(command,commandAnswerFromDetector, true);	    
	    DEB_TRACE() << DEB_VAR1(res);
	 }
	 
	
	
      }
      
	
	
	
	
	
	
	
	
	
	
}

void lima::Pixirad::Camera::sendCommandForDetectorCooling(){
  
  DEB_MEMBER_FUNCT();
  
  std::string command = string("DAQ:! INIT "
  + to_string(m_pixirad->m_envConfigCoolingTemperature) +" "
  + to_string(m_pixirad->m_envConfigCoolingOnOff ) +" "
  + to_string(m_pixirad->m_envConfigHighVoltageBiais ) +" "
  + to_string(m_pixirad->m_envConfigHighVoltageBiaisOnOff )
  ); 
  
  res = m_pixirad->sendCommand(command,commandAnswerFromDetector, true);	
  DEB_TRACE() << DEB_VAR1(res);
  
  
}

void lima::Pixirad::Camera::resetAllAlarms(){
  
  DEB_MEMBER_FUNCT();
  
  // Reset all alarms
  std::string command = string("SYS:! CLEAR_ALARMS");
  res = m_pixirad->sendCommand(command,commandAnswerFromDetector, true);
  
  DEB_TRACE() << DEB_VAR1(res);
  
}




void lima::Pixirad::Camera::startAcq() {
    DEB_MEMBER_FUNCT();
    
    
     StdBufferCbMgr& buffer_mgr = m_bufferCtrlObj.getBuffer();
    
    m_pixirad->getImagesInAThread();
    
	
	std::string command;
	command = string("DAQ:! LOOP "+ to_string(m_pixirad->m_nbFramesAcq) +" " + to_string(m_pixirad->m_shutterMs) +" " +to_string(m_pixirad->m_pauseBetweenAcq) +" " +m_pixirad->m_RunModeColors  +" " +m_pixirad->m_TriggerMode  +" " +m_pixirad->m_moderationMode +" " +m_pixirad->m_HvMode);
	
	char commandAnswerFromDetector[MAX_MSG_STR_LENGTH];
	int res = m_pixirad->sendCommand(command,commandAnswerFromDetector, true);
	
	DEB_TRACE() << DEB_VAR1(res); 
	// For timestamping the starting time
		  buffer_mgr.setStartTimestamp(Timestamp::now());
			      
	
		  
		  /// If using the two threads version
//   	m_pixirad->dispatchLoopForUDPStreamToIndividualImage();
	
	 
}

void lima::Pixirad::Camera::stopAcq() {
	DEB_MEMBER_FUNCT();
	
	std::string command;
	
	command = string("DAQ:!!ACQUISITIONBREAK");
	
	char commandAnswerFromDetector[MAX_MSG_STR_LENGTH];
	int res = m_pixirad->sendCommand(command,commandAnswerFromDetector, true);
	
	m_pixirad->stopAcq();
	
	
	DEB_TRACE() << DEB_VAR2(res,commandAnswerFromDetector); 
	
}
/*
void Camera::getStatus(DetectorStatus& status) {
	DEB_MEMBER_FUNCT();
        std::unique_lock<std::mutex> uniqLock(m_pixirad->m_mutexStatus);
//         status = m_pixirad->m_status;        
}*/

void lima::Pixirad::Camera::getStatusCamera(HwInterface::StatusType::Basic& status) {
  DEB_MEMBER_FUNCT();
  m_pixirad->getStatusDetector(status);
}






void lima::Pixirad::Camera::getStatus(HwInterface::StatusType::Basic& status) {
  DEB_MEMBER_FUNCT();
  // getStatusDetector has its own mutex. do not put it here too.
  m_pixirad->getStatusDetector(status); 
}






void lima::Pixirad::Camera::readFrame(void *bptr, int frame_nb) {
	DEB_MEMBER_FUNCT();
}

int lima::Pixirad::Camera::getNbHwAcquiredFrames() {
	DEB_MEMBER_FUNCT();
	return m_pixirad->m_acq_frame_nb;
}

void lima::Pixirad::Camera::getImageType(ImageType& type) {
	DEB_MEMBER_FUNCT();
	type = lima::Bpp16;
}

void lima::Pixirad::Camera::setImageType(ImageType type) {
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "setImageType " << type;
	if (type != lima::Bpp16)THROW_HW_ERROR(Error) << "Image type " << type << " is not supported";
}

void lima::Pixirad::Camera::getDetectorType(std::string& type) {
	DEB_MEMBER_FUNCT();
	type = "Pixirad";
}

void lima::Pixirad::Camera::getDetectorModel(std::string& model) {
	DEB_MEMBER_FUNCT();
	stringstream ss;
        
	ss << "Pixirad: "  << m_pixirad->m_sensorConfigBuild;
	model = ss.str();
}

void lima::Pixirad::Camera::getDetectorImageSize(Size& size) {
  DEB_MEMBER_FUNCT();
/*
  // Default
  size = Size(0, 0);
  
  SensorConfigBuild build;
  getSensorConfigBuild(build);
  
  if(build == PX1){
    DEB_TRACE() << "Pixel number is selected for a PX1" ;
     size = Size(476, 512);
//    size = Size(512, 476);
  }
  
  if(build == PX8){
    DEB_TRACE() << "Pixel number is selected for a PX8" ;
    size = Size(476, 4096);  // TODO: CHANGE FOR PX8 REAL SIZE
  } 
    */
  
  
  size = Size(0, 0);
  
  m_pixirad->getSize(size);
  
  
}


// TODO: Check with manual pixel size
void lima::Pixirad::Camera::getPixelSize(double& sizex, double& sizey) {
	DEB_MEMBER_FUNCT();
	sizex = 1e-4;
	sizey = 1e-4;
        
	
	SensorConfigBuild build;
	getSensorConfigBuild(build);
	
	if(build == PX1){
            DEB_TRACE() << " getPixelSize is selected for a PX1" ;
	sizex = 60e-6;
	sizey = 60e-6;
        }
        
        
        if(build == PX8){
            DEB_TRACE() << " getPixelSize is selected for a PX8" ;
	sizex = 60e-6;
	sizey = 60e-6; // TODO: CHANGE FOR PX8 REAL SIZE
        }
}


// Interface will need to know the BufferCO
SoftBufferCtrlObj* lima::Pixirad::Camera::getBufferCtrlObj() {
	return &m_bufferCtrlObj;
}

void lima::Pixirad::Camera::setTrigMode(TrigMode mode) {
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setTrigMode() " << DEB_VAR1(mode);
	DEB_TRACE() << "BEFORE\n\n" ; //<< m_pixirad->m_TriggerMode;
	
        m_pixirad->m_TriggerMode = "INT";
            
	switch (mode) {
	case IntTrig:
	    DEB_TRACE() << "Setting internal trigger mode " << DEB_VAR1(mode);
            m_pixirad->m_TriggerMode = "INT";
            break;
	case ExtTrigMult:
	    DEB_TRACE() << "Setting external trigger mode " << DEB_VAR1(mode);
            m_pixirad->m_TriggerMode = "EXT1";
            break;
//         case IntTrigMult:
// 	case ExtTrigSingle:
// 	case ExtGate:
// 	case ExtStartStop:
// 	case ExtTrigReadout:
	default:
		THROW_HW_ERROR(Error) << "Cannot change the Trigger Mode of the camera, this mode is not managed !";
	}
}

void lima::Pixirad::Camera::getTrigMode(TrigMode& mode) {
	DEB_MEMBER_FUNCT();
	if (strcmp(m_pixirad->m_TriggerMode.c_str() ,"INT" )==0 )
            mode = IntTrig;
        else if (strcmp(m_pixirad->m_TriggerMode.c_str() , "EXT1") == 0)
            mode = ExtTrigMult;
            
	DEB_RETURN() << DEB_VAR1(mode);
}

void lima::Pixirad::Camera::getExpTime(double& exp_time) {
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::getExpTime()";
	
	exp_time = m_pixirad->m_shutterMs / 1000;
	DEB_RETURN() << DEB_VAR1(exp_time);
}

void lima::Pixirad::Camera::setExpTime(double exp_time) {
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setExpTime() " << DEB_VAR1(exp_time);

        if (exp_time * 1000 >= 1){
            m_pixirad->m_shutterMs = exp_time * 1000;
        }
        else THROW_HW_ERROR(Error) << "Expo time impossible for this detector";
}

void lima::Pixirad::Camera::setLatTime(double lat_time) {
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Set latency time (s)" << DEB_VAR1(lat_time);
        
        
        m_pixirad->m_pauseBetweenAcq = lat_time * 1000;
	DEB_TRACE() << "Latency time setted for (ms): " << DEB_VAR1(m_pixirad->m_pauseBetweenAcq);
}

void lima::Pixirad::Camera::getLatTime(double& lat_time) {
	DEB_MEMBER_FUNCT();
	lat_time = m_pixirad->m_pauseBetweenAcq / 1000 + m_pixirad->latency_measured;
}

void lima::Pixirad::Camera::getExposureTimeRange(double& min_expo, double& max_expo) const {
	DEB_MEMBER_FUNCT();
	min_expo = 1/1000 ;
	max_expo = 999; // Waiting for an answer from Massimo
	DEB_RETURN() << DEB_VAR2(min_expo, max_expo);
}

void lima::Pixirad::Camera::getLatTimeRange(double& min_lat, double& max_lat) const {
	DEB_MEMBER_FUNCT();	
	min_lat = 0.;
	max_lat = 999;// Waiting for an answer from Massimo
	DEB_RETURN() << DEB_VAR2(min_lat, max_lat);
}

void lima::Pixirad::Camera::setNbFrames(int nb_frames) {
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setNbFrames() " << DEB_VAR1(nb_frames);
	
	// For video mode we cheat by putting a good 32bit number as the number of shots. stopacq will stop the video anyway.  
// 	 3*60*60*24*7 = 	1814400   7 days of video at 3fps.
	
        if(nb_frames == 0){
	  nb_frames = 1814400; // This will take some time.
	  
// 	  if(m_pixirad->get) 
// Adjust dead time 
	  
        }
        
	m_pixirad->m_nbFramesAcq = nb_frames;
	m_nb_frames = nb_frames;
}

void lima::Pixirad::Camera::getNbFrames(int& nb_frames) {
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::getNbFrames";
	nb_frames = m_pixirad->m_nbFramesAcq ;
//	nb_frames = m_nb_frames;
}

bool lima::Pixirad::Camera::isAcqRunning() const {    
	return m_pixirad->m_flagAcquisitionRunning ;
}













///////////////////////     SPECIFIC PIXIRAD     ///////////////////////



// FAKE //

void lima::Pixirad::Camera::setFakeMode(FakeMode fake) {
 	DEB_MEMBER_FUNCT();
        switch(fake){
            case FakeON:
                m_pixirad->m_fake = "On" ;
                break;
            default:
                m_pixirad->m_fake = "Off" ;
        }
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_fake,fake);
                
}


void lima::Pixirad::Camera::getFakeMode(FakeMode& fake) {
	DEB_MEMBER_FUNCT();
        if(m_pixirad->m_fake.compare("On") == 0 )
            fake = FakeON;
        else
            fake = FakeOFF;
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_fake,fake);
}

// UDP DELAY NCYCLES


void lima::Pixirad::Camera::setNCyclesUdpDelay(int nbcycles){
	DEB_MEMBER_FUNCT();    
        if (nbcycles <=4000000 && nbcycles>=0){
            m_pixirad->m_nCyclesUdpDelay = nbcycles;
        }
        else THROW_HW_ERROR(Error) << "Please Think. You are setting a number of hardware cycles to delay between to UDP datagrams, negative number is silly, and you really don't want to wait for more than 4millions cycles.";
}

void lima::Pixirad::Camera::getNCyclesUdpDelay(int& nbcycles) {
	DEB_MEMBER_FUNCT();    
        nbcycles = m_pixirad->m_nCyclesUdpDelay ;
}






// THRESHOLDS 



void lima::Pixirad::Camera::setHighThreshold0(float t){
	DEB_MEMBER_FUNCT();    
        if (t <=100 && t>=0){
            m_pixirad->m_sensorConfigHighThreshold0 = t;
        }
        else THROW_HW_ERROR(Error) << "Threshold Value Impossible";
}

void lima::Pixirad::Camera::getHighThreshold0(float& t) {
	DEB_MEMBER_FUNCT();    
        t = m_pixirad->m_sensorConfigHighThreshold0 ;
}


void lima::Pixirad::Camera::setLowThreshold0(float t){
	DEB_MEMBER_FUNCT();    
        if (t <=100 && t>=0){
            m_pixirad->m_sensorConfigLowThreshold0 = t;
        }
        else THROW_HW_ERROR(Error) << "Threshold Value Impossible";
}

void lima::Pixirad::Camera::getLowThreshold0(float& t) {
	DEB_MEMBER_FUNCT();    
        t = m_pixirad->m_sensorConfigLowThreshold0 ;
}



void lima::Pixirad::Camera::setHighThreshold1(float t){
	DEB_MEMBER_FUNCT();    
        if (t <=100 && t>=0){
            m_pixirad->m_sensorConfigHighThreshold1 = t;
        }
        else THROW_HW_ERROR(Error) << "Threshold Value Impossible";
}

void lima::Pixirad::Camera::getHighThreshold1(float& t) {
	DEB_MEMBER_FUNCT();    
        t = m_pixirad->m_sensorConfigHighThreshold1 ;
}


void lima::Pixirad::Camera::setLowThreshold1(float t){
	DEB_MEMBER_FUNCT();    
        if (t <=100 && t>=0){
            m_pixirad->m_sensorConfigLowThreshold1 = t;
        }
        else THROW_HW_ERROR(Error) << "Threshold Value Impossible";
}

void lima::Pixirad::Camera::getLowThreshold1(float& t) {
	DEB_MEMBER_FUNCT();    
        t = m_pixirad->m_sensorConfigLowThreshold1 ;
}







// Dead time free mode //

void lima::Pixirad::Camera::setDeadTimeFreeMode(DeadTimeFreeMode dtf) {
	DEB_MEMBER_FUNCT();
        switch(dtf){
            case DeadTimeFreeModeON:
                m_pixirad->m_sensorConfigDeadTimeFreeMode = "DTF" ;
                break;
            default:
                m_pixirad->m_sensorConfigDeadTimeFreeMode = "NODTF" ;
        }
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_sensorConfigDeadTimeFreeMode,dtf);
                
}


void lima::Pixirad::Camera::getDeadTimeFreeMode(DeadTimeFreeMode &dtf) {
	DEB_MEMBER_FUNCT();
        if(m_pixirad->m_sensorConfigDeadTimeFreeMode.compare("DTF") == 0 )
            dtf = DeadTimeFreeModeON;
        else
            dtf = DeadTimeFreeModeOFF;
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_sensorConfigDeadTimeFreeMode,dtf);
}



        // SensorConfig NBI

void lima::Pixirad::Camera::setNbiMode(SensorConfigNBI nbi) {
	DEB_MEMBER_FUNCT();
        switch(nbi){
            case SensorConfigNBI_ON:
                m_pixirad->m_sensorConfigNBI = "NBI" ;
                break;
            default:
                m_pixirad->m_sensorConfigNBI = "NONBI" ;
        }
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_sensorConfigNBI,nbi);
}


void lima::Pixirad::Camera::getNbiMode(SensorConfigNBI &nbi) {
	DEB_MEMBER_FUNCT();
        if(m_pixirad->m_sensorConfigNBI.compare("NBI") == 0 )
            nbi = SensorConfigNBI_ON;
        else
            nbi = SensorConfigNBI_OFF;
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_sensorConfigNBI,nbi);
}





        // SensorConfig ASIC

void lima::Pixirad::Camera::setAsicMode(SensorConfigASIC asic){
	DEB_MEMBER_FUNCT();
        switch(asic){
            case PIII:
                m_pixirad->m_sensorConfigAsic = "PIII" ;
                break;
            default:
                m_pixirad->m_sensorConfigAsic = "PII" ;
        }
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_sensorConfigAsic,asic);
}


void lima::Pixirad::Camera::getAsicMode(SensorConfigASIC &asic){
	DEB_MEMBER_FUNCT();
        if(m_pixirad->m_sensorConfigAsic.compare("PIII") == 0 )
            asic = PIII;
        else
            asic = PII;
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_sensorConfigAsic,asic);
}



        // SensorConfig ASIC

void lima::Pixirad::Camera::setHybridMode(SensorConfigHybrid hybrid){
	DEB_MEMBER_FUNCT();
        switch(hybrid){
            case GAAS:
                m_pixirad->m_sensorConfigHybrid = "GAAS" ;
                break;
            default:
                m_pixirad->m_sensorConfigHybrid = "CDTE" ;
        }
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_sensorConfigHybrid,hybrid);
}


void lima::Pixirad::Camera::getHybridMode(SensorConfigHybrid &hybrid){
	DEB_MEMBER_FUNCT();
        if(m_pixirad->m_sensorConfigHybrid.compare("GAAS") == 0 )
            hybrid = GAAS;
        else
            hybrid = CDTE;
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_sensorConfigHybrid,hybrid);
}




        // SensorConfig Build

void lima::Pixirad::Camera::setSensorConfigBuild(SensorConfigBuild build){
	DEB_MEMBER_FUNCT();
        switch(build){
            case PX2:
                m_pixirad->m_sensorConfigBuild = "PX2" ;
                break;
            case PX8:
                m_pixirad->m_sensorConfigBuild = "PX8" ;
                break;
            default:
                m_pixirad->m_sensorConfigBuild = "PX1" ;
        }
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_sensorConfigBuild,build);
}


void lima::Pixirad::Camera::getSensorConfigBuild(SensorConfigBuild &build){
  DEB_MEMBER_FUNCT();
  if(m_pixirad->m_sensorConfigBuild.compare("PX1") == 0 )
    build = PX1;
  else if(m_pixirad->m_sensorConfigBuild.compare("PX8") == 0 )
    build = PX8;
  else
    build = PX2;
  DEB_TRACE() << DEB_VAR2(m_pixirad->m_sensorConfigBuild,build);
	
}



      // Run Config Modes 

void lima::Pixirad::Camera::setRunConfigMode(RunConfigMode mode){
	DEB_MEMBER_FUNCT();
        switch(mode){
            case SUMMED_DATA:
                m_pixirad->m_runConfigRunMode = "SUMMED_DATA" ;
                DEB_TRACE() << DEB_VAR2(m_pixirad->m_runConfigRunMode,mode);
                break;
            case COLLECT_FLAT:
                m_pixirad->m_runConfigRunMode = "COLLECT_FLAT" ;
                DEB_TRACE() << DEB_VAR2(m_pixirad->m_runConfigRunMode,mode);
                break;
            case SUMMED_COLLECT_FLAT:
                m_pixirad->m_runConfigRunMode = "SUMMED_COLLECT_FLAT" ;
                DEB_TRACE() << DEB_VAR2(m_pixirad->m_runConfigRunMode,mode);
                break;
            default:
                m_pixirad->m_runConfigRunMode = "DATA" ;
                DEB_TRACE() << DEB_VAR2(m_pixirad->m_runConfigRunMode,mode);
        }
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_runConfigRunMode,mode);
}


void lima::Pixirad::Camera::getRunConfigMode(RunConfigMode &mode){
	DEB_MEMBER_FUNCT();
        if(m_pixirad->m_runConfigRunMode.compare("SUMMED_DATA") == 0 )
            mode = SUMMED_DATA;
        else if(m_pixirad->m_runConfigRunMode.compare("COLLECT_FLAT") == 0 )
            mode = COLLECT_FLAT;
        else if(m_pixirad->m_runConfigRunMode.compare("SUMMED_COLLECT_FLAT") == 0 )
            mode = SUMMED_COLLECT_FLAT;
        else
            mode = DATA;
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_runConfigRunMode,mode);
}





void lima::Pixirad::Camera::setCoolingTemperatureSetpoint(float t){
	DEB_MEMBER_FUNCT();    
        if (t <=30 && t>=-50){  // TODO: get the real range that can be setup
            m_pixirad->m_envConfigCoolingTemperature = t;
	    
	    sendCommandForDetectorCooling();
	    
        }
        else THROW_HW_ERROR(Error) << "Cooling temperature Impossible. If you want to burn it, don't do it through software, go wild.";
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_envConfigCoolingTemperature,t);
}

void lima::Pixirad::Camera::getCoolingTemperatureSetpoint(float& t) {
	DEB_MEMBER_FUNCT();    
        t = m_pixirad->m_envConfigCoolingTemperature ;
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_envConfigCoolingTemperature,t);
}



      // Run Config Modes 

void lima::Pixirad::Camera::setCoolingMode(CoolingMode mode){
	DEB_MEMBER_FUNCT();
        switch(mode){
            case COOLING_OFF:
                m_pixirad->m_envConfigCoolingOnOff = 0 ;
                DEB_TRACE() << DEB_VAR2(m_pixirad->m_envConfigCoolingOnOff,mode);
                break;
            default:
                m_pixirad->m_envConfigCoolingOnOff = 1 ;
                DEB_TRACE() << DEB_VAR2(m_pixirad->m_envConfigCoolingOnOff,mode);
        }
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_envConfigCoolingOnOff,mode);
	
	sendCommandForDetectorCooling();
}


void lima::Pixirad::Camera::getCoolingMode(CoolingMode &mode){
	DEB_MEMBER_FUNCT();
        if(m_pixirad->m_envConfigCoolingOnOff == 0 )
            mode = COOLING_OFF;
        else
            mode = COOLING_ON;
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_envConfigCoolingOnOff,mode);
}




        // HighVoltage Biais



void lima::Pixirad::Camera::setHighVoltageBiais(float hv){
	DEB_MEMBER_FUNCT();    
        if (hv <=m_pixirad->m_envConfigHighVoltageBiaisMax && hv>=0){  // TODO: get the real range that can be setup
	  m_pixirad->m_envConfigHighVoltageBiais = hv;
	  
	  sendCommandForDetectorCooling();
        }
        else THROW_HW_ERROR(Error) << "High Voltage Value Impossible" <<DEB_VAR2(m_pixirad->m_envConfigHighVoltageBiaisMax, hv);
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_envConfigHighVoltageBiais,hv);
}

void lima::Pixirad::Camera::getHighVoltageBiais(float& hv) {
	DEB_MEMBER_FUNCT();    
        hv = m_pixirad->m_envConfigHighVoltageBiais ;
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_envConfigHighVoltageBiais,hv);
}





        // HighVoltage Mode on off

void lima::Pixirad::Camera::setHVBiasModePower(HVBiaisPower mode){
	DEB_MEMBER_FUNCT();
        switch(mode){
            case HV_OFF:
                m_pixirad->m_envConfigHighVoltageBiaisOnOff = 0 ;
                DEB_TRACE() << DEB_VAR2(m_pixirad->m_envConfigHighVoltageBiaisOnOff,mode);
                break;
            default:
                m_pixirad->m_envConfigHighVoltageBiaisOnOff = 1 ;
                DEB_TRACE() << DEB_VAR2(m_pixirad->m_envConfigHighVoltageBiaisOnOff,mode);
        }
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_envConfigHighVoltageBiaisOnOff,mode);
	
	sendCommandForDetectorCooling();
}



void lima::Pixirad::Camera::getHVBiasModePower(HVBiaisPower &mode){
	DEB_MEMBER_FUNCT();
        if(m_pixirad->m_envConfigHighVoltageBiaisOnOff == 0 )
            mode = HV_OFF;
        else
            mode = HV_ON;
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_envConfigHighVoltageBiaisOnOff,mode);
}








           // HighVoltage Mode
           
void lima::Pixirad::Camera::setHVBiasMode(HVMode mode){
	DEB_MEMBER_FUNCT();
        switch(mode){
            case AUTOHV_LC:
                m_pixirad->m_biaisMode = "AUTOHV_LC" ;
                DEB_TRACE() << DEB_VAR2(m_pixirad->m_biaisMode,mode);
                break;
            case STDHV:
                m_pixirad->m_biaisMode = "STDHV" ;
                DEB_TRACE() << DEB_VAR2(m_pixirad->m_biaisMode,mode);
                break;
            default:
                m_pixirad->m_biaisMode = "AUTOHV" ;
                DEB_TRACE() << DEB_VAR2(m_pixirad->m_biaisMode,mode);
        }
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_biaisMode,mode);
}


void lima::Pixirad::Camera::getHVBiasMode(HVMode &mode){
	DEB_MEMBER_FUNCT();
        if(m_pixirad->m_biaisMode.compare("STDHV") == 0 )
            mode = STDHV;
        else if(m_pixirad->m_biaisMode.compare("AUTOHV_LC") == 0 )
            mode = AUTOHV_LC;
        else
            mode = AUTOHV;
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_biaisMode,mode);
}





        // High voltage delays Before 

void lima::Pixirad::Camera::setHighVoltageDelayBeforeOn(float sec){
	DEB_MEMBER_FUNCT();    
        if (sec>=0){  // TODO: get the real range that can be setup
            m_pixirad->m_toffHvAcquisitionDelay = sec;
        }
        else THROW_HW_ERROR(Error) << "Delay Value Impossible. Seriously ? a negative number for a delay ? If you wish to go back in the past talk to a scifi movie fan.";
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_toffHvAcquisitionDelay,sec);
}

void lima::Pixirad::Camera::getHighVoltageDelayBeforeOn(float& sec) {
	DEB_MEMBER_FUNCT();    
        sec = m_pixirad->m_toffHvAcquisitionDelay ;
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_toffHvAcquisitionDelay,sec);
}





        // High voltage Reset after how many image 

void lima::Pixirad::Camera::setHVRefreshPeriod(int nbOfImages){
	DEB_MEMBER_FUNCT();    
        if (nbOfImages>=0){  // TODO: get the real range that can be setup
            m_pixirad->m_HvRefreshPeriod = nbOfImages;
        }
        else THROW_HW_ERROR(Error) << "Comme on, a negative number of frames ? seriously ? ";
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_HvRefreshPeriod,nbOfImages);
}

void lima::Pixirad::Camera::getHVRefreshPeriod(int& nbOfImages){
	DEB_MEMBER_FUNCT();    
        nbOfImages = m_pixirad->m_HvRefreshPeriod ;
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_HvRefreshPeriod,nbOfImages);
}





        // Delay between frames during a serie in ms

void lima::Pixirad::Camera::setDelayBetweenFrames(int delayms){
	DEB_MEMBER_FUNCT();    
        if (delayms>=0){  // TODO: get the real range that can be setup
            m_pixirad->m_pauseBetweenAcq = delayms;
        }
        else THROW_HW_ERROR(Error) << "Comme on, a negative delay ? Do you really want to take the next frame before you finished the previous one? ";
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_pauseBetweenAcq,delayms);
}

void lima::Pixirad::Camera::getDelayBetweenFrames(int& delayms){
	DEB_MEMBER_FUNCT();    
        delayms = m_pixirad->m_pauseBetweenAcq ;
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_pauseBetweenAcq,delayms);
}



//     int m_oneChipModeOutOfEight = 0;      
//    bool m_seedModeForDebugOnlyInOneChipWithPX8 = false;

void lima::Pixirad::Camera::setWhichModuleOutOfEightOnPX8(int module){
	DEB_MEMBER_FUNCT();    
        if (module>=0 and module < 8){  // 01234567  are real modules, else => -1 meaning
	    DEB_TRACE() << "Configuration for a PX8 detector with only 1 module. Chosen module is :" << DEB_VAR1(module);
            m_pixirad->m_oneChipModeOutOfEight = module;
        }
        else {
// 	  8 modules 
	    DEB_TRACE() << "Configuration for a PX8 detector with 8 modules. " ;
            m_pixirad->m_oneChipModeOutOfEight = -1;	  
	}
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_oneChipModeOutOfEight,module);
	
}
void lima::Pixirad::Camera::getWhichModuleOutOfEightOnPX8(int& module){
	DEB_MEMBER_FUNCT();    
        module = m_pixirad->m_oneChipModeOutOfEight ;
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_oneChipModeOutOfEight,module);
}


void lima::Pixirad::Camera::setSeedModeForDebugOnlyInOneChipWithPX8(bool saintRita){
	DEB_MEMBER_FUNCT();   
	
        m_pixirad->m_seedModeForDebugOnlyInOneChipWithPX8 = saintRita;
        if (saintRita){  
	    DEB_TRACE() << "You have my sincere sympathy for debugging the image reconstruction." << DEB_VAR1(saintRita);
        }
	
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_seedModeForDebugOnlyInOneChipWithPX8,saintRita);
}
void lima::Pixirad::Camera::getSeedModeForDebugOnlyInOneChipWithPX8(bool& saintRita){
	DEB_MEMBER_FUNCT();    
        saintRita = m_pixirad->m_seedModeForDebugOnlyInOneChipWithPX8 ;
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_seedModeForDebugOnlyInOneChipWithPX8,saintRita);
}







        // Color mode for the detector
           
void lima::Pixirad::Camera::setColorMode(ColorMode color){
	DEB_MEMBER_FUNCT();
        switch(color){
            case COLMODE_2COL:
                m_pixirad->m_RunModeColors = "2COL" ;
                DEB_TRACE() << DEB_VAR2(m_pixirad->m_RunModeColors,color);
                break;
            case COLMODE_1COL1:
                m_pixirad->m_RunModeColors = "1COL1" ;
                DEB_TRACE() << DEB_VAR2(m_pixirad->m_RunModeColors,color);
                break;
            case COLMODE_DTF:
                m_pixirad->m_RunModeColors = "DTF" ;
                DEB_TRACE() << DEB_VAR2(m_pixirad->m_RunModeColors,color);
                break;
            case COLMODE_4COL:
                m_pixirad->m_RunModeColors = "4COL" ;
                DEB_TRACE() << DEB_VAR2(m_pixirad->m_RunModeColors,color);
                break;
            default:
                m_pixirad->m_RunModeColors = "1COL0" ;
                DEB_TRACE() << DEB_VAR2(m_pixirad->m_RunModeColors,color);
        }
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_RunModeColors,color);
}


void lima::Pixirad::Camera::getColorMode(ColorMode &color){
	DEB_MEMBER_FUNCT();
        if(m_pixirad->m_RunModeColors.compare("2COL") == 0 )
            color = COLMODE_2COL;
        else if(m_pixirad->m_RunModeColors.compare("1COL1") == 0 )
            color = COLMODE_1COL1;
        else if(m_pixirad->m_RunModeColors.compare("DTF") == 0 )
            color = COLMODE_DTF;
        else if(m_pixirad->m_RunModeColors.compare("4COL") == 0 )
            color = COLMODE_4COL;
        else
            color = COLMODE_1COL0;
        DEB_TRACE() << DEB_VAR2(m_pixirad->m_RunModeColors,color);
}





void lima::Pixirad::Camera::setSyncOutFunction(SyncOutFunction mode){
  
  DEB_MEMBER_FUNCT();
  
  switch(mode){
    case SHUTTER:
      m_pixirad->m_syncOutFunction = "SHUTTER" ;
      DEB_TRACE() << DEB_VAR2(m_pixirad->m_syncOutFunction,mode);
      break;
    case RODONE:
      m_pixirad->m_syncOutFunction = "RODONE" ;
      DEB_TRACE() << DEB_VAR2(m_pixirad->m_syncOutFunction,mode);
      break;
    case READ:
      m_pixirad->m_syncOutFunction = "READ" ;
      DEB_TRACE() << DEB_VAR2(m_pixirad->m_syncOutFunction,mode);
      break;
    default:
      m_pixirad->m_syncOutFunction = "SHUTTER" ;
      DEB_TRACE() << DEB_VAR2(m_pixirad->m_syncOutFunction,mode);
  }  

  
}


void lima::Pixirad::Camera::getSyncOutFunction(SyncOutFunction &mode){
  
  DEB_MEMBER_FUNCT();
  if(m_pixirad->m_syncOutFunction.compare("SHUTTER") == 0 )
    mode = SHUTTER;
  else if(m_pixirad->m_syncOutFunction.compare("RODONE") == 0 )
    mode = RODONE;
  else if(m_pixirad->m_syncOutFunction.compare("READ") == 0 )
    mode = READ;
  else
    mode = SHUTTER;
  DEB_TRACE() << DEB_VAR2(m_pixirad->m_syncOutFunction,mode);
  
  
}


void lima::Pixirad::Camera::setSyncOutPol(Polarity mode){
  
  DEB_MEMBER_FUNCT();
  
  switch(mode){
    case POS:
      m_pixirad->m_syncOutPolarity = "POS" ;
      DEB_TRACE() << DEB_VAR2(m_pixirad->m_syncOutPolarity,mode);
      break;
    case NEG:
      m_pixirad->m_syncOutPolarity = "NEG" ;
      DEB_TRACE() << DEB_VAR2(m_pixirad->m_syncOutPolarity,mode);
      break;
    default:
      m_pixirad->m_syncOutPolarity = "POS" ;
      DEB_TRACE() << DEB_VAR2(m_pixirad->m_syncOutPolarity,mode);
  }  
  
  
}
void lima::Pixirad::Camera::getSyncOutPol(Polarity &mode){
  
  DEB_MEMBER_FUNCT();
  if(m_pixirad->m_syncOutPolarity.compare("POS") == 0 )
    mode = POS;
  else if(m_pixirad->m_syncOutPolarity.compare("NEG") == 0 )
    mode = NEG;
  else
    mode = POS;
  DEB_TRACE() << DEB_VAR2(m_pixirad->m_syncOutPolarity,mode);
  
  
}

void lima::Pixirad::Camera::setSyncInPol(Polarity mode){
  
  DEB_MEMBER_FUNCT();
  
  switch(mode){
    case POS:
      m_pixirad->m_syncInPolarity = "POS" ;
      DEB_TRACE() << DEB_VAR2(m_pixirad->m_syncInPolarity,mode);
      break;
    case NEG:
      m_pixirad->m_syncInPolarity = "NEG" ;
      DEB_TRACE() << DEB_VAR2(m_pixirad->m_syncInPolarity,mode);
      break;
    default:
      m_pixirad->m_syncInPolarity = "POS" ;
      DEB_TRACE() << DEB_VAR2(m_pixirad->m_syncInPolarity,mode);
  }  
  
}


void lima::Pixirad::Camera::getSyncInPol(Polarity &mode){
  
  DEB_MEMBER_FUNCT();
  if(m_pixirad->m_syncInPolarity.compare("POS") == 0 )
    mode = POS;
  else if(m_pixirad->m_syncInPolarity.compare("NEG") == 0 )
    mode = NEG;
  else
    mode = POS;
  DEB_TRACE() << DEB_VAR2(m_pixirad->m_syncInPolarity,mode);
  
}










void lima::Pixirad::Camera::setTrsfMode (TrsfMode mode){
  
  DEB_MEMBER_FUNCT();
  
  switch(mode){
    case UNMOD:
      m_pixirad->m_moderationMode = "UNMOD" ;
      DEB_TRACE() << DEB_VAR2(m_pixirad->m_moderationMode,mode);
      break;
    case MOD:
      m_pixirad->m_moderationMode = "MOD" ;
      DEB_TRACE() << DEB_VAR2(m_pixirad->m_moderationMode,mode);
      break;
    default:
      m_pixirad->m_moderationMode = "UNMODH" ;
      DEB_TRACE() << DEB_VAR2(m_pixirad->m_moderationMode,mode);
  }  
  
}


void lima::Pixirad::Camera::getTrsfMode(TrsfMode &mode){
  
  DEB_MEMBER_FUNCT();
  if(m_pixirad->m_moderationMode.compare("MOD") == 0 )
    mode = MOD;
  else if(m_pixirad->m_moderationMode.compare("UNMOD") == 0 )
    mode = UNMOD;
  else
    mode = UNMODH;
  DEB_TRACE() << DEB_VAR2(m_pixirad->m_moderationMode,mode);
  
}










void lima::Pixirad::Camera::getTemperaturePeltierCold(float & information){
  DEB_MEMBER_FUNCT();   
  
  AutoMutex lock(m_pixirad->m_cond_regexExtract.mutex());
  
  information = m_pixirad->m_temperaturePeltierCold;
  DEB_TRACE() << DEB_VAR2(m_pixirad->m_temperaturePeltierCold, information);
  
}






void lima::Pixirad::Camera::getTemperaturePeltierHot(float& information){
  DEB_MEMBER_FUNCT();        
  AutoMutex lock(m_pixirad->m_cond_regexExtract.mutex());
  information = m_pixirad->m_temperaturePeltierHot;
  DEB_TRACE() << DEB_VAR2(m_pixirad->m_temperaturePeltierHot, information);
  
}
void lima::Pixirad::Camera::getHighVoltageTension(float& information){
  DEB_MEMBER_FUNCT();        
  AutoMutex lock(m_pixirad->m_cond_regexExtract.mutex());
  information = m_pixirad->m_HighVoltageTension;
  DEB_TRACE() << DEB_VAR2(m_pixirad->m_HighVoltageTension, information);
  
}
void lima::Pixirad::Camera::getBoxHumidity(float& information){
  DEB_MEMBER_FUNCT();        
  AutoMutex lock(m_pixirad->m_cond_regexExtract.mutex());
  information = m_pixirad->m_boxHumidity;
  DEB_TRACE() << DEB_VAR2(m_pixirad->m_boxHumidity, information);
  
}
void lima::Pixirad::Camera::getBoxTemperature(float& information){
  DEB_MEMBER_FUNCT();        
  AutoMutex lock(m_pixirad->m_cond_regexExtract.mutex());
  information = m_pixirad->m_boxTemperature;
  DEB_TRACE() << DEB_VAR2(m_pixirad->m_boxTemperature, information);
  
}
void lima::Pixirad::Camera::getPeltierPower(float& information){
  DEB_MEMBER_FUNCT();        
  AutoMutex lock(m_pixirad->m_cond_regexExtract.mutex());
  information = m_pixirad->m_peltierPower;
  DEB_TRACE() << DEB_VAR2(m_pixirad->m_peltierPower, information);
  
}

void lima::Pixirad::Camera::getAlarmTempTooHot(bool& information){
  DEB_MEMBER_FUNCT();        
  AutoMutex lock(m_pixirad->m_cond_regexExtract.mutex());
  information = m_pixirad->m_alarmTempTooHot;
  DEB_TRACE() << DEB_VAR2(m_pixirad->m_alarmTempTooHot, information);
  
}
void lima::Pixirad::Camera::getAlarmTempTooHotEnabled(bool& information){
  DEB_MEMBER_FUNCT();        
  AutoMutex lock(m_pixirad->m_cond_regexExtract.mutex());
  information = m_pixirad->m_alarmTempTooHotEnabled;
  DEB_TRACE() << DEB_VAR2(m_pixirad->m_alarmTempTooHotEnabled, information);
  
}
void lima::Pixirad::Camera::getAlarmTempTooCold(bool& information){
  DEB_MEMBER_FUNCT();        
  AutoMutex lock(m_pixirad->m_cond_regexExtract.mutex());
  information = m_pixirad->m_alarmTempTooCold;
  DEB_TRACE() << DEB_VAR2(m_pixirad->m_alarmTempTooCold, information);
  
}
void lima::Pixirad::Camera::getAlarmTempTooColdEnabled(bool& information){
  DEB_MEMBER_FUNCT();        
  AutoMutex lock(m_pixirad->m_cond_regexExtract.mutex());
  information = m_pixirad->m_alarmTempTooColdEnabled;
  DEB_TRACE() << DEB_VAR2(m_pixirad->m_alarmTempTooColdEnabled, information);
  
}
void lima::Pixirad::Camera::getAlarmHumidity(bool& information){
  DEB_MEMBER_FUNCT();        
  AutoMutex lock(m_pixirad->m_cond_regexExtract.mutex());
  information = m_pixirad->m_alarmHumidity;
  DEB_TRACE() << DEB_VAR2(m_pixirad->m_alarmHumidity, information);
  
}
void lima::Pixirad::Camera::getAlarmHumidityEnabled(bool& information){
  DEB_MEMBER_FUNCT();        
  AutoMutex lock(m_pixirad->m_cond_regexExtract.mutex());
  information = m_pixirad->m_alarmHumidityEnabled;
  DEB_TRACE() << DEB_VAR2(m_pixirad->m_alarmHumidityEnabled, information);
  
}



void  lima::Pixirad::Camera::printMissingImageInfo(){
  DEB_MEMBER_FUNCT();        
  m_pixirad->printMissingImageInfo();  
}






























