/* 
 * File:   pixiradDetector.cpp
 * Author: watier
 * 
 * Created on May 27, 2015, 11:02 AM
 */
#include <iostream>
#include <string>
#include <cstring>
#include "pixiradDetector.h"
// #include "../../simulator/sip/SimulatorFrameBuilder.sip"

#include "pixiradHelperFunctions.h"

#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <arpa/inet.h>


#include <sys/types.h>

#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>

// #include "lima/ThreadUtils.h"
#include "lima/Exceptions.h"
#include "lima/Debug.h"


#include <unistd.h> // sleep

// #include <regex>  // weather
#include <regex.h> // Command acknowledgment
#include <cstdlib>
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION 



// to be removed:
#include <fstream>

#include "pixiradFpgaTools.h"




using namespace std;
using namespace lima;
using namespace lima::Pixirad;



pixiradDetector::pixiradDetector(std::string ipAdressDetector, int TcpPort, SoftBufferCtrlObj& buffer): m_bufferCtrlObj(buffer), m_ipAdressDetector(ipAdressDetector), m_TcpPort(TcpPort) {
  
  DEB_CONSTRUCTOR();
  DEB_TRACE() << "Starting threads for socket handling";
  DEB_TRACE() << "TCP Server for the pixirad is " << DEB_VAR2(ipAdressDetector, TcpPort);
  
  
  setStatusDetector(HwInterface::StatusType::Config);
  
  
  // TODO: COntinue testing regexp on this thread :
  m_boxHumidityTempMonitor =  std::thread(&pixiradDetector::boxHumidityTempMonitor, this);
  
//   setStatusDetector(HwInterface::StatusType::Ready);
  
  
  // Create once and for all the conversion table for the reconstruction task.
    m_conversion_table=(unsigned short*)calloc(32768, sizeof(unsigned short));    
    memset(m_conversion_table, 0, 32768*sizeof(unsigned short));    
    genera_tabella_clock(m_conversion_table, 32768, 15);
    
     
   
     char *sourceAsChar = reinterpret_cast<char*>(m_conversion_table); 
     std::ofstream b_stream("/tmp/m_conversion_table.bin", std::fstream::out | std::fstream::binary);
     b_stream.write(sourceAsChar, 32768*sizeof(unsigned short));
     b_stream.close();
     
  
    
}


pixiradDetector::~pixiradDetector(){
  DEB_DESTRUCTOR();
  stopAcq();
}


void pixiradDetector::boxHumidityTempMonitor(){
  
  DEB_MEMBER_FUNCT();
  
  int localSocket = 0;  
  struct sockaddr_in serv_addr;
  
  while(1){
  
  DEB_TRACE() << "Weather monitor (re) start for the box. Receive humidity, temperatures and electrical parameters via a dedicated port through UDP." ;
  
//   2224
  
  int broadcast=1;
  
  setsockopt(localSocket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof broadcast);
  
  
  
  
  if ( (localSocket=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1){
    DEB_TRACE() << "Error : Could not create a socket ";
  }
  
  memset(&serv_addr, '0', sizeof (serv_addr));
  
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(m_weatherUdpPort);
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  
  bind(localSocket, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
  
  
  
    
      DEB_TRACE() << "Weather monitor communication ready." << DEB_VAR1(m_weatherUdpPort);
      
      int bytes_recvd=0;
      char weather[1024];
	
      regex_t regex;  // pixi8 or all
      regex_t regexpixi1style; // pixi1 only
	
      
      while (1){
	bytes_recvd=0;
	bytes_recvd = recvfrom(localSocket, (char*)weather, 1024,  0, NULL, 0);
// 	 	DEB_TRACE() << "Weather thread: "<<DEB_VAR2(bytes_recvd, weather);
	
	
	// Here we find the interesting values within the udp "weather" stream of the detector. 
	// Thing changed between pixirad1 and 8 so, for some, there is two regexp slightly different.
	
	// Note for debug: if there is no update of the values, check that the binding is not already in use, like for example another instance of this detector running.
	
	
	
	if (bytes_recvd > 0){
	
// 	  DEB_TRACE()<<weather;
	  
// 	READ_TCOLD_DONE 22.80 �C
	regcomp(&regex, "READ_TCOLD_DONE ([-0123456789.]+)",  REG_EXTENDED);
	regcomp(&regexpixi1style, "READ_TCOLD ([-0123456789.]+)",  REG_EXTENDED);
	
	if ( lima::Pixirad::findThisValueIn(regex , std::string(weather), m_temperaturePeltierCold) 
	  or lima::Pixirad::findThisValueIn(regexpixi1style , std::string(weather), m_temperaturePeltierCold) 
	){		
//  	  DEB_TRACE() << "Cold side of the peltier : "<<DEB_VAR1(m_temperaturePeltierCold);
 	}
 	else DEB_TRACE() << "NO PELTIER COLD INFO IN WEATHER STREAM " << DEB_VAR1(weather);
	
	
	regfree(&regex);
	regfree(&regexpixi1style);
	
	
	// // 	READ_THOT_DONE 15.95 �C
	regcomp(&regex, "READ_THOT_DONE ([-0123456789.]+)",  REG_EXTENDED);
	regcomp(&regexpixi1style, "READ_THOT ([-0123456789.]+)",  REG_EXTENDED);
	
	if ( lima::Pixirad::findThisValueIn(regex , std::string(weather), m_temperaturePeltierHot) 
	  or lima::Pixirad::findThisValueIn(regexpixi1style , std::string(weather), m_temperaturePeltierHot) 
	){		
// 	  DEB_TRACE() << "Hot side of the peltier : "<<DEB_VAR1(m_temperaturePeltierHot);
	}
	else DEB_TRACE() << "NO PELTIER HOT INFO IN WEATHER STREAM " << DEB_VAR1(weather);
	
	
	regfree(&regex);
	regfree(&regexpixi1style);
	
	
	
	// 	READ_HV_DONE 0.14 V
	regcomp(&regex, "READ_HV_DONE ([-0123456789.]+)",  REG_EXTENDED);
	regcomp(&regexpixi1style, "READ_HV ([-0123456789.]+)",  REG_EXTENDED);
	
	if ( lima::Pixirad::findThisValueIn(regex , std::string(weather), m_HighVoltageTension) 
	  or lima::Pixirad::findThisValueIn(regexpixi1style , std::string(weather), m_HighVoltageTension) 
	){	
// 	  DEB_TRACE() << "HV tension : "<<DEB_VAR1(m_HighVoltageTension);
	}
	else DEB_TRACE() << "NO HIGH VOLTAGE INFO IN WEATHER STREAM " << DEB_VAR1(weather);
	
	regfree(&regex);
	regfree(&regexpixi1style);
	
	
	
	// 	READ_BOX_RH 27.25 %
	regcomp(&regex, "READ_BOX_RH ([-0123456789.]+)",  REG_EXTENDED);
	regcomp(&regexpixi1style, "READ_BOX_HUM ([-0123456789.]+)",  REG_EXTENDED);
	if ( lima::Pixirad::findThisValueIn(regex , std::string(weather), m_boxHumidity) 
	  or lima::Pixirad::findThisValueIn(regexpixi1style , std::string(weather), m_boxHumidity) 
	){		
// 	  DEB_TRACE() << "Humidity : "<<DEB_VAR1(m_boxHumidity);
	}
	else DEB_TRACE() << "NO HUMIDITY INFO IN WEATHER STREAM " << DEB_VAR1(weather);
	
	regfree(&regex);
	regfree(&regexpixi1style);
	
	
// 	READ_BOX_TEMP_DONE 19.70 �C  Pixirad 8
	
	regcomp(&regex, "READ_BOX_TEMP_DONE ([-0123456789.]+)",  REG_EXTENDED);
	regcomp(&regexpixi1style, "READ_BOX_TEMP ([-0123456789.]+)",  REG_EXTENDED);
	if ( lima::Pixirad::findThisValueIn(regex , std::string(weather), m_boxTemperature) 
	  or lima::Pixirad::findThisValueIn(regexpixi1style , std::string(weather), m_boxTemperature) 
	){	
// 	  DEB_TRACE() << "Box temperature : "<<DEB_VAR1(m_boxTemperature);
	}
	else DEB_TRACE() << "NO HIGH VOLTAGE INFO IN WEATHER STREAM " << DEB_VAR1(weather);
	
	
	regfree(&regex);
	regfree(&regexpixi1style);
	
	
// 	READ_PELTIER_PWR 0.00 %
	
	regcomp(&regex, "READ_PELTIER_PWR ([-0123456789.]+)",  REG_EXTENDED);
	if (lima::Pixirad::findThisValueIn(regex , std::string(weather), m_peltierPower)){	
// 	  DEB_TRACE() << "Peltier Power : "<<DEB_VAR1(m_peltierPower);
	}
	else DEB_TRACE() << "NO PELTIER POWER INFO IN WEATHER STREAM " << DEB_VAR1(weather);
	
	regfree(&regex);
	
	// Alarms can be disabled, off, or on.
	// Good practice is to check if enabled before looking for an alarm status.
	// alarms can be activated / deactivated / reinitialised with proper commands
	
	// Alarm for HOT PELTIER
	regcomp(&regex, ".*THOT_ALARM_STATUS OFF.*" , 0);
	if ( regexec(&regex, weather, 0, NULL, 0) == 0 ){
	  m_alarmTempTooHot = false;
	  m_alarmTempTooHotEnabled = true;	  
// 	  DEB_TRACE() << "Alarm : "<<DEB_VAR2(m_alarmTempTooHot, m_alarmTempTooHotEnabled);
	}	
	regfree(&regex);
	regcomp(&regex, ".*THOT_ALARM_STATUS ON.*" , 0);
	if ( regexec(&regex, weather, 0, NULL, 0) == 0 ){
	  m_alarmTempTooHot = true;
	  m_alarmTempTooHotEnabled = true;
// 	  DEB_TRACE() << "Alarm : "<<DEB_VAR2(m_alarmTempTooHot, m_alarmTempTooHotEnabled);
	}
	regfree(&regex);
	regcomp(&regex, ".*THOT_ALARM_STATUS DISABLED.*" , 0);
	if ( regexec(&regex, weather, 0, NULL, 0) == 0 ){
	  m_alarmTempTooHot = false;
	  m_alarmTempTooHotEnabled = false;
// 	  DEB_TRACE() << "Alarm : "<<DEB_VAR2(m_alarmTempTooHot, m_alarmTempTooHotEnabled);
	}
	regfree(&regex);
	
	
	
	// Alarm for COLD PELTIER
	regcomp(&regex, ".*TCOLD_ALARM_STATUS OFF.*" , 0);
	if ( regexec(&regex, weather, 0, NULL, 0) == 0 ){
	  m_alarmTempTooCold = false;
	  m_alarmTempTooColdEnabled= true;
// 	  DEB_TRACE() << "Alarm : "<<DEB_VAR2(m_alarmTempTooCold, m_alarmTempTooColdEnabled);
	}	
	regfree(&regex);
	
	
	regcomp(&regex, ".*TCOLD_ALARM_STATUS ON.*" , 0);
	if ( regexec(&regex, weather, 0, NULL, 0) == 0 ){
	  m_alarmTempTooCold = true;
	  m_alarmTempTooColdEnabled= true;
// 	  DEB_TRACE() << "Alarm : "<<DEB_VAR2(m_alarmTempTooCold, m_alarmTempTooColdEnabled);
	}	
	regfree(&regex);
	
	
	regcomp(&regex, ".*TCOLD_ALARM_STATUS DISABLED.*" , 0);
	if ( regexec(&regex, weather, 0, NULL, 0) == 0 ){
	  m_alarmTempTooCold = false;
	  m_alarmTempTooColdEnabled= false;
// 	  DEB_TRACE() << "Alarm : "<<DEB_VAR2(m_alarmTempTooCold, m_alarmTempTooColdEnabled);
	}
	regfree(&regex);
	
	
	
	// Alarm for HUMIDITY
	regcomp(&regex, ".*HUMIDITY_ALARM_STATUS OFF.*" , 0);
	if ( regexec(&regex, weather, 0, NULL, 0) == 0 ){
	  m_alarmHumidity = false;
	  m_alarmHumidityEnabled= true;
// 	  DEB_TRACE() << "Alarm : "<<DEB_VAR2(m_alarmHumidity, m_alarmHumidityEnabled);
	}	
	
	regfree(&regex);
	
	
	regcomp(&regex, ".*HUMIDITY_ALARM_STATUS ON.*" , 0);
	if ( regexec(&regex, weather, 0, NULL, 0) == 0 ){
	  m_alarmHumidity = true;
	  m_alarmHumidityEnabled= true;
// 	  DEB_TRACE() << "Alarm : "<<DEB_VAR2(m_alarmHumidity, m_alarmHumidityEnabled);
	}
	regfree(&regex);
	
	regcomp(&regex, ".*HUMIDITY_ALARM_STATUS DISABLED.*" , 0);
	if ( regexec(&regex, weather, 0, NULL, 0) == 0 ){
	  m_alarmHumidity = false;
	  m_alarmHumidityEnabled= false;
// 	  DEB_TRACE() << "Alarm : "<<DEB_VAR2(m_alarmHumidity, m_alarmHumidityEnabled);
	}
	regfree(&regex);
	
	// There is no alarm feedback for the PX1. Humidity is nice too know.
	if(m_sensorConfigBuild == "PX1" and m_boxHumidity >= 2){
	  // Forcing alarm if humidity more than 2%
	  m_alarmHumidity = true;	  
	}
	
	if(m_sensorConfigBuild == "PX1" and m_boxHumidity < 2){
	  // Forcing alarm if humidity more than 2%
	  m_alarmHumidity = false;	  
	}
	
	if(m_sensorConfigBuild == "PX1" and m_boxTemperature >= 30){
	  // Forcing alarm if humidity more than 2%
	  m_alarmTempTooHot = true;	 
	}
	
	if(m_sensorConfigBuild == "PX1" and m_boxTemperature <= 8){
	  // Forcing alarm if humidity more than 2%
	  m_alarmTempTooCold = true;	  
	}
	
	if(m_sensorConfigBuild == "PX1" and m_boxTemperature > 8 and m_boxTemperature < 30){
	  // Forcing alarm if humidity more than 2%
	  m_alarmTempTooCold = false;	  
	  m_alarmTempTooHot = false;	  
	}
	
	
	
	
	regcomp(&regex, ".*PIXIRAD-8 SN 8000.*" , 0);
	if ( regexec(&regex, weather, 0, NULL, 0) == 0 ){
	  if (m_sensorConfigBuild != "PX8"){ // Only first time
	    m_sensorConfigAsic = "PII";
	    m_sensorConfigHybrid = "CDTE"; // CDTE GAAS
	    m_UdpPortImages = 9999;
	    m_nbModules =8;
	    m_sensorConfigBuild = "PX8"; // PX1 PX2 PX8
	    
	    DEB_TRACE() << "A pixirad 8 model has been detected - AUTOCONFIGURATION based on UDP stream";
	  }
	}	
	regfree(&regex);
	
	regcomp(&regex, ".*BOX_SERIAL 1018.*" , 0);
	if ( regexec(&regex, weather, 0, NULL, 0) == 0 ){
	  if (m_sensorConfigBuild != "PX1"){
	    m_sensorConfigAsic = "PII";
	    m_sensorConfigHybrid = "CDTE"; // CDTE GAAS
	    m_sensorConfigBuild = "PX1"; // PX1 PX2 PX8	  
	    m_UdpPortImages = 2223;
	    m_nbModules = 1;
	    DEB_TRACE() << "A pixirad 1 model has been detected  - AUTOCONFIGURATION based on UDP stream";	  
	  }
	}
	regfree(&regex);
	
	
	}
	
	
      }
    
    
    /*
    * **PIXIRAD-8 SN 8000 ***
    READ_TCOLD_DONE 22.80 �C
    READ_THOT_DONE 15.95 �C
    READ_HV_DONE 0.14 V
    READ_BOX_RH 27.25 %
    READ_BOX_TEMP_DONE 19.70 �C
    READ_PELTIER_PWR 0.00 %
    ALARMS:
    THOT_ALARM_STATUS OFF
    TCOLD_ALARM_STATUS DISABLED
    HUMIDITY_ALARM_STATUS ON
   */ 

  
  // close the socket
  close(localSocket);
  // relase the cracken
  //   pthread_mutex_unlock(&m_mutex);
  
}

}













void pixiradDetector::getImagesInAThread()
{
  DEB_MEMBER_FUNCT();
  
  if(m_imageThread.joinable()){
    DEB_TRACE() << "A PREVIOUS IMAGE THREAD IS STILL ALIVE" << DEB_VAR1(m_imageThread.get_id());
    m_imageThread.detach();
    DEB_TRACE() << "Detached" << DEB_VAR1(m_imageThread.get_id());
  }
  
  DEB_TRACE() << "Creation of an independant thread for image reception." ;
  m_imageThread =  std::thread(&pixiradDetector::getImages, this);
  
  
}


void pixiradDetector::prepareAcq()
{
  DEB_MEMBER_FUNCT();
  
  // Prepare new buffer for reconstruction tasks
  // This may have changed due to acquisition option, so lets have it at each new set of images request.
  
  if(m_sensorConfigBuild.compare("PX1") == 0){ 
    m_nbModules = 1;
    m_UdpPortImages = 2223;
  }
  
  if(m_sensorConfigBuild.compare("PX8") == 0){
    if(m_oneChipModeOutOfEight != -1 ){
      m_nbModules = 1;
    }
    else {
      m_nbModules = 8;
    }
  
    
    m_UdpPortImages = 9999;
  }
  
  FrameDim      myFrameDim;  
  myFrameDim.setImageType(Bpp16);  
  
  Size mySize;
  
    DEB_TRACE() << "Size declaration :" << DEB_VAR1(mySize);
  
  mySize = Size(476, 512);
  
    DEB_TRACE() << "Size default :" << DEB_VAR1(mySize);
  
  getSize(mySize);
  
  
    DEB_TRACE() << "Size getted :" << DEB_VAR1(mySize);
  
    
    
    DEB_TRACE() << "frame initialise  :" << DEB_VAR1(mySize);
  myFrameDim.setSize(mySize);
  
    DEB_TRACE() << "Size getted :" << DEB_VAR1(mySize);
  
  
    
  m_reconstructionBufferCtrlObj = new SoftBufferCtrlObj(); //Lolo's m_temp_buffer_ctrl_obj
  m_reconstructionBufferCtrlObj->setFrameDim(myFrameDim);  
  m_reconstructionBufferCtrlObj->setNbBuffers(m_nbOfFrameInReconstructionBuffer);
    
  
  

}

void pixiradDetector::getSize(Size &size){

  DEB_MEMBER_FUNCT();
  
  if(m_sensorConfigBuild.compare("PX1") == 0){ 
      size = Size(476, 512);  // what it should be
//     size = Size(512, 476);

  }
  
  if(m_sensorConfigBuild.compare("PX8") == 0){
    if(m_oneChipModeOutOfEight != -1 ){
      size = Size(476, 512);
    }
    else {
      size = Size(476, 4096);
    }
  
  }
  
    DEB_TRACE() << "Size has been configured by detector class as :" << DEB_VAR1(size);
  
  
}




void pixiradDetector::getImages()
{
  DEB_MEMBER_FUNCT();
  
  
  
  m_allImagesReceived = false;
  
  
  // Buffer manager for the reconstruction (the one that gives the pointer)
  StdBufferCbMgr & reconstructionBufferMgr  = m_reconstructionBufferCtrlObj->getBuffer();
  
  
  //////////////////// REAL TIME  /////////////////////////////
  // Going to a real time fifo mode to be sure to not loose any udp packets.
  pthread_t this_thread = pthread_self(); 
  struct sched_param params;     

  params.sched_priority = 5; 
  
  DEB_TRACE() << "Trying to set thread realtime prio " << DEB_VAR1(params.sched_priority);
  // Attempt to set thread real-time priority to the SCHED_FIFO policy     
  int ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);     
  if (ret != 0) {         
    DEB_TRACE() << "Sadly I am Unsuccessful in setting a higher thread realtime priority. Add yourself to /etc/security/limits.conf , you deserve it" ;
  }
  
  
  
  
  std::unique_lock<std::mutex> uniqLock(m_mutexUDPImage);
  
  /////////////////////  SOCKET UDP /////////////////////////////////
  DEB_TRACE() << "UDP Socket Creation";
  struct sockaddr_in sockaddrInUDP;
  int socketUDPImage;
  
  
  
  //////////////////  Receiving //////////////
  if(m_sensorConfigBuild == "PX8" ){ m_UdpPortImages = 9999; }  
  else { m_UdpPortImages = 2223; }
  
  
  memset((unsigned char *) &sockaddrInUDP, 0, sizeof(sockaddrInUDP));
  
  
  unsigned char buf[BUFLEN];
  
  if ( (socketUDPImage=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    DEB_ERROR() << "UDP Socket Creation Failed";
    //TODO: Set Error to lima 
  }
  
  
  sockaddrInUDP.sin_family = AF_INET;
  sockaddrInUDP.sin_port = htons(m_UdpPortImages);
  
  
  bind(socketUDPImage, (struct sockaddr *) &sockaddrInUDP, sizeof(sockaddrInUDP));
  
  if (inet_aton("192.168.0.255" , &sockaddrInUDP.sin_addr) == 0) 
  {
    DEB_ERROR() << "inet_aton() failed" ;
  }
  
  
  
 
  
  DEB_TRACE() << "Modification of socket SO_RCVBUF size";
  int rcvBufferSize;
  socklen_t sockOptSize = sizeof(rcvBufferSize);
  getsockopt(socketUDPImage, SOL_SOCKET, SO_RCVBUF, &rcvBufferSize, &sockOptSize);
  DEB_TRACE() << "Modification of socket SO_RCVBUF size. Initial socket receive buf SO_RCVBUF size:" << DEB_VAR1(rcvBufferSize);
  
  
  //  net.core.rmem_max = 507217408 (pages of 4kbytes)
  // kernel will x2 this. 
  int kernelbuffersize =250000000 ;  // max for kernel must be twice.
  
  if (setsockopt(socketUDPImage, SOL_SOCKET, SO_RCVBUF, &kernelbuffersize, sockOptSize) == -1) {
    DEB_ERROR() << "UDP Socket buffer size SO_RCVBUF increase failed, change /etc/sysctl.conf if you experience problems.";
  }
   getsockopt(socketUDPImage, SOL_SOCKET, SO_RCVBUF, &rcvBufferSize, &sockOptSize);
  DEB_TRACE() << "Final socket receive buf SO_RCVBUF size:" << DEB_VAR1(rcvBufferSize);
  
  
  
  /*
  
  //////////////////  Receiving //////////////
  if(m_sensorConfigBuild == "PX8" and m_readOutSchema == "DEFAULT"){
    m_numberOfUDPPacketsPerImage = 2539;
//     m_numberOfUDPPacketsPerImage = PII_PX8_DEFAULT_NPACK;
  }  
  else {
//     m_numberOfUDPPacketsPerImage = PII_PX1_DEFAULT_NPACK;
    m_numberOfUDPPacketsPerImage = 360;
  }
  */
  
  
  unsigned short acknowledgator[m_nbFramesAcq];
  memset(acknowledgator, 0, m_nbFramesAcq*sizeof(unsigned short));
  m_acknowledgatorPointer = & acknowledgator[0];
  
  
  unsigned short   slotId=0;
  unsigned short   packet_id;
  unsigned short   packet_id_1;
  unsigned short   packet_id_2;
  unsigned short   packet_id_18;
  unsigned short   packetTag =0;
  
  
  StdBufferCbMgr& finalBufferMgr = m_bufferCtrlObj.getBuffer();
  
  m_reconstructionBufferCtrlObj = new SoftBufferCtrlObj();
  
  
  setStatusDetector(HwInterface::StatusType::Readout);
  
  m_stopAcquisition = false;
  
  
  // /////// The asic send more pixels than needed.  /////    
  // For PX1 :       
  // For the PX1 we will receive 360 * 1440 (useful) bytes
  // The data is 15 * 476 * 512
  // Therefore the 15 * 476 * 512 / 1440.0 / 8 =  317.3333333333333 , the foloowing 42 UDP datagrams needs not to be saved       
  // packetid larger than 318 are to be discarded
  // packetid 317 will have to be truncated to the first 480 bytes
  // ( 476*512*15 - 317*1440*8 )  /8  = 480
  
  // For PX8 :       
  // For the PX8 we will receive 2539 * 1440 (useful) bytes
  // The data  is 15 * 476 * 512 *8 =  29245440 bits
  // Therefore the 476*512*15*8/1440.0/8 = 2538.6666666666665 , 
  // the last datagram (packetid==2539) have to be truncated to the first 960 bytes
  //(476*512*15 *8 - 2538*1440*8)/8 = 960  

  int lastDatagramToKeep = 0;
  int amountOfTheLastDatagramToKeep = 0;
  if (m_sensorConfigBuild == "PX8" and m_nbModules == 8){
    lastDatagramToKeep = 2538;
    amountOfTheLastDatagramToKeep= 960;
    m_numberOfUDPPacketsPerImage = 2539; 
  }
  else{
    lastDatagramToKeep = 317;
    amountOfTheLastDatagramToKeep= 480;   
    m_numberOfUDPPacketsPerImage = 360; 
  }
  
  
  
  DEB_TRACE()<< "Waiting for UDP datagrams"<< DEB_VAR2((m_numberOfUDPPacketsPerImage-1)*m_nbFramesAcq, m_UdpPortImages ) ;
  for(int packet = 0 ; packet < (m_numberOfUDPPacketsPerImage)*m_nbFramesAcq; packet++  ){
    if(not m_stopAcquisition){
    
      int realpacketsize = recvfrom(socketUDPImage, (char*)buf, MAX_PACK_LEN,  0, NULL, 0);
//    recvfrom(socketUDPImage, (char*)buf, 4,  0, NULL, 0);
     
      if (realpacketsize != 1448){DEB_ERROR() << "A packet has an unexpected size, you should be worried about it." <<DEB_VAR1(realpacketsize);}
      if(realpacketsize == -1){DEB_ERROR() << "UDP error, some datagrams has been lost in the wild world of copper cables."<<DEB_VAR1(packet);} 
      else{
	
      packetTag=*buf;
      
      //TODO: Autocal
      if(packetTag & AUTOCAL_DATA){DEB_ERROR() << "AUTOCAL DATA received - Someone should do something about it.";}
      
      // SlotId is the image frame number in pixirad manual
      // packet id is the part of the image that is in the received udp datagram
      
      slotId=buf[1];
      packet_id_1=buf[2];
      packet_id_2=buf[3];
      
      packet_id_18=packet_id_1<<8;
      packet_id = packet_id_18 + packet_id_2;
      
      if (packet_id <= lastDatagramToKeep){
	
	bool iDontKnowMyPlace = true;
	bool fireLima = false;
	
	while (iDontKnowMyPlace) {
	  
	  if(acknowledgator[slotId] <= lastDatagramToKeep - 2){
	    // first round 
	    // fill image here
	    iDontKnowMyPlace = false; 
	    //printf("image %i packet %i \n", slotId, packet_id);	
	    
//     	  DEB_TRACE() << "recv -2: " <<DEB_VAR3(slotId, packet_id, acknowledgator[slotId]);
	    
	    acknowledgator[slotId] = acknowledgator[slotId] +1;
	    
	  }
	  else if(acknowledgator[slotId] == lastDatagramToKeep - 1 ){
	    
	    // fire lima !	
//     	  DEB_TRACE() << "recv -1: " <<DEB_VAR3(slotId, packet_id, acknowledgator[slotId]);
	    DEB_TRACE() << "Image is completely received " <<DEB_VAR1(slotId);	
	    
	    acknowledgator[slotId] = acknowledgator[slotId] +1;
	    iDontKnowMyPlace = false; 
	    fireLima = true;
	  }
	  if( (not fireLima) and acknowledgator[slotId] >= lastDatagramToKeep){
	    slotId = slotId + 256;
	    iDontKnowMyPlace = true; // Still true, could be more than 512
	  }
	}
	
	
	void *voidimageptr =reconstructionBufferMgr.getFrameBufferPtr(slotId);
	
	uint8_t *image8b = reinterpret_cast<uint8_t*>(voidimageptr);
	
	// image =16b  =>/2
  //       int positionOfDatagramInImage8b =  packet_id * 1440; //
	int positionOfDatagramInImage8b =  packet_id * 1440; // 
	
	
	
	
  //       DEB_TRACE()<<"MEMCOPY BEFORE >> "<<DEB_VAR4(image8b[positionOfDatagramInImage8b],positionOfDatagramInImage8b,buf[4], realpacketsize);
	//       memcpy(&image8b[positionOfDatagramInImage8b], &buf[4], realpacketsize-8);
	
	if (packet_id == lastDatagramToKeep){
	  memcpy(&image8b[positionOfDatagramInImage8b], buf+4, amountOfTheLastDatagramToKeep);
	}
	else {
	  memcpy(&image8b[positionOfDatagramInImage8b], buf+4, 1440);
	}
  //       DEB_TRACE()<<"MEMCOPY AFTER <<"<<DEB_VAR4(image8b[positionOfDatagramInImage8b],positionOfDatagramInImage8b,buf[4], realpacketsize);
	
	// TODO: do something for tha last datagram, which will have only a part of the buffer related to the image.
  
	
	if(fireLima)
	{   
	  // Build a frame info for Lima.
	  HwFrameInfoType frame_info;
	  frame_info.acq_frame_nb = (int)slotId;// First image is 0 for frame info
	  
	  DEB_TRACE() << DEB_VAR2(frame_info, slotId);
	  
	  bool result = finalBufferMgr.newFrameReady(frame_info); 
	  
	  DEB_ALWAYS() << "Image has been published in Lima through newFrameReady." << DEB_VAR4(result,  m_numberOfUDPPacketsPerImage*m_nbFramesAcq, packet, frame_info);
	  
	  fireLima = false;
	  
		// To be removed :
	  char *sourceAsChar4 = reinterpret_cast<char*>(image8b); 
	  std::ofstream b_stream4("/tmp/before_limabuf_0.bin", std::fstream::out | std::fstream::binary);
	  b_stream4.write(sourceAsChar4, 512*476*1*2);
	  b_stream4.close();
     
	  
	}
      }// discard too much datagrams
      
    } // if not stop acquisition
    }
  }

//   free(acknowledgator);
//   // Banzai
//   m_imageThread.detach();
m_allImagesReceived = true;

// free(buf);

close(socketUDPImage);

m_mutexUDPImage.unlock();

}




int pixiradDetector::sendCommand(std::string command, char commandAnswerFromDetector[MAX_MSG_STR_LENGTH], bool waitanswer){
  DEB_MEMBER_FUNCT();
  
//   pthread_mutex_lock(&m_mutex);
  
  // flushing answer buffer
  for(int i = 0 ; i<MAX_MSG_STR_LENGTH; i++){commandAnswerFromDetector[i]='\0';}
  
  
  std::unique_lock<std::mutex> uniqLock(m_mutexCommandTCP);
  
  int m_socketToPixiradServer = 0;
  
  
  int resultConnection = 0;
  int resultConnection2 = 0;
  struct sockaddr_in serv_addr;
  
//   DEB_TRACE() << "Attempting to create a AF_INET, SOCK_STREAM TCP socket for command sending ";
  if ((m_socketToPixiradServer = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
     DEB_ERROR() << "Error : Could not create a socket ";
  }
  
  memset(&serv_addr, '0', sizeof (serv_addr));
  
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(m_TcpPort);
  
  
//   DEB_TRACE() << "socket inet_pton ";
  if (inet_pton(AF_INET, m_ipAdressDetector.c_str(), &serv_addr.sin_addr) <= 0) {
    DEB_ERROR() << "inet_pton error occured";   
    close(m_socketToPixiradServer); 
    return -1;
  }
  
//   DEB_TRACE() << "Socket Connection" ;
  if (connect(m_socketToPixiradServer, (struct sockaddr *) &serv_addr, sizeof (serv_addr)) < 0) {
    DEB_ERROR() << "Error : Connect Failed ";
    close(m_socketToPixiradServer);
    return -2;
  }
  else {  
    
      
       DEB_TRACE() << "TX --- Sending : " << DEB_VAR1(command);
      uint32_t sizeOfCommand = command.length();
      
      // Then the message itself.
      command.append("\n");
      resultConnection = write(m_socketToPixiradServer, command.c_str(), sizeOfCommand+1); 
      
      
//       DEB_TRACE() << "TX --- Sent successfully the command: " << DEB_VAR1(resultConnection);
      
//       resultConnection2 = write(m_socketToPixiradServer, "\n", 2); 
      
//       DEB_TRACE() << "TX --- Sent successfully the cariage return: " << DEB_VAR1(resultConnection2);
      
      if (resultConnection < 0 or resultConnection2 < 0) {
// 	DEB_TRACE() << "TX --- ERROR writing to socket";
	close(m_socketToPixiradServer);
	return -3;
      }
      
      // Now we listen the answer from the detector
      if(waitanswer){
      
// 	DEB_TRACE() << "RX TCP --- Waiting for answer from the detector " ;
	
	int bytes_recvd=0;
	int index=0;
	do{
	  bytes_recvd=recv(m_socketToPixiradServer,(char*)(commandAnswerFromDetector+index),128,0);
	  index+=bytes_recvd;
	}
	while(commandAnswerFromDetector[index]!='\n' && bytes_recvd!=SOCKET_ERROR && bytes_recvd!=0 && index<MAX_MSG_STR_LENGTH);
 	DEB_TRACE() << "RX  TCP --- detector answer: "<<DEB_VAR1(commandAnswerFromDetector) ;
      }
    }
  
  
  // close the socket
  close(m_socketToPixiradServer);
  // release the cracken
//    pthread_mutex_unlock(&m_mutexCommandTCP);
  
  return resultConnection + resultConnection2;
 // return &commandAnswerFromDetector;
}





int pixiradDetector::update_thresolds_from_energies(){
  DEB_MEMBER_FUNCT();
  double th_req[4],th_act[4];
  int   th_int[4];
  int Vthmax,i;
  /**************/
  th_req[0]=(double)m_sensorConfigLowThreshold0;
  th_req[1]=(double)m_sensorConfigHighThreshold0;
  th_req[2]=(double)m_sensorConfigLowThreshold1;
  th_req[3]=(double)m_sensorConfigHighThreshold1;
  /**************/
  for(i=0;i<4;i++)
    if(th_req[i]<=0.0)
      return(-1);
    for(i=0;i<4;i++)
      if(th_req[i]>=MAXIMUM_REQ_EN_KEV)
	return(-2);
      /**************/
      PIXIEIIThresholdCalculator( VTHMAX_UPPER_LIMIT,
				  th_req,
				  &Vthmax,
				  th_int,
				  th_act);
      /*************/
      m_sensorConfigLowThreshold0	= (float)th_act[0];
    m_sensorConfigHighThreshold0	= (float)th_act[1];
    m_sensorConfigLowThreshold1	        = (float)th_act[2];
    m_sensorConfigHighThreshold1	= (float)th_act[3];
    /*************/
    m_sensorConfigLowThreshold0DAC	= th_int[0];
    m_sensorConfigHighThreshold0DAC 	= th_int[1];
    m_sensorConfigLowThreshold1DAC      = th_int[2];
    m_sensorConfigHighThreshold1DAC     = th_int[3];
    m_envConfigHighVoltageBiaisMax		= Vthmax;
    /*************/
    return(1);
    
}






void pixiradDetector::setStatusDetector(HwInterface::StatusType::Basic status){ 
  //         mutex
  DEB_MEMBER_FUNCT();
  std::unique_lock<std::mutex> uniqLock(m_mutexStatus);
  m_pixiradStatus = status;
  DEB_TRACE() << "Change of status " << DEB_VAR1(status);
}

// Or just read in interface ?
void pixiradDetector::getStatusDetector(HwInterface::StatusType::Basic & status){ 
  //         mutex
  DEB_MEMBER_FUNCT();
  
  char detectorStatusFromDetector[MAX_MSG_STR_LENGTH] = "";
  // Warning: the doc says SYS:? GET_ACQUISTION_STATUS , I prefer to send an extra I.
  sendCommand("SYS:? GET_ACQUISITION_STATUS", detectorStatusFromDetector, true);
  
  DEB_TRACE() << "Detector says : " << DEB_VAR1(detectorStatusFromDetector);
  
//   TODO:  CASES ON WHAT DETECTOR SAYS.
  
  std::unique_lock<std::mutex> uniqLock(m_mutexStatus);
  
  regex_t regex;
  
  regcomp(&regex, ".*ACQ STATUS: IDLE" , 0);
  if ( regexec(&regex, detectorStatusFromDetector, 0, NULL, 0) == 0 ){m_pixiradStatus = HwInterface::StatusType::Ready;}
  regfree(&regex);
  
  regcomp(&regex, ".*ACQ STATUS: STARTED" , 0);
  if ( regexec(&regex, detectorStatusFromDetector, 0, NULL, 0) == 0 ){m_pixiradStatus = HwInterface::StatusType::Exposure;}
  regfree(&regex);
  
  regcomp(&regex, ".*ACQ STATUS: RUNNING" , 0);
  if ( regexec(&regex, detectorStatusFromDetector, 0, NULL, 0) == 0 ){m_pixiradStatus = HwInterface::StatusType::Exposure;}
  regfree(&regex);
  
  regcomp(&regex, ".*ACQ STATUS: DONE" , 0);
  if ( regexec(&regex, detectorStatusFromDetector, 0, NULL, 0) == 0 ){m_pixiradStatus = HwInterface::StatusType::Ready;}
  regfree(&regex);
  
  // BROKEN Means that is has been stopped. so detector is ready.
  regcomp(&regex, ".*ACQ STATUS: BROKEN" , 0);
  if ( regexec(&regex, detectorStatusFromDetector, 0, NULL, 0) == 0 ){m_pixiradStatus = HwInterface::StatusType::Ready;}
  regfree(&regex);
  
  regcomp(&regex, ".*ACQ STATUS: BREAK_REQ" , 0);
  if ( regexec(&regex, detectorStatusFromDetector, 0, NULL, 0) == 0 ){m_pixiradStatus = HwInterface::StatusType::Ready;}
  regfree(&regex);
  
  regcomp(&regex, ".*ACQ STATUS: ERROR_GETTING_STATE" , 0);
  if ( regexec(&regex, detectorStatusFromDetector, 0, NULL, 0) == 0 ){m_pixiradStatus = HwInterface::StatusType::Fault;}
  regfree(&regex);
  
  regcomp(&regex, ".*ACQ STATUS: UNKNOWN" , 0);
  if ( regexec(&regex, detectorStatusFromDetector, 0, NULL, 0) == 0 ){m_pixiradStatus = HwInterface::StatusType::Fault;}  
  regfree(&regex);
  
  
  
  if(m_allImagesReceived == false){
    DEB_TRACE() << "All images not received, status forced to readout, no matter what the detector said. Use printMissingImageInfo() to get more infos";
    
    m_pixiradStatus = HwInterface::StatusType::Readout;
  }
  
  status = m_pixiradStatus;
  DEB_TRACE() << "Access to status " << DEB_VAR2(m_pixiradStatus, status);
  
}




void pixiradDetector::printMissingImageInfo(){  
  DEB_MEMBER_FUNCT();
  
  if(m_allImagesReceived == false){
    DEB_ALWAYS() << "EE Acquisition thread think that all images have not been received"  << DEB_VAR1(m_allImagesReceived)  ;
  }
  else{
    DEB_ALWAYS() << "OK Acquisition thread think that all images have been received" << DEB_VAR1(m_allImagesReceived)  ; 
  }
  
  DEB_TRACE() << "Searching for incomplete images.";
  
  int lastDatagramToKeep = 0;
  if (m_sensorConfigBuild == "PX8" and m_nbModules == 8){
    lastDatagramToKeep = 2538;
  }
  else{
    lastDatagramToKeep = 317;
  }
  
  
  for (int img = 0; img<m_nbFramesAcq; img++){
    
    if( m_acknowledgatorPointer[img] < lastDatagramToKeep ){
      
      DEB_ALWAYS() << "Incomplete Image: " << DEB_VAR3(img,lastDatagramToKeep, lastDatagramToKeep - m_acknowledgatorPointer[img]);
    }
  }
}


void pixiradDetector::stopAcq(){
  
  DEB_MEMBER_FUNCT();
  DEB_ALWAYS() << "ACQUISITION HAS BEEN FORCED TO STOP !!!!  Forcing a stop on the acquisition thread.";
  
  m_stopAcquisition = true;  // should get out of the acquisition loop
  m_allImagesReceived = true;
  if(m_imageThread.joinable()){
    DEB_TRACE() << "A PREVIOUS IMAGE THREAD IS STILL ALIVE ... " << DEB_VAR1(m_imageThread.get_id());
    m_imageThread.detach();
    DEB_TRACE() << "Well, was alive, I mean.  Now detached, a thread beheading of a sort." << DEB_VAR1(m_imageThread.get_id());
  }
  else {
    DEB_TRACE() << "No previous image thread pending. No problem then.";
  }
  
  
  
}



























