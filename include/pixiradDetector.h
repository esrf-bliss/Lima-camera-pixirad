/* 
 * File:   pixiradDetector.h
 * Author: watier
 *
 * Created on May 27, 2015, 11:02 AM
 */

#ifndef PIXIRADDETECTOR_H
#define	PIXIRADDETECTOR_H


#include <thread>
#include <mutex>
#include <condition_variable>

#include <vector>
//#include <string>
//#include <iostream>

#include "lima/HwInterface.h" // For Status

#include "lima/Debug.h"
#include "lima/HwBufferMgr.h"
/*
#include <regex>  // Command acknowledgment*/

using namespace std;

namespace lima {
namespace Pixirad {

class pixiradDetector {
    DEB_CLASS_NAMESPC(DebModCamera, "pixiradDetector", "Pixirad");

public:
  
  pixiradDetector(std::string ipAdressDetectorServer, int TcpPort,SoftBufferCtrlObj& buffer);
  ~pixiradDetector();
  
  #define MAX_BOX_NUMBER 20
  #define BOX_MAX_AGE_SEC 5
  #define BOX_IP_STR_LEN 20
  #define BOX_SN_STR_LEN 20
  #define BOX_EXTRA_STR_LEN 20
  #define BOX_LIST_MSG_STR_LEN (BOX_IP_STR_LEN+BOX_SN_STR_LEN+BOX_EXTRA_STR_LEN)
  #define BOX_LIST_MSG_STR_HEAD_LEN 100
  #define BOX_LIST_MSG_STR_TRAIL_LEN 100
  
  #define MAXIMUM_REQ_EN_KEV 150.0
  #define  VTHMAX_UPPER_LIMIT    2200
  
  
  
  struct box_status{
    float Tcold;
    float Thot;
    float RH;
    float HVVoltage;
    float HVCurrent;
    int   PeltierPWR;
    float Tbox;
  };
  
//   struct box{
//     int SN;
//     char IPAdd[BOX_IP_STR_LEN];
//     int cmd_port;
//     int data_port;
//     int FakeBoxMode;
//     struct box_status status;
//     clock_t lastTimeStamp;
//     pthread_mutex_t box_mutex;
//   };
  
//   struct boxset{
//     struct box boxlist[MAX_BOX_NUMBER];
//     int PlaceHolder[MAX_BOX_NUMBER];
//     int boxes_total;
//   };
//   
  
  // Weather variable extracted from UDP stream.
   float m_temperaturePeltierCold = 999.99;
  float m_temperaturePeltierHot = 999.99;
  float m_HighVoltageTension = 999.99;
  float m_boxHumidity = 999.99;
  float m_boxTemperature = 999.99;
  float m_peltierPower = 999.99;
  
  bool m_alarmTempTooHot = false;
  bool m_alarmTempTooHotEnabled = true;
  bool m_alarmTempTooCold = false;
  bool m_alarmTempTooColdEnabled = false;
  bool m_alarmHumidity = false;
  bool m_alarmHumidityEnabled = true;
  
  
  void boxHumidityTempMonitor();
  
  
  
  int pixirad_send_cmd(char * cmd_str, char* reply_str,int reply_buffer_len,int options);
//   void print_detsett(Detector DetSett);
  
  
  #define SOCKET_ERROR -1
  
  #define NOVERBOSE 0
  #define VERBOSITY_LOW 1
  #define VERBOSITY_MEDIUM 2
  #define VERBOSITY_HIGH 3
  #define VERBOSITY_VERYHIGH 4
  #define VERBOSITY_ULTRAHIGH 5
  
  
  #define MACHINE_SHORT_DELAY_MS 10
  #define MACHINE_LONG_DELAY_MS 500
  #define HUMAN_SHORT_DELAY_MS 1000
  
  /**************************************************************/
  #define MAX_FILENAME_STR_LENGTH 1000
  #define MAX_MSG_STR_LENGTH 10000
  #define MAX_SHORT_MSG_STR_LEN 50
  #define MAX_INPUT_STRING_LENGTH MAX_MSG_STR_LENGTH
  #define MAX_CMD_PARAMETERS 15
  #define CMD_STR_ELEMENT_MAX_LENGTH 50
  /**************************************************************/
  #define CONN_RX_BUFF_LEN 8192
  #define CONN_TX_BUFF_LEN 8192
  /**************************************************************/
  #define SRV_TAG_STR "SRV"
  #define DAQ_TAG_STR "DAQ"
  #define SYS_TAG_STR "SYS"
  /**************************************************************/
  
  
  /*
   / / stru*ct Detector{
   float HighTh1;
   float LowTh1;
   float HighTh0;
   float LowTh0;
   int HighTh1DAC;
   int LowTh1DAC;
   int HighTh0DAC;
   int LowTh0DAC;
   int VthMax;
   int Ref;
   int AutoFS;
   int HVBias;
   int Tcold;
   // };*/
  
    
  
  int MAXSIZEFORSOCKETBUFFER = 2562177280; // 4000000; 256217728  25621772800
  bool m_flagAcquisitionRunning;
  
  int m_acq_frame_nb; // nos of frames acquired
  
  bool m_reset_m_nbFramesAlreadyAcq = 0; // Will let know if we start a new serie of image next run.
  
  // fake
  std::string  m_fake = "Off"; // "Off" or "On"
  
  //Sensor config  
  //    command = string("! Sensor_Config 60 30 15 2 1 7 NODTF NONBI");  
  //Thresholds

  
  std::string m_readOutSchema ="DEFAULT"; // Can be CHIP1 CHIP2 ....
  
  
  int m_nCyclesUdpDelay = 0;

  float m_sensorConfigHighThreshold1 = 60;
  float m_sensorConfigLowThreshold1 = 8;
  float m_sensorConfigHighThreshold0 = 60;
  float m_sensorConfigLowThreshold0 = 8;
  
  int  m_sensorConfigHighThreshold1DAC = 0;
  int m_sensorConfigLowThreshold1DAC = 0;
  int m_sensorConfigHighThreshold0DAC = 0;
  int m_sensorConfigLowThreshold0DAC = 0;
  
  // Internal biais
  int m_sensorConfigRefInternalBiais = 1;
  int m_sensorConfigAusFSInternalBiais = 7;
  // Dead time free mode or not 
  std::string m_sensorConfigDeadTimeFreeMode = "NODTF"; // DTF or NODTF
  std::string m_sensorConfigNBI = "NONBI"; // NBI or NONBI
  
  
  
  // Sensor Model Config :
  //! Sensor_Model_Config Asic Hybrid Build
  std::string m_sensorConfigAsic = "PII"; // PII PIII
  std::string m_sensorConfigHybrid = "CDTE"; // CDTE GAAS
  std::string m_sensorConfigBuild = "PX8"; // PX1 PX2 PX8
  
  int m_nbModules = 8;
  
  // Run config (maybe later ?)
  //! Run_Config RunMode FilenameA FilenameB Parameters
  std::string m_runConfigRunMode = "DATA"; // DATA or "COLLECT_FLAT", "SUMMED_DATA", "SUMMED_COLLECT_FLAT";
  std::string m_runConfigFilenameA = "-"; // filename without path. or "-"
  std::string m_runConfigFilenameB = "-"; // filename without path. or "-"
  
  
  // Environmental parameters config
  //! Env_Config CoolT CollStatus HVVal HVStatus
//   int m_envConfigCoolingTemperature = -30; // deg C
  int m_envConfigCoolingTemperature = 15; // deg C
  int m_envConfigCoolingOnOff = 1; //0-off / 1-on
  int m_envConfigHighVoltageBiais = 0; // Volts
  int m_envConfigHighVoltageBiaisMax = 2200; // Volts
  int m_envConfigHighVoltageBiaisOnOff = 1; //0-off / 1-on
  
  //  ! Bias_management_Config BiasMode TonDelay ToffDelay RefreshPeriod
  //  ! Bias_management_Config AUTOHV 1 4 5");    
  
  std::string m_biaisMode = "AUTOHV"; // BiasMode (S) One of "AUTOHV", "AUTOHV_LC", "STDHV"; 
  //    std::string m_biaisMode = "STDHV"; // BiasMode (S) One of "AUTOHV", "AUTOHV_LC", "STDHV"; 
  int m_tonHvAcquisitionDelay = 1; // If AutoHV, Number of seconds Biais is turned on before aquisition. Acquisition Delay (secs.) from the HV Turn On; 
  int m_toffHvAcquisitionDelay = 4; // ToffDelay (F) HV Refresh cycle duration (secs.); 
  int m_HvRefreshPeriod = 5; //RefreshPeriod (N) HV Refresh cycle period (#of frames);
  
  
  // Haha ! Acquisition parameters 
  //  ! Acquire Frames Shutt_ms Pause_ms RunMode TrgMode HVMode
  
  int m_nbFramesAcq = 1; //    Frames(N) Number of frames to acquire;
  int m_nbFramesAlreadyAcq = 0; //    Frames(N) Number of frames to acquire;
  int m_shutterMs = 1; //    Shutt_ms(R) Shutter width(ms);
  int m_pauseBetweenAcq = 0; //    Pause_ms(R) Pause in ms;
  std::string m_RunModeColors = "1COL0"; //    RunMode(S) Selects the detector run mode : “2COL” Two colors;//    “1COL0” One color reg0;//    “1COL1” One color reg1;//    “DTF” One Color in Dead Time Free;//    “4COL” Four colors;
  std::string m_TriggerMode = "INT"; //    TrgMode(S) Selects the trigger configuration : “INT” (default) Internal trigger. Acquisition starts when the LOOP command is received by the box and Frames acquisition is internally triggered;
  //    “EXT1” External trigger fixed shutter width, a positive edge at “SyncIn” triggers the frame acquisition. Shutter duration is internally managed;
  std::string m_HvMode = "NULL"; //    HVMode(S) HV management Mode : - Please Set to "NULL" this parameter
  std::string m_moderationMode = "UNMOD"; //    TrsfMode   "MOD" or "UNMOD" or "UNMODH" 
  
  
  
  // Auto reset mode
  int m_autoResetMode = 1; // 1 will restart to 0 the number of frame for each new loop. (1 default)
  
  
  std::string m_syncInPolarity = "POS";
  std::string m_syncOutPolarity = "POS";
  std::string m_syncOutFunction = "SHUTTER";
  
  
  // List of booleans flags to memorise if the important commands have been received and acknowledge by the detector. // False => the command is not acknowledged yet, True, it has been acknowledge. // RX take care of setting it to True when aware of it. // Prepare and Start take care of presetting to False.    
  // Mutex for writing to flags:    
  //     std::mutex m_mutexRX;  
  
  std::mutex m_mutexCommandTCP;
  std::mutex m_mutexUDPImage;
  std::mutex m_mutexUDPTemperature;
  
 // std::mutex m_mutexStatus;
 // std::condition_variable m_condVariableRX;
  
  //pthread_mutex_t m_mutex;
  
  
  
  //init flags
  bool m_flagSetFake = false;
  bool m_flagSensorModelConfig = false;    
  // prepares flags
  bool m_flagRunConfig = false;
  bool m_flagSensorConfig = false;
  bool m_flagBiasMngConfig = false;
  bool m_flagEnvConfig = false;           
  // Start acq flags
  bool m_flagAcquireCmd = false;    
  // Stop flags // Useful ?
  bool m_flagAcquireStopCmd = false;
  
  
  
  // Mutex to handle status
  std::mutex m_mutexStatus;
  HwInterface::StatusType::Basic  m_pixiradStatus;
  
  
  
  // set status handling mutex
  void setStatusDetector(HwInterface::StatusType::Basic status );
  
  void getStatusDetector(HwInterface::StatusType::Basic& status);
  
  void getSize(Size & size);
  
  // Buffer for Lima
  //    BufferCtrlMgr m_buffer;
  //    HwBufferCtrlObj*  m_bufferCtrlObj;
  StdBufferCbMgr* buffer_mgr;
  StdBufferCbMgr* finalBufferMgr ;
  
  SoftBufferCtrlObj& m_bufferCtrlObj;
  
  SoftBufferCtrlObj* m_reconstructionBufferCtrlObj;
  
//   int m_nbOfFrameInReconstructionBuffer = 32;
  int m_nbOfFrameInReconstructionBuffer = 128;
  
  
  
  bool m_allImagesReceived = true;
  
  
  //   ////////////// PREPARE AND INIT ?///////////////
  int cmd_socket = 0;
  
//   int m_vthMax = 0;
  
  
  int update_thresolds_from_energies();
  
  
  
  int m_numberOfUDPPacketsPerImage =0;
  
  
  
  
  
  ushort *m_acknowledgatorPointer;
  
  
  // Mode where e pixirad8 is used with only one module. -1 means 8 modules together.
//     int m_oneChipModeOutOfEight = -1;  
    int m_oneChipModeOutOfEight = -1;  
    
   bool m_seedModeForDebugOnlyInOneChipWithPX8 = false;
  
  
  // /////////////// START   ////////////////////
  
  
  
  #define DATA_BUFFLEN_BYTES 2048
  #define HEADER_BUFFLEN_BYTES 32
  #define BUFLEN (DATA_BUFFLEN_BYTES+HEADER_BUFFLEN_BYTES)
  #define MAX_PACK_LEN 1448
  /******************************************
   * Originally PACKET_TAG was 2 bytes. At some point more space was needed by to communicate
   * the information about the remaining frames and a Frame_Tag containing informations about
   * the image trasfereed from the UDP thread to the Processor thread
   * I left the OLD Packet TAG OFFSET and expanded its range. FRAME_TAG and REMAINING_FRAMES are allocated
   * in PACKET_TAG too.
   * SInce the ORGINAL PACKET_TAG was two bytes, the new field will begin at offset 2
   */
  
  
  /******************************************/
  #define PACKET_ID_BYTES 2
  #define PACKET_ID_OFFSET 2
  #define PACKET_CRC_BYTES 4
  #define SLOT_ID_BYTES 1
  #define RUN_MASK_BYTES 1
  // the UDP packet has 4 bytes in the head the second two (2,3) are the packet identification number. These doesn't need
  // to be copied in the processing buffer.
  // The first two bytes (0,1) are the RUN mask byte(wich contains the register, autocal...infos) and the slot_id
  // byte. these two bytes needed to be copied to the processing buffer
  
  #define PACKET_SENSOR_DATA_OFFSET (PACKET_ID_BYTES+SLOT_ID_BYTES+RUN_MASK_BYTES)
  #define PACKET_EXTRA_BYTES (PACKET_ID_BYTES+SLOT_ID_BYTES+RUN_MASK_BYTES+PACKET_CRC_BYTES)
  #define PACKET_SENSOR_DATA_BYTES (MAX_PACK_LEN-PACKET_EXTRA_BYTES)
  
  
  
  
  #define BUFFER_TAG_BYTES 10
  #define BUFFER_TAG_OFFSET 0
  #define FRAME_TAG_OFFSET 2
  #define REMAINING_FRAMES_OFFSET 4
  
  #define SENSOR_DATA_OFFSET (BUFFER_TAG_OFFSET+BUFFER_TAG_BYTES)
  #define SENSOR_DATA_BYTES PACKET_SENSOR_DATA_BYTES
  
  #define PIXIEII_MODULES 1
  
  #define FRAME_HAS_ALIGN_ERRORS 0x20
  #define REG_PACKET 0x80
  #define SLOT_ID_MASK 0xff
  #define SLOT_ID_OFFSET 1
  #define AUTOCAL_DATA 	0x40
  #define CNT_DATA 		0x00
  
  
  //bit masks used to store parameters in dummy0_1
  //dummy0
  #define DUMMY_0_OFFSET 0
  #define PIXIE_THDAC_MASK 0x1f
  #define PIXIE_THDAC_OFFSET 0
  //dummy1
  #define DUMMY_1_OFFSET 8
  #define LOOP_MODE_OFFSET 0
  #define LOOP_MODE_MASK 0xff
  #define LOOP_COLOR_MODE_OFFSET 0
  #define LOOP_COLOR_MODE_MASK 0xf
  #define LOOP_DTF_MODE_OFFSET 4
  #define LOOP_DTF_MODE_MASK 0xf
  
  
  #define MAX_STRLEN 200
  //il limite sul numero massimo di frames dovrebbe essere il piu alto possibile immaginabile, la memoria per i frame è allocata dinamicamente
  #define MAX_PENDING_BUFFERS 100000
  
  /****************PX1********************/
  #define PII_PX1_DEFAULT_NPACK 360
  #define PII_PX1_AUTOCAL_NPACK 135
  #define PII_PX1_DAQ_PACK_FRAGM 45
  
  
  #define PIII_PX1_DEFAULT_NPACK 270
  #define PIII_PX1_AUTOCAL_NPACK 180
  #define PIII_PX1_DAQ_PACK_FRAGM 45
  
  /****************PX2********************/
  #define PII_PX2_DEFAULT_NPACK (360*2)
  #define PII_PX2_AUTOCAL_NPACK (135*2)
  #define PII_PX2_DAQ_PACK_FRAGM 90
  
  
  #define PIII_PX2_DEFAULT_NPACK (270*2)
  #define PIII_PX2_AUTOCAL_NPACK (180*2)
  #define PIII_PX2_DAQ_PACK_FRAGM 90
  /****************PX8********************/
  #define PII_PX8_DEFAULT_NPACK (2539)
  #define PII_PX8_AUTOCAL_NPACK (135*8)
  #define PII_PX8_DAQ_PACK_FRAGM 90
  
  
  #define PIII_PX8_DEFAULT_NPACK (270*8)
  #define PIII_PX8_AUTOCAL_NPACK (180*8)
  #define PIII_PX8_DAQ_PACK_FRAGM 90
  
  #define PII_PX8_MONO_DEFAULT_NPACK (360)
  #define PII_PX8_MONO_AUTOCAL_NPACK (135)
  #define PII_PX8_MONO_DAQ_PACK_FRAGM 90
  
  
  #define PIII_PX8_MONO_DEFAULT_NPACK (270)
  #define PIII_PX8_MONO_AUTOCAL_NPACK (180)
  #define PIII_PX8_MONO_DAQ_PACK_FRAGM 90
  /**************************************/
  
  
  
  #define MOD_UDP_REMOTE_PORT 3333
  #define PORTB 2224
  #define MAXBUF (256217728)
  #define BYTES_PER_ROWS 30
  
  void threadWhoReceiveImagesFromCameraThroughUDP();
  
  int sendCommand(std::string command, char commandAnswerFromDetector[MAX_MSG_STR_LENGTH], bool waitForAnswer);
  
  void getImages();
  void getImagesInAThread();
//   void getDetectorImageSize(Size size);
//   void getConfigBuild(SensorConfigBuild build);
  void prepareAcq();
  
  void printMissingImageInfo();
  
  bool m_stopAcquisition;
  void stopAcq();
  
  
  int m_UdpPortImages = 9999; // 2223 for PX1
  short unsigned int * m_conversion_table;
  
  
private:
  std::string m_ipAdressDetector = "192.168.0.1";
  int m_TcpPort = 2222;
  int m_weatherUdpPort = 2224;
  int m_UdpPort = 4444;
  
  std::thread m_imageThread;
  std::thread m_boxHumidityTempMonitor;
  
    //   pixiradDetector();
/*  
    pixiradDetector(std::string ipAdressDetectorServer, int TcpPort,SoftBufferCtrlObj& buffer);
    ~pixiradDetector();
    // Functions
    int reconnectSocket();
    void stopSocketThread();
    void sendCommand(std::string command);
    void closeConnection();

    //Thread functions
    void watchDogThreads();
    void TxThread();
    void RxThread();


    // Lima mimetism 
    void prepare();
    void acquisition();
    void stopAcquisition();

    // Socket
    int m_socketToPixiradServer = 0;

    // Real Threads
    std::thread m_watchdog  ;
    std::thread m_tcpRxThread  ;
    std::thread m_tcpTxThread ;

    std::vector<const char *> m_TxBuffer;

    // All public ? or getter/setter with private ?

    bool m_flagAcquisitionRunning;

    int m_acq_frame_nb; // nos of frames acquired

    bool m_reset_m_nbFramesAlreadyAcq = 0; // Will let know if we start a new serie of image next run.
    
    // fake
    std::string  m_fake = "Off"; // "Off" or "On"

    //Sensor config  
    //    command = string("! Sensor_Config 60 30 15 2 1 7 NODTF NONBI");  
    //Thresholds
    float m_sensorConfigHighThreshold1 = 60;
    float m_sensorConfigLowThreshold1 = 0;
    float m_sensorConfigHighThreshold0 = 15;
    float m_sensorConfigLowThreshold0 = 0;
    // Internal biais
    int m_sensorConfigRefInternalBiais = 1;
    int m_sensorConfigAusFSInternalBiais = 7;
    // Dead time free mode or not 
    std::string m_sensorConfigDeadTimeFreeMode = "NODTF"; // DTF or NODTF
    std::string m_sensorConfigNBI = "NONBI"; // NBI or NONBI



    // Sensor Model Config :
    //! Sensor_Model_Config Asic Hybrid Build
    std::string m_sensorConfigAsic = "PII"; // PII PIII
    std::string m_sensorConfigHybrid = "CDTE"; // CDTE GAAS
    std::string m_sensorConfigBuild = "PX1"; // PX1 PX2 PX8


    // Run config (maybe later ?)
    //! Run_Config RunMode FilenameA FilenameB Parameters
    std::string m_runConfigRunMode = "DATA"; // DATA or "COLLECT_FLAT", "SUMMED_DATA", "SUMMED_COLLECT_FLAT";
    std::string m_runConfigFilenameA = "-"; // filename without path. or "-"
    std::string m_runConfigFilenameB = "-"; // filename without path. or "-"


    // Environmental parameters config
    //! Env_Config CoolT CollStatus HVVal HVStatus
    int m_envConfigCoolingTemperature = 20; // deg C
    int m_envConfigCoolingOnOff = 1; //0-off / 1-on
    int m_envConfigHighVoltageBiais = 0; // Volts
    int m_envConfigHighVoltageBiaisOnOff = 0; //0-off / 1-on

    //  ! Bias_management_Config BiasMode TonDelay ToffDelay RefreshPeriod
    //  ! Bias_management_Config AUTOHV 1 4 5");    

    std::string m_biaisMode = "AUTOHV"; // BiasMode (S) One of "AUTOHV", "AUTOHV_LC", "STDHV"; 
//    std::string m_biaisMode = "STDHV"; // BiasMode (S) One of "AUTOHV", "AUTOHV_LC", "STDHV"; 
    int m_tonHvAcquisitionDelay = 1; // If AutoHV, Number of seconds Biais is turned on before aquisition. Acquisition Delay (secs.) from the HV Turn On; 
    int m_toffHvAcquisitionDelay = 1; // ToffDelay (F) HV Refresh cycle duration (secs.); 
    int m_HvRefreshPeriod = 50000; //RefreshPeriod (N) HV Refresh cycle period (#of frames);


    // Haha ! Acquisition parameters 
    //  ! Acquire Frames Shutt_ms Pause_ms RunMode TrgMode HVMode

    int m_nbFramesAcq = 1; //    Frames(N) Number of frames to acquire;
    int m_nbFramesAlreadyAcq = 0; //    Frames(N) Number of frames to acquire;
    int m_shutterMs = 1; //    Shutt_ms(R) Shutter width(ms);
    int m_pauseBetweenAcq = 0; //    Pause_ms(R) Pause in ms;
    std::string m_RunModeColors = "1COL0"; //    RunMode(S) Selects the detector run mode : “2COL” Two colors;//    “1COL0” One color reg0;//    “1COL1” One color reg1;//    “DTF” One Color in Dead Time Free;//    “4COL” Four colors;
    std::string m_TriggerMode = "INT"; //    TrgMode(S) Selects the trigger configuration : “INT” (default) Internal trigger. Acquisition starts when the LOOP command is received by the box and Frames acquisition is internally triggered;
    //    “EXT1” External trigger fixed shutter width, a positive edge at “SyncIn” triggers the frame acquisition. Shutter duration is internally managed;
    std::string m_HvMode = "NULL"; //    HVMode(S) HV management Mode : - Please Set to "NULL" this parameter

    
    
    // List of booleans flags to memorise if the important commands have been received and acknowledge by the detector. // False => the command is not acknowledged yet, True, it has been acknowledge. // RX take care of setting it to True when aware of it. // Prepare and Start take care of presetting to False.    
    // Mutex for writing to flags:    
//     std::mutex m_mutexRX;  
    std::mutex m_mutexRX;
    std::condition_variable m_condVariableRX;
       
    
    
    //init flags
    bool m_flagSetFake = false;
    bool m_flagSensorModelConfig = false;    
    // prepares flags
    bool m_flagRunConfig = false;
    bool m_flagSensorConfig = false;
    bool m_flagBiasMngConfig = false;
    bool m_flagEnvConfig = false;           
    // Start acq flags
    bool m_flagAcquireCmd = false;    
    // Stop flags // Useful ?
    bool m_flagAcquireStopCmd = false;
    

    
    // Mutex to handle status
    std::mutex m_mutexStatus;
    HwInterface::StatusType::Basic  m_pixiradStatus;
    
    
    
    // set status handling mutex
    void setStatusDetector(HwInterface::StatusType::Basic status );
    
    void getStatusDetector(HwInterface::StatusType::Basic& status);
        
        

	// Buffer for Lima
//    BufferCtrlMgr m_buffer;
//    HwBufferCtrlObj*  m_bufferCtrlObj;
    StdBufferCbMgr* buffer_mgr;
    
    SoftBufferCtrlObj& m_bufferCtrlObj;*/
// private:  
//     std::string m_ipAdressDetectorServer = "NOT INITIALISED";
//     int m_TcpPort = 9999;

};
}} // namespaces
#endif
