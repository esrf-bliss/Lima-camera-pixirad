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
    //   pixiradDetector();
  
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
    
    SoftBufferCtrlObj& m_bufferCtrlObj;
private:  
    std::string m_ipAdressDetectorServer = "NOT INITIALISED";
    int m_TcpPort = 9999;

};
}} // namespaces
#endif	/* PIXIRADDETECTOR_H */

