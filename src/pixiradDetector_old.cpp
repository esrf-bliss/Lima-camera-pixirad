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

//#include <regex>  // Command acknowledgment
#include <regex.h>  // Command acknowledgment

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION 

using namespace std;
using namespace lima;
using namespace lima::Pixirad;

// Constructor. Define the address of the pixirad server which link the camera to the detector pc.
// pass also the buffer controller, so it can give a proper buffer for each images.

pixiradDetector::pixiradDetector(std::string ipAdressDetectorServer, int TcpPort, SoftBufferCtrlObj& buffer): m_bufferCtrlObj(buffer), m_ipAdressDetectorServer(ipAdressDetectorServer), m_TcpPort(TcpPort) {
    
    DEB_CONSTRUCTOR();
    DEB_TRACE() << "Starting threads for socket handling";
    DEB_TRACE() << "TCP Server for the pixirad is " << DEB_VAR2(ipAdressDetectorServer, TcpPort);

    setStatusDetector(HwInterface::StatusType::Config);
    
//     m_status = Camera::Ready
    
// ./camera/adsc/src/AdscCamera.cpp:                       return Camera::Ready;
// ./camera/adsc/src/AdscCamera.cpp:                       return Camera::Exposure;
// ./camera/adsc/src/AdscCamera.cpp:                       return Camera::Readout;
// ./camera/adsc/src/AdscCamera.cpp:                       return Camera::Latency;

    m_TxBuffer.clear();
    
    // TODO: Start watchdog  Which will restart the Rx/Tx Threads if they go berserk.    
    m_watchdog= std::thread(&pixiradDetector::watchDogThreads, this);
    
    setStatusDetector(HwInterface::StatusType::Ready);
    
}




pixiradDetector::~pixiradDetector(){
	DEB_DESTRUCTOR();
        DEB_TRACE() << "Joining watchdog" ;
        if (m_watchdog.joinable() ) {m_watchdog.detach();}
        DEB_TRACE() << "Stopping RX/TX" ;
        stopSocketThread();
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
    std::unique_lock<std::mutex> uniqLock(m_mutexStatus);
    
    status = m_pixiradStatus;
    DEB_TRACE() << "Access to status " << DEB_VAR1(status);
    
}



void pixiradDetector::watchDogThreads(){
    
    DEB_MEMBER_FUNCT();
 
    // Try to connect the socket until it works, then check every second if threads still alive.
    // If threads are joinable, then something failed in it. close all, and restart.
    
    
    int trial = 0;
    int notconnected = 1;
    
    while(1){
        if(notconnected){
            while (notconnected){
                DEB_TRACE() << "Connection trial :" << DEB_VAR1(++trial);
                notconnected = reconnectSocket();
                
                if(notconnected == 1){
                    sleep(1);
                    DEB_TRACE() << "Connection impossible, is the pixirad server started ?" << DEB_VAR1(notconnected);
                }
            }
        }
        
        // Check every sec if alive or not.   
        sleep(1);
//         DEB_TRACE() << "Thread status :" << DEB_VAR2(m_tcpTxThread.joinable(),m_tcpRxThread.joinable());
        // If one thread is down, stop both and restart
        if (  not(m_tcpTxThread.joinable())  or not(m_tcpRxThread.joinable())){        
                DEB_TRACE() << "One thread is down, probably a connection pbl..." ;
                notconnected = 1;
                stopSocketThread();        
        }
        
    }
    
}


void pixiradDetector::stopSocketThread() {    
    
    DEB_MEMBER_FUNCT();
        
    setStatusDetector(HwInterface::StatusType::Fault);
    
    if (m_tcpTxThread.joinable() ) {        
        DEB_TRACE() << "Stopping Thread TX ";
        m_tcpTxThread.detach();
        DEB_TRACE() << "Thread TX stopped";
    }
    
    if (m_tcpRxThread.joinable() ) {
        DEB_TRACE() << "Stopping Thread RX ";
        m_tcpRxThread.detach();
        DEB_TRACE() << "Thread RX stopped";
    }    
    
}




// For one socket, two threads:
// RX will get responses and images.
// TX will send command in asked order 

int pixiradDetector::reconnectSocket() {

    DEB_MEMBER_FUNCT();

    setStatusDetector(HwInterface::StatusType::Config);
    
    // first disconnect if need :
    stopSocketThread();
    
    struct sockaddr_in serv_addr;

    if ((m_socketToPixiradServer = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        DEB_TRACE() << "Error : Could not create a listening socket ";
        return 1;
    }

    memset(&serv_addr, '0', sizeof (serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(m_TcpPort);

    if (inet_pton(AF_INET, m_ipAdressDetectorServer.c_str(), &serv_addr.sin_addr) <= 0) {
        DEB_TRACE() << "inet_pton error occured";
        return 1;
    }

    if (connect(m_socketToPixiradServer, (struct sockaddr *) &serv_addr, sizeof (serv_addr)) < 0) {
        DEB_TRACE() << "Error : Connect Failed ";
        return 1;
    }

    
    
    // Connection is OK, start the Rx and Tx threads.
    m_tcpRxThread = std::thread(&pixiradDetector::TxThread, this);
    m_tcpTxThread = std::thread(&pixiradDetector::RxThread, this);    

    //     a member function can't be called without an object. Provide a pointer to this so that the current object is used: 
    //    std::thread m_tcpThread(&pixiradDetector::reconnectSocket, this, m_ipAdressDetectorServer, m_m_TcpPort); /// WORKS

    setStatusDetector(HwInterface::StatusType::Ready);
    
    return 0;

}


// Main thread, handle message reception, extraction, and pointing of images to Lima.

void pixiradDetector::RxThread() {
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "RX --- RX started";
    //    When a Server transmits a message to the Client It must first transmits four bytes representing the Message Total Size (Stringlen + Binarylen + 4) in Bytes then a second set of four bytes indicating the String section length (Stringlen). Then the string section and the binary section then are sent out to the Client in the same order. Zero length binary sections are allowed, zero length string sections are not.

    // TODO bool for stopping


    uint32_t fullMessageLenghtInt;

    uint32_t stringLengthIncludeInFullMessageInt;

    uint32_t binaryPartLength;


    char * stringPartOfMessageReceivedStr;
//     bool continueFlag;
    
    while (1) {
        fullMessageLenghtInt = 0;
        stringLengthIncludeInFullMessageInt = 0;
        binaryPartLength = 0;

        
        DEB_TRACE()<<  "RX --- AVANT RECV1 ";
        recv(m_socketToPixiradServer, &fullMessageLenghtInt, 4, MSG_WAITALL);
        DEB_TRACE()<<  "RX --- APRES RECV1 ";


        if (fullMessageLenghtInt <= 0) {
            DEB_TRACE()<<  "RX --- ERROR ON READING SOCKET ---- Leaving RX thread ";
            // Banzai
            m_tcpRxThread.detach();
            return;
        }
        
        // New message 
        if (fullMessageLenghtInt > 0) {

            DEB_TRACE()<<  "RX --- AVANT RECV2 ";
            recv(m_socketToPixiradServer, &stringLengthIncludeInFullMessageInt, 4, MSG_WAITALL);
            DEB_TRACE()<<  "RX --- APRES RECV2 ";
            
            // String part of message
            if (stringLengthIncludeInFullMessageInt > 0) {

                // Reading String part 
                stringPartOfMessageReceivedStr = new char[stringLengthIncludeInFullMessageInt + 1];

                DEB_TRACE()<<  "RX --- AVANT RECV3 ";
                recv(m_socketToPixiradServer, stringPartOfMessageReceivedStr, stringLengthIncludeInFullMessageInt, MSG_WAITALL);
                DEB_TRACE()<<  "RX --- APRES RECV3 ";

                DEB_TRACE() << "RX --- String received :  " << DEB_VAR1(stringPartOfMessageReceivedStr);
                
                
                regex_t regex;
                const char * constString = stringPartOfMessageReceivedStr;
                // Looking for some acknowledgments from the detector, and setting the proper flags:
                
                {
                    DEB_TRACE()<<  "RX --- AVANT MutexRX ";
                    std::unique_lock<std::mutex> uniqLock(m_mutexRX);
                    DEB_TRACE()<<  "RX --- APRES MutexRX ";
                    
                    /* Compile regular expression */
                    regcomp(&regex, "CNSRV GOT:! Set_Fake Off Command Accepted!.*" , 0);                    
                    if ( regexec(&regex, constString, 0, NULL, 0) == 0 ){m_flagSetFake=true;}
    
    
                    regcomp(&regex, "CNSRV GOT:! Sensor_Model_Config.*Command Accepted!.*" , 0);
                     if ( regexec(&regex, constString, 0, NULL, 0) == 0 ){m_flagSensorModelConfig=true;}
                    
                    
                    regcomp(&regex, "CNSRV GOT:! Run_Config DATA.*Command Accepted!.*" , 0);
                     if ( regexec(&regex, constString, 0, NULL, 0) == 0 ){m_flagRunConfig=true;}
                    
                    
                    regcomp(&regex, "CNSRV GOT:! Sensor_Config.*Command Accepted!.*" , 0);
                     if ( regexec(&regex, constString, 0, NULL, 0) == 0 ){m_flagSensorConfig=true;}
                    
                    
                    regcomp(&regex, "CNSRV GOT:! Bias_management_Config.*Command Accepted!.*" , 0);
                     if ( regexec(&regex, constString, 0, NULL, 0) == 0 ){m_flagBiasMngConfig=true;}
                    
                                    
                    regcomp(&regex, "CNSRV GOT:! Env_Config.*Command Accepted!.*" , 0);
                     if ( regexec(&regex, constString, 0, NULL, 0) == 0 ){m_flagEnvConfig=true;}
                    
                    regcomp(&regex, "CNSRV GOT:! Acquire_Stop" , 0);
                     if ( regexec(&regex, constString, 0, NULL, 0) == 0 ){
                         m_flagAcquireStopCmd=true;                         
                         setStatusDetector(HwInterface::StatusType::Ready);
                    }
                     else{                        
                        regcomp(&regex, "CNSRV GOT:! Acquire.*" , 0);
                        if ( regexec(&regex, constString, 0, NULL, 0) == 0 ){
                            m_flagAcquireCmd=true;
//                             setStatusDetector(HwInterface::StatusType::Exposure);
                        }
                     }

                    uniqLock.unlock();
                    m_condVariableRX.notify_one();
                }
                
                
                
                
                // Binary length = Total - String - 4
            } else {
                DEB_TRACE() << "STRING LENGTH CANNOT BE 0  EXIT ";
                fullMessageLenghtInt = 4;
                stringLengthIncludeInFullMessageInt = 0; // should avoid entering binary block
                    
                // Banzai
                m_tcpRxThread.detach();
                return;
            }

            
            // Binary part of Message

            binaryPartLength = fullMessageLenghtInt - stringLengthIncludeInFullMessageInt - 4;
            
            DEB_TRACE() << "Full message lenght :" << DEB_VAR1(fullMessageLenghtInt);
            DEB_TRACE() << "string  lenght :" << DEB_VAR1(stringLengthIncludeInFullMessageInt);
            DEB_TRACE() << "image lenght :" << DEB_VAR1(binaryPartLength);
            
            
            if (binaryPartLength > 0) {
                                
                
                setStatusDetector(HwInterface::StatusType::Readout);
    
                // Will reset the actual frame number if we do a new acquisition. // Works also if we stop and restart.
                // do not force when doing stop, like that we can still have the images already taken by the camera (usually one after stop is sent).
                // only force at each new start()
                
                if(m_reset_m_nbFramesAlreadyAcq){ 
                    m_nbFramesAlreadyAcq = 0;
                    m_reset_m_nbFramesAlreadyAcq = 0; 
                } 
                   
                
//                 DEB_TRACE() << "Message comport binary part" ;

                // we need the buffer manager from the buffer control obj.
                StdBufferCbMgr& buffer_mgr = m_bufferCtrlObj.getBuffer();
        
                // Lets read the socket little by little :
                const uint32_t sizeOfChunck = 2048;
                uint32_t nbBuffers = binaryPartLength / sizeOfChunck;
                uint32_t leftOver = binaryPartLength % sizeOfChunck;
                
                // Lima provide us a void pointer on the next image.
		void*  voidimageptr = buffer_mgr.getFrameBufferPtr(m_nbFramesAlreadyAcq); 
                // Let's use it as uint16_t to be able to calculate on it the next chunks.
		uint16_t * image = reinterpret_cast<uint16_t*>(voidimageptr);
                uint32_t nb;

                for (nb = 0; nb < nbBuffers; nb = nb + 1) {
                    int positionInmemoryAsInt = nb * sizeOfChunck / 2;
                    
                    DEB_TRACE()<<  "RX --- AVANT RECV4  " << DEB_VAR1(positionInmemoryAsInt);
                    recv(m_socketToPixiradServer, &image[positionInmemoryAsInt], sizeOfChunck, MSG_WAITALL);
                    DEB_TRACE()<<  "RX --- APRES RECV4  " << DEB_VAR1(positionInmemoryAsInt);

//  cout << "buffer nb : " << std::to_string(nb) << " / " << std::to_string(nbBuffers-1)<< "        " << std::to_string((nb+1)*sizeOfChunck) << "";

                }
                if (leftOver > 0) {
                    DEB_TRACE()<<  "RX --- AVANT RECV5  " << DEB_VAR3(sizeOfChunck,nbBuffers,leftOver);
                    recv(m_socketToPixiradServer, &image[sizeOfChunck * nbBuffers], leftOver, MSG_WAITALL);
                    DEB_TRACE()<<  "RX --- APRES RECV5  " << DEB_VAR3(sizeOfChunck,nbBuffers,leftOver);
                }
		
                
                // Build a frame info for Lima.
		HwFrameInfoType frame_info;
		frame_info.acq_frame_nb = m_nbFramesAlreadyAcq;// First image is 0 for frame info
// 		continueFlag = buffer_mgr.newFrameReady(frame_info);
                buffer_mgr.newFrameReady(frame_info);
                
                m_nbFramesAlreadyAcq++; // Will be 1 after first image //                
                
                DEB_TRACE() << "Total images received  " << DEB_VAR1(m_nbFramesAlreadyAcq) << " / " << DEB_VAR1(m_nbFramesAcq);
                
                if(m_nbFramesAlreadyAcq == m_nbFramesAcq){
                 setStatusDetector(HwInterface::StatusType::Ready);
                }
                else{
                 setStatusDetector(HwInterface::StatusType::Readout);                    
                }
                
            }

        }

    }

}




// TxThread has its own buffer of pending commands that it will send in order.
void pixiradDetector::TxThread() {
    
    DEB_MEMBER_FUNCT();
    DEB_TRACE() <<  "TX --- TX started, waiting for sending commands to pixirad server.";
    //    Client to Server message
    //    When a Client transmits a message to the Server It must first transmits 
    //    four bytes representing the Message Size in Bytes (Stringlen) then the 
    //    message string itself. 
    int resultConnection = 0;

    while (1) {

        if (m_TxBuffer.size() > 0) {

            const char * command = m_TxBuffer.front();

            DEB_TRACE() << "TX --- Sending : " << DEB_VAR1(command);



            uint32_t sizeOfCommand = strlen(command);
            // sending the size
            resultConnection = send(m_socketToPixiradServer, &sizeOfCommand, sizeof (sizeOfCommand), 0);

            if (resultConnection < 0) {
                DEB_TRACE() << "TX --- ERROR writing to socket --- leaving TX thread" << DEB_VAR1(command);    
                // Banzai
                m_tcpTxThread.detach();
                return;
            }

            // Then the message itself.
            resultConnection = write(m_socketToPixiradServer, command, sizeOfCommand); // -1 to remove the trailing '\0' from string.
            if (resultConnection < 0) {
                DEB_TRACE() << "TX --- ERROR writing to socket --- leaving TX thread";       
                // Banzai
                m_tcpTxThread.detach();
                return;         
            }

            //TODO TEST CONNECTION BEFORE
            // TODO: Free malloc memory somewhere here.
            m_TxBuffer.erase(m_TxBuffer.begin());


        }
    }
    
    // Banzai
    m_tcpTxThread.join();
}

void pixiradDetector::closeConnection() {
    
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Closing socket";
    close(m_socketToPixiradServer);
    stopSocketThread();
}

void pixiradDetector::sendCommand(string command) {

    DEB_MEMBER_FUNCT();
    // TODO : Check that buffer is initialised

    int sizeMalloc = sizeof (char)*command.size() + 10;

    char* commandCopyChar = (char*) malloc(sizeMalloc);
    strcpy(commandCopyChar, command.c_str());

    // Free need to be done later, when buffer sent the command.
    //    const char * commandCopyChar = command.c_str();

    m_TxBuffer.insert(m_TxBuffer.end(), commandCopyChar);

}


