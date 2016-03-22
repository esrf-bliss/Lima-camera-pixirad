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

#ifndef PIXIRADCAMERA_H_
#define PIXIRADCAMERA_H_

#include <ostream>
#include <map>
//#include "lima/HwMaxImageSizeCallback.h"
#include "lima/HwBufferMgr.h"
//#include "lima/HwInterface.h"
#include "lima/Debug.h"
#include "lima/SizeUtils.h"
//#include "lima/Debug.h"
//#include "lima/Debug.h"
//#include "PixiradNet.h"


#include "lima/HwInterface.h" // For Status
#include "pixiradDetector.h" //Yves

using namespace std;

namespace lima {
namespace Pixirad {

//const static string actions[] = {"GET","SET","CMD"};
//const static string id = "MPX,";
//const int xPixelSize = 55; // pixel size is 55 micron
//const int yPixelSize = 55;

class BufferCtrlObj;

/*******************************************************************
 * \class Camera
 * \brief object controlling the Pixirad camera
 *******************************************************************/
class Camera {
DEB_CLASS_NAMESPC(DebModCamera, "Camera", "Pixirad");

public:

	Camera(std::string hostname, int cmdPort = 6341);
	~Camera();
            
            

	
    
	void init();
	void reset();
	void prepareAcq();
	void startAcq();
	void stopAcq();
        
//	void getStatus(); // Not good idea
	void getStatus(HwInterface::StatusType::Basic& status);
	int getNbHwAcquiredFrames();

	// -- detector info object
	void getImageType(ImageType& type);
	void setImageType(ImageType type);

	void getDetectorType(std::string& type);
	void getDetectorModel(std::string& model);
	void getDetectorImageSize(Size& size);
	void getPixelSize(double& sizex, double& sizey);

	// -- Buffer control object
	SoftBufferCtrlObj* getBufferCtrlObj();
// 	HwBufferCtrlObj* getBufferCtrlObj();

	//-- Synch control object
	void setTrigMode(TrigMode mode);
	void getTrigMode(TrigMode& mode);

	void setExpTime(double exp_time);
	void getExpTime(double& exp_time);

	void setLatTime(double lat_time);
	void getLatTime(double& lat_time);

	void getExposureTimeRange(double& min_expo, double& max_expo) const;
	void getLatTimeRange(double& min_lat, double& max_lat) const;

	void setNbFrames(int nb_frames);
	void getNbFrames(int& nb_frames);

	bool isAcqRunning() const;

        

	void setNCyclesUdpDelay(int nbcycles);
        void getNCyclesUdpDelay(int& nbcycles) ;



        pixiradDetector *m_pixirad;
        std::string m_hostname;
	int m_tcpPort;
	

	class AcqThread;

	AcqThread *m_acq_thread;
	TrigMode m_trigger_mode;
	double m_exp_time;
	ImageType m_image_type;
	int m_nb_frames; // nos of frames to acquire
	bool m_thread_running;
	bool m_wait_flag;
	bool m_quit;
	mutable Cond m_cond;
        

	// Buffer control object
	SoftBufferCtrlObj m_bufferCtrlObj;

        
        int res;
	
	char commandAnswerFromDetector[MAX_MSG_STR_LENGTH];
        // 

        // PIXIRAD SPECIFIC SETTERS AND GETTERS
        
        
        // Fake        
	enum FakeMode {FakeOFF, FakeON};
        void setFakeMode(FakeMode fake);
        void getFakeMode(FakeMode &fake) ;
        
        
        // Thresholds high / low  0 and 1; 
        void setHighThreshold0(float t);
        void getHighThreshold0(float& t) ;   
        
        void setLowThreshold0(float t);
        void getLowThreshold0(float& t) ;
        
        void setHighThreshold1(float t);
        void getHighThreshold1(float& t) ;
        
        void setLowThreshold1(float t);
        void getLowThreshold1(float& t) ;        
        
	
	
	
        // Dead time free mode
        enum DeadTimeFreeMode {DeadTimeFreeModeOFF,DeadTimeFreeModeON};
        void setDeadTimeFreeMode(DeadTimeFreeMode dtf);
        void getDeadTimeFreeMode(DeadTimeFreeMode &dtf);
        
        
        // SensorConfig NBI
        enum SensorConfigNBI  {SensorConfigNBI_OFF,SensorConfigNBI_ON};
        void setNbiMode(SensorConfigNBI nbi);
        void getNbiMode(SensorConfigNBI &nbi);
        
        
        // SensorConfig ASIC
        enum SensorConfigASIC {PII,PIII};
        void setAsicMode(SensorConfigASIC asic);
        void getAsicMode(SensorConfigASIC &asic);
        
        
        // SensorConfig Hybrid
        enum SensorConfigHybrid {CDTE,GAAS};
        void setHybridMode(SensorConfigHybrid hybrid);
        void getHybridMode(SensorConfigHybrid &hybrid);
        
        
        // SensorConfig Build
        enum SensorConfigBuild {PX1, PX2, PX8};
	void setSensorConfigBuild(SensorConfigBuild build);
	void getSensorConfigBuild(SensorConfigBuild &build);
        
        
        // Run Config Modes 
        enum RunConfigMode {DATA, COLLECT_FLAT, SUMMED_DATA, SUMMED_COLLECT_FLAT};
        void setRunConfigMode(RunConfigMode mode);
        void getRunConfigMode(RunConfigMode &mode);
        
        
        // CoolingTemperatureSetpoint
        void setCoolingTemperatureSetpoint(float t);
        void getCoolingTemperatureSetpoint(float& t) ;   
        
        // Cooling ON OFF 
        enum CoolingMode {COOLING_OFF, COOLING_ON};
        void setCoolingMode(CoolingMode mode);
        void getCoolingMode(CoolingMode &mode);
        
        
        // HighVoltage Biais
        void setHighVoltageBiais(float hv);
        void getHighVoltageBiais(float& hv) ;   
        
        
        // HighVoltage Mode
        enum HVBiaisPower {HV_OFF, HV_ON};
        void setHVBiasModePower(HVBiaisPower mode);
        void getHVBiasModePower(HVBiaisPower &mode);

        
        // HighVoltage Mode
        enum HVMode {AUTOHV, AUTOHV_LC, STDHV};
        void setHVBiasMode(HVMode mode);
        void getHVBiasMode(HVMode &mode);

	
	
	// Data transfer Modes
	enum TrsfMode {UNMODH, UNMOD, MOD};
	void setTrsfMode(TrsfMode mode);
	void getTrsfMode(TrsfMode &mode);
	
	
	
        // High voltage delays Before 
        void setHighVoltageDelayBeforeOn(float sec);
        void getHighVoltageDelayBeforeOn(float& sec) ;   
        
        
        // High voltage Reset after how many image 
        void setHVRefreshPeriod(int nbOfImages);
        void getHVRefreshPeriod(int& nbOfImages);   
        
        
        // Delay between frames during a serie in ms
        void setDelayBetweenFrames(int delayms);
        void getDelayBetweenFrames(int& delayms);   
        
        // Color mode for the detector
        enum ColorMode {COLMODE_1COL0, COLMODE_2COL, COLMODE_1COL1, COLMODE_DTF, COLMODE_4COL};
        void setColorMode(ColorMode color);
        void getColorMode(ColorMode &color);
        
        
        void getStatusCamera(HwInterface::StatusType::Basic & status);
	
	
	
	// Set_Sync variables
	enum SyncOutFunction {SHUTTER, RODONE, READ};
	void setSyncOutFunction(SyncOutFunction mode);
	void getSyncOutFunction(SyncOutFunction &mode);
	
	enum Polarity {POS, NEG};
	void setSyncOutPol(Polarity mode);
	void getSyncOutPol(Polarity &mode);
	
	void setSyncInPol(Polarity mode);
	void getSyncInPol(Polarity &mode);
	
	
	
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
	
	
	
	void sendCommandForDetectorCooling();
	void resetAllAlarms();
	
	void printMissingImageInfo();
	
	
	
// 	std::string temp_str = "Not initialised"; //  
	char temp_str[1000];
	
	HwDetInfoCtrlObj* m_InterfaceHwDetInfoCtrlObjForAutoConfig;
	
	
	
	
        
private:

	void readFrame(void *bptr, int frame_nb);
};


}; // namespace Pixirad
}; // namespace lima

#endif
