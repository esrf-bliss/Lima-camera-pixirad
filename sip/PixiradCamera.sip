//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2011
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

namespace Pixirad {

    class Camera {
%TypeHeaderCode
#include "PixiradCamera.h"
using namespace lima;
#include <string>
//#include <ostream>
//#include <map>
//#include "lima/HwBufferMgr.h"
//#include "lima/Debug.h"
//#include "lima/SizeUtils.h"
#include "pixiradDetector.h" //Yves
%End

    public:


        Camera(std::string hostname, int cmdPort = 6341);

//        enum DetectorStatus { // To be kept for getDetecorStatus
//            IDLE, ///< Detector idle
//            BUSY, ///< Detector Busy
//            STANDBY, ///< Detecotr in standby
//        };
//        //	enum ColourMode {Monochrome, Colour};
        //	enum Switch {OFF, ON};
        //	enum GainSetting {SLGM, LGM, HGM, SHGM};
        //	enum Counter {COUNTER0, COUNTER1, BOTH};
        //	enum Depth {BPP1=1, BPP6=6, BPP12=12, BPP24=24};
        //	enum Trigger {INTERNAL, RISING_EDGE_TTL, FALLING_EDGE_TTL, RISING_EDGE_LVDS, FALLING_EDGE_LVDS, SOFT};
        //	enum TriggerOutput {TTL, LVDS, TTL_DELAYED, LVDS_DELAYED, FOLLOW_SHUTTER, ONE_PER_ACQ_BURST,
        //			   SHUTTER_AND_SENSOR_READ, OUTPUT_BUSY};
        //	enum TriggerLevel {NORMAL, INVERTED};
        //	enum Threshold {THRESHOLD0, THRESHOLD1, THRESHOLD2, THRESHOLD3, THRESHOLD4,
        //		THRESHOLD5, THRESHOLD6, THRESHOLD7};

        void init();
        void reset();
        void prepareAcq();
        void startAcq();
        void stopAcq();
        
        
        // Fake        
	enum FakeMode {FakeOFF, FakeON};
        void setFakeMode(FakeMode fake);
        void getFakeMode(FakeMode &fake /Out/) ;
        
        
	// Cycles
	void setNCyclesUdpDelay(int nbcycles);
        void getNCyclesUdpDelay(int& nbcycles /Out/) ;


        // Thresholds high / low  0 and 1; 
        void setHighThreshold0(float t);
        void getHighThreshold0(float& t /Out/) ;   
        
        void setLowThreshold0(float t);
        void getLowThreshold0(float& t /Out/) ;
        
        void setHighThreshold1(float t);
        void getHighThreshold1(float& t /Out/) ;
        
        void setLowThreshold1(float t);
        void getLowThreshold1(float& t /Out/) ;        
        
        
        
        // Dead time free mode
        enum DeadTimeFreeMode {DeadTimeFreeModeOFF,DeadTimeFreeModeON};
        void setDeadTimeFreeMode(DeadTimeFreeMode dtf);
        void getDeadTimeFreeMode(DeadTimeFreeMode &dtf /Out/);
        
        
        // SensorConfig NBI
        enum SensorConfigNBI {SensorConfigNBI_OFF,SensorConfigNBI_ON};
        void setNbiMode(SensorConfigNBI nbi);
        void getNbiMode(SensorConfigNBI &nbi /Out/);
        
        //ok
        
        // SensorConfig ASIC
        enum SensorConfigASIC {PII,PIII};
        void setAsicMode(SensorConfigASIC asic);
        void getAsicMode(SensorConfigASIC &asic /Out/);
        
        
        // SensorConfig Hybrid
        enum SensorConfigHybrid {CDTE,GAAS};
        void setHybridMode(SensorConfigHybrid hybrid);
        void getHybridMode(SensorConfigHybrid &hybrid /Out/);
        
        
        // SensorConfig Build
        enum SensorConfigBuild {PX1, PX2, PX8};
	void setSensorConfigBuild(SensorConfigBuild build);
	void getSensorConfigBuild(SensorConfigBuild &build /Out/);
        
        
        // Run Config Modes 
        enum RunConfigMode {DATA, COLLECT_FLAT, SUMMED_DATA, SUMMED_COLLECT_FLAT};
        void setRunConfigMode(RunConfigMode mode);
        void getRunConfigMode(RunConfigMode &mode /Out/);
        
         //ok
        
        // CoolingTemperatureSetpoint
        void setCoolingTemperatureSetpoint(float t);
        void getCoolingTemperatureSetpoint(float& t /Out/) ;   
        
        
        
        // Cooling ON OFF 
        enum CoolingMode {COOLING_OFF, COOLING_ON};
        void setCoolingMode(CoolingMode mode);
        void getCoolingMode(CoolingMode &mode /Out/);
        
        //passe pas
  
        // HighVoltage Biais
        void setHighVoltageBiais(float hv);
        void getHighVoltageBiais(float& hv /Out/) ;   
//         
//         
        // HighVoltage Mode
        enum HVBiaisPower {HV_OFF, HV_ON};
        void setHVBiasModePower(HVBiaisPower mode);
        void getHVBiasModePower(HVBiaisPower &mode /Out/);

//         // Passe pas
        
	
	// HighVoltage Mode
	enum HVMode {AUTOHV, AUTOHV_LC, STDHV};
	void setHVBiasMode(HVMode mode);
	void getHVBiasMode(HVMode &mode /Out/);
	
	
	// Data transfer Modes
	enum TrsfMode {UNMODH, UNMOD, MOD};
	void setTrsfMode(TrsfMode mode);
	void getTrsfMode(TrsfMode &mode /Out/);


        // High voltage delays Before         
        void setHighVoltageDelayBeforeOn(float sec);
        void getHighVoltageDelayBeforeOn(float& sec /Out/) ;   
        
        
        
        // High voltage Reset after how many image 
        void setHVRefreshPeriod(int nbOfImages);
        void getHVRefreshPeriod(int& nbOfImages /Out/);   
        
        
        
        // Delay between frames during a serie in ms
        void setDelayBetweenFrames(int delayms);
        void getDelayBetweenFrames(int& delayms /Out/);   
        
        // Color mode for the detector
        enum ColorMode {COLMODE_1COL0, COLMODE_2COL, COLMODE_1COL1, COLMODE_DTF, COLMODE_4COL};
        void setColorMode(ColorMode color);
        void getColorMode(ColorMode &color /Out/);
        
	
	
	// Set_Sync variables
	enum SyncOutFunction {SHUTTER, RODONE, READ};
	void setSyncOutFunction(SyncOutFunction mode);
	void getSyncOutFunction(SyncOutFunction &mode /Out/);
	
	enum Polarity {POS, NEG};
	void setSyncOutPol(Polarity mode);
	void getSyncOutPol(Polarity &mode /Out/);
	
	void setSyncInPol(Polarity mode);
	void getSyncInPol(Polarity &mode /Out/);
	
	
	
	// Weather variable extracted from UDP stream, needs get/set
	void getTemperaturePeltierCold(float& information /Out/);
	void getTemperaturePeltierHot(float& information /Out/);
	void getHighVoltageTension(float& information /Out/);
	void getBoxHumidity(float& information /Out/);
	void getBoxTemperature(float& information /Out/);
	void getPeltierPower(float& information /Out/);
	
	void getAlarmTempTooHot(bool& information /Out/);
	void getAlarmTempTooHotEnabled(bool& information /Out/);
	void getAlarmTempTooCold(bool& information /Out/);
	void getAlarmTempTooColdEnabled(bool& information /Out/);
	void getAlarmHumidity(bool& information /Out/);
	void getAlarmHumidityEnabled(bool& information /Out/);
	
	
	
	void setSeedModeForDebugOnlyInOneChipWithPX8(bool saintRita);
	void getSeedModeForDebugOnlyInOneChipWithPX8(bool& saintRita /Out/);
	void setWhichModuleOutOfEightOnPX8(int module);
	void getWhichModuleOutOfEightOnPX8(int & module /Out/);

	
	void printMissingImageInfo();
	
	void sendCommandForDetectorCooling();
	void resetAllAlarms();
	
	

    };

}; // namespace Pixirad
