//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2019
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

class Interface : public HwInterface  /NoDefaultCtors/
{
%TypeHeaderCode
#include "PixiradInterface.h"
#include "pixiradDetector.h"
// #include "PixiradReconstructionCtrlObj.h"
using namespace lima;
%End


public:
	Interface(Pixirad::Camera& cam /KeepReference/);
//	Interface(Pixirad::Camera& cam);
//	virtual ~Interface();
        virtual void getCapList(std::vector<HwCap> &cap_list /Out/) const;
	virtual void reset(ResetLevel reset_level);
	virtual void prepareAcq();
	virtual void startAcq();
	virtual void stopAcq();
	//        virtual void getStatus(StatusType& status /Out/);
	virtual void getStatus(HwInterface::StatusType& status /Out/);
	
	virtual int getNbHwAcquiredFrames();
        /*
        //// PIXIRAD SPECIFIC //////
        void setFakeMode(Pixirad::Camera::FakeMode fake);
        void getFakeMode(Pixirad::Camera::FakeMode& fake /Out/);*/
        
        
   // Cycles setNCyclesUdpDelay
	void setNCyclesUdpDelay(int nbcycles);
        void getNCyclesUdpDelay(int& nbcycles /Out/) ;

        void setHighThreshold0(float t);
        void getHighThreshold0(float& t /Out/) ;
        void setLowThreshold0(float t);
        void getLowThreshold0(float& t /Out/) ;
        void setHighThreshold1(float t);
        void getHighThreshold1(float& t /Out/) ;
        void setLowThreshold1(float t);
        void getLowThreshold1(float& t /Out/) ;
        void setDeadTimeFreeMode(Pixirad::Camera::DeadTimeFreeMode dtf) ;
        void getDeadTimeFreeMode(Pixirad::Camera::DeadTimeFreeMode &dtf /Out/) ;
        void setNbiMode(Pixirad::Camera::SensorConfigNBI nbi) ;
        void getNbiMode(Pixirad::Camera::SensorConfigNBI &nbi /Out/) ;
        void setAsicMode(Pixirad::Camera::SensorConfigASIC asic);
        void getAsicMode(Pixirad::Camera::SensorConfigASIC &asic /Out/);
        void setHybridMode(Pixirad::Camera::SensorConfigHybrid hybrid);
        void getHybridMode(Pixirad::Camera::SensorConfigHybrid &hybrid /Out/);
	void setSensorConfigBuild(Pixirad::Camera::SensorConfigBuild build);
	void getSensorConfigBuild(Pixirad::Camera::SensorConfigBuild &build /Out/);
        void setRunConfigMode(Pixirad::Camera::RunConfigMode mode);
        void getRunConfigMode(Pixirad::Camera::RunConfigMode &mode /Out/);
        void setCoolingTemperatureSetpoint(float t);
        void getCoolingTemperatureSetpoint(float& t /Out/) ;
        void setCoolingMode(Pixirad::Camera::CoolingMode mode);
        void getCoolingMode(Pixirad::Camera::CoolingMode &mode /Out/);
        void setHighVoltageBiais(float hv);
        void getHighVoltageBiais(float& hv /Out/) ;
        void setHVBiasModePower(Pixirad::Camera::HVBiaisPower mode);
        void getHVBiasModePower(Pixirad::Camera::HVBiaisPower &mode /Out/);
        void setHVBiasMode(Pixirad::Camera::HVMode mode);
        void getHVBiasMode(Pixirad::Camera::HVMode &mode /Out/);
        void setHighVoltageDelayBeforeOn(float sec);
        void getHighVoltageDelayBeforeOn(float& sec /Out/) ;
        void setHVRefreshPeriod(int nbOfImages);
        void getHVRefreshPeriod(int& nbOfImages /Out/);
        void setDelayBetweenFrames(int delayms);
        void getDelayBetweenFrames(int& delayms /Out/);
        void setColorMode(Pixirad::Camera::ColorMode color);
	void getColorMode(Pixirad::Camera::ColorMode &color /Out/);
	
	
	void setSyncOutFunction(Pixirad::Camera::SyncOutFunction mode);
	void getSyncOutFunction(Pixirad::Camera::SyncOutFunction &mode /Out/);
	
	void setSyncOutPol(Pixirad::Camera::Polarity mode);
	void getSyncOutPol(Pixirad::Camera::Polarity &mode /Out/);
	
	void setSyncInPol(Pixirad::Camera::Polarity mode);
	void getSyncInPol(Pixirad::Camera::Polarity &mode /Out/);

	
	void setTrsfMode(Pixirad::Camera::TrsfMode mode);
	void getTrsfMode(Pixirad::Camera::TrsfMode &mode /Out/);
	
	// Weather variable extracted from UDP stream, needs get/set
	void getTemperaturePeltierCold(float& information /Out/ );
	void getTemperaturePeltierHot(float& information /Out/ );
	void getHighVoltageTension(float& information /Out/ );
	void getBoxHumidity(float& information /Out/ );
	void getBoxTemperature(float& information /Out/ );
	void getPeltierPower(float& information /Out/ );
	
	void getAlarmTempTooHot(bool& information /Out/ );
	void getAlarmTempTooHotEnabled(bool& information /Out/ );
	void getAlarmTempTooCold(bool& information /Out/ );
	void getAlarmTempTooColdEnabled(bool& information /Out/ );
	void getAlarmHumidity(bool& information /Out/ );
	void getAlarmHumidityEnabled(bool& information /Out/ );
	
	void autocalibration();
   private:
        Interface(const Pixirad::Interface&);

};

}; // namespace Pixirad
