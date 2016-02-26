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
//
// PixiradInterface.h

#ifndef PIXIRADINTERFACE_H_
#define PIXIRADINTERFACE_H_

#include "lima/HwInterface.h"
#include "PixiradCamera.h"
#include "PixiradReconstructionCtrlObj.h"

#include "lima/Debug.h"

//using namespace std;

namespace lima {
namespace Pixirad {

class Interface;

/*******************************************************************
 * \class DetInfoCtrlObj
 * \brief Control object providing Pixirad detector info interface
 *******************************************************************/

class DetInfoCtrlObj: public HwDetInfoCtrlObj {
    
DEB_CLASS_NAMESPC(DebModCamera, "DetInfoCtrlObj","Pixirad");

public:
	DetInfoCtrlObj(Camera & cam);
	virtual ~DetInfoCtrlObj();

	virtual void getMaxImageSize(Size& max_image_size);
	virtual void getDetectorImageSize(Size& det_image_size);

	virtual void getDefImageType(ImageType& def_image_type);
	virtual void getCurrImageType(ImageType& curr_image_type);
	virtual void setCurrImageType(ImageType curr_image_type);

	virtual void getPixelSize(double& x_size, double &y_size);
	virtual void getDetectorType(std::string& det_type);
	virtual void getDetectorModel(std::string& det_model);

	virtual void registerMaxImageSizeCallback(HwMaxImageSizeCallback& cb);
	virtual void unregisterMaxImageSizeCallback(HwMaxImageSizeCallback& cb);

        
        
private:
	Camera& m_cam;
};

/*******************************************************************
 * \class SyncCtrlObj
 * \brief Control object providing Pixirad synchronization interface
 *******************************************************************/

class SyncCtrlObj: public HwSyncCtrlObj {
DEB_CLASS_NAMESPC(DebModCamera,"SyncCtrlObj","Pixirad");

public:
	SyncCtrlObj(Camera& cam); //, BufferCtrlObj& buffer);
	virtual ~SyncCtrlObj();

	virtual bool checkTrigMode(TrigMode trig_mode);
	virtual void setTrigMode(TrigMode trig_mode);
	virtual void getTrigMode(TrigMode& trig_mode);

	virtual void setExpTime(double exp_time);
	virtual void getExpTime(double& exp_time);

	virtual void setLatTime(double lat_time);
	virtual void getLatTime(double& lat_time);

	virtual void setNbHwFrames(int nb_frames);
	virtual void getNbHwFrames(int& nb_frames);

	virtual void getValidRanges(ValidRangesType& valid_ranges);

private:
	Camera& m_cam;
};
//
///*******************************************************************
// * \class Interface
// * \brief Pixirad hardware interface
// *******************************************************************/

class Interface: public HwInterface {
//DEB_CLASS_NAMESPC(DebModCamera, "Interface", "Pixirad");
DEB_CLASS_NAMESPC(DebModCamera, "Interface", "Pixirad");

public:
	Interface(Camera& cam);
	virtual ~Interface();
	virtual void getCapList(CapList&) const;
	virtual void reset(ResetLevel reset_level);
	virtual void prepareAcq();
	virtual void startAcq();
	virtual void stopAcq();
	virtual void getStatus(StatusType& status);
	virtual int getNbHwAcquiredFrames();

//	Camera& getCamera() {return m_cam;}
        /*
        void setFakeMode(Camera::FakeMode fake);
        void getFakeMode(Camera::FakeMode& fake);*/
        
        
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
	
	
	
	

	
private:
    Camera& m_cam;


	CapList m_cap_list;
	DetInfoCtrlObj m_det_info;
	HwBufferCtrlObj*  m_bufferCtrlObj;
	SyncCtrlObj m_sync;
	
	ReconstructionCtrlObj *m_reconstruction;
	
};

} // namespace Pixirad
} // namespace lima

#endif /* PIXIRADINTERFACE_H_ */
