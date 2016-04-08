//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2014
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

#include "PixiradInterface.h"

//#include "pixiradDetector.h"

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION  // Silly: warning "Using deprecated NumPy API, disable it by " 

using namespace lima;
using namespace lima::Pixirad;

Interface::Interface(Camera& cam) : m_cam(cam), m_det_info(cam), m_sync(cam)
{
	DEB_CONSTRUCTOR();
	HwDetInfoCtrlObj *det_info = &m_det_info;
	m_cap_list.push_back(det_info);

	// To be used by final detector class for auto updating capabilities
// 	cam.m_InterfaceHwDetInfoCtrlObjForAutoConfig = det_info;
	
	// Buffer Ctrl Object is made in pixiradDetector.cpp
	// It is received and declarded in the capability list here.
// 	m_bufferCtrlObj = m_cam.getBufferCtrlObj();
// 	HwBufferCtrlObj *buffer = m_cam.getBufferCtrlObj();
	SoftBufferCtrlObj *buffer = m_cam.getBufferCtrlObj();
	m_cap_list.push_back(buffer);
        
        

	HwSyncCtrlObj *sync = &m_sync;
	m_cap_list.push_back(sync);

	
	
	m_reconstruction = new ReconstructionCtrlObj(m_cam);
	m_cap_list.push_back(HwCap(m_reconstruction));
}





Interface::~Interface() {
  DEB_DESTRUCTOR();
//   delete m_reconstruction;
//   delete m_sync;
  
}
//
void Interface::getCapList(CapList &cap_list) const {
	DEB_MEMBER_FUNCT();
	cap_list = m_cap_list;
}

void Interface::reset(ResetLevel reset_level) {
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(reset_level);
//
//	stopAcq();
//	m_cam.reset();
}

void Interface::autocalibration(){
  DEB_MEMBER_FUNCT();
  
  m_cam.autocalibration();
  
}


void Interface::prepareAcq() {
	DEB_MEMBER_FUNCT();
	
	m_cam.prepareAcq();
	
	m_reconstruction->prepareAcq();
	
}

void Interface::startAcq() {
	DEB_MEMBER_FUNCT();
	m_cam.startAcq();
}

void Interface::stopAcq() {
	DEB_MEMBER_FUNCT();
	m_cam.stopAcq();
}

void Interface::getStatus(StatusType& status) {
	DEB_MEMBER_FUNCT();
        
        HwInterface::StatusType::Basic basic;
        
        m_cam.getStatusCamera(basic);
        
        status.set(basic);
        
        DEB_TRACE() << "Access to status " << DEB_VAR2(basic,status);
        
}

int Interface::getNbHwAcquiredFrames() {
	DEB_MEMBER_FUNCT();
//	return m_cam.getNbHwAcquiredFrames();
	return 4;
}


/// Pour call back on size change

void DetInfoCtrlObj::registerMaxImageSizeCallback(
					HwMaxImageSizeCallback& cb)
{
	DEB_MEMBER_FUNCT();
// 	m_cam.registerMaxImageSizeCallback(cb);
}

void DetInfoCtrlObj::unregisterMaxImageSizeCallback(
					HwMaxImageSizeCallback& cb)
{
	DEB_MEMBER_FUNCT();
// 	m_cam.unregisterMaxImageSizeCallback(cb);
}



///////////////////////     SPECIFIC PIXIRAD     ///////////////////////



// FAKE //
/*
void Interface::setFakeMode(Camera::FakeMode fake) {
	DEB_MEMBER_FUNCT();
        m_cam.setFakeMode(fake);
}

void Interface::getFakeMode(Camera::FakeMode& fake) {
	DEB_MEMBER_FUNCT();        
        m_cam.getFakeMode(fake);
}*/


// UDP           setNCyclesUdpDelay
void  Interface::setNCyclesUdpDelay(int nbcycles){
	DEB_MEMBER_FUNCT();        
        m_cam.setNCyclesUdpDelay(nbcycles);
}

void  Interface::getNCyclesUdpDelay(int& nbcycles){
	DEB_MEMBER_FUNCT();        
        m_cam.getNCyclesUdpDelay(nbcycles);
}



void Interface::setHighThreshold0(float t){
	DEB_MEMBER_FUNCT();        
        m_cam.setHighThreshold0(t);
}

void Interface::getHighThreshold0(float& t) {
	DEB_MEMBER_FUNCT();        
        m_cam.getHighThreshold0(t);
}

void Interface::setLowThreshold0(float t){
	DEB_MEMBER_FUNCT();        
        m_cam.setLowThreshold0(t);
}

void Interface::getLowThreshold0(float& t) {
	DEB_MEMBER_FUNCT();        
        m_cam.getLowThreshold0(t);
}

void Interface::setHighThreshold1(float t){
	DEB_MEMBER_FUNCT();        
        m_cam.setHighThreshold1(t);
}

void Interface::getHighThreshold1(float& t) {
	DEB_MEMBER_FUNCT();        
        m_cam.getHighThreshold1(t);
}

void Interface::setLowThreshold1(float t){
	DEB_MEMBER_FUNCT();        
        m_cam.setLowThreshold1(t);
}

void Interface::getLowThreshold1(float& t) {
	DEB_MEMBER_FUNCT();        
        m_cam.getLowThreshold1(t);
}

void Interface::setDeadTimeFreeMode(Camera::DeadTimeFreeMode dtf) {
	DEB_MEMBER_FUNCT();        
        m_cam.setDeadTimeFreeMode(dtf);
}

void Interface::getDeadTimeFreeMode(Camera::DeadTimeFreeMode &dtf) {
	DEB_MEMBER_FUNCT();        
        m_cam.getDeadTimeFreeMode(dtf);
}

void Interface::setNbiMode(Camera::SensorConfigNBI nbi) {
	DEB_MEMBER_FUNCT();        
        m_cam.setNbiMode(nbi);
}

void Interface::getNbiMode(Camera::SensorConfigNBI &nbi) {
	DEB_MEMBER_FUNCT();        
        m_cam.getNbiMode(nbi);
}

void Interface::setAsicMode(Camera::SensorConfigASIC asic){
	DEB_MEMBER_FUNCT();        
        m_cam.setAsicMode(asic);
}

void Interface::getAsicMode(Camera::SensorConfigASIC &asic){
	DEB_MEMBER_FUNCT();        
        m_cam.getAsicMode(asic);
}

void Interface::setHybridMode(Camera::SensorConfigHybrid hybrid){
	DEB_MEMBER_FUNCT();        
        m_cam.setHybridMode(hybrid);
}

void Interface::getHybridMode(Camera::SensorConfigHybrid &hybrid){
	DEB_MEMBER_FUNCT();        
        m_cam.getHybridMode(hybrid);
}

void Interface::setSensorConfigBuild(Camera::SensorConfigBuild build){
	DEB_MEMBER_FUNCT();        
	m_cam.setSensorConfigBuild(build);
}

void Interface::getSensorConfigBuild(Camera::SensorConfigBuild &build){
	DEB_MEMBER_FUNCT();        
	m_cam.getSensorConfigBuild(build);
}

void Interface::setRunConfigMode(Camera::RunConfigMode mode){
	DEB_MEMBER_FUNCT();        
        m_cam.setRunConfigMode(mode);
}

void Interface::getRunConfigMode(Camera::RunConfigMode &mode){
	DEB_MEMBER_FUNCT();        
        m_cam.getRunConfigMode(mode);
}

void Interface::setCoolingTemperatureSetpoint(float t){
	DEB_MEMBER_FUNCT();        
        m_cam.setCoolingTemperatureSetpoint(t);
	
	m_cam.sendCommandForDetectorCooling();
}

void Interface::getCoolingTemperatureSetpoint(float& t) {
	DEB_MEMBER_FUNCT();        
        m_cam.getCoolingTemperatureSetpoint(t);
}

void Interface::setCoolingMode(Camera::CoolingMode mode){
	DEB_MEMBER_FUNCT();        
	m_cam.setCoolingMode(mode);
	m_cam.sendCommandForDetectorCooling();
}

void Interface::getCoolingMode(Camera::CoolingMode &mode){
	DEB_MEMBER_FUNCT();        
        m_cam.getCoolingMode(mode);
}

void Interface::setHighVoltageBiais(float hv){
	DEB_MEMBER_FUNCT();        
	m_cam.setHighVoltageBiais(hv);
	m_cam.sendCommandForDetectorCooling();
}

void Interface::getHighVoltageBiais(float& hv) {
	DEB_MEMBER_FUNCT();        
        m_cam.getHighVoltageBiais(hv);
}

void Interface::setHVBiasModePower(Camera::HVBiaisPower mode){
	DEB_MEMBER_FUNCT();        
	m_cam.setHVBiasModePower(mode);
	m_cam.sendCommandForDetectorCooling();
}

void Interface::getHVBiasModePower(Camera::HVBiaisPower &mode){
	DEB_MEMBER_FUNCT();        
        m_cam.getHVBiasModePower(mode);
}

void Interface::setHVBiasMode(Camera::HVMode mode){
	DEB_MEMBER_FUNCT();        
        m_cam.setHVBiasMode(mode);
}

void Interface::getHVBiasMode(Camera::HVMode &mode){
	DEB_MEMBER_FUNCT();        
        m_cam.getHVBiasMode(mode);
}

void Interface::setHighVoltageDelayBeforeOn(float sec){
	DEB_MEMBER_FUNCT();        
        m_cam.setHighVoltageDelayBeforeOn(sec);
}

void Interface::getHighVoltageDelayBeforeOn(float& sec) {
	DEB_MEMBER_FUNCT();        
        m_cam.getHighVoltageDelayBeforeOn(sec);
}

void Interface::setHVRefreshPeriod(int nbOfImages){
	DEB_MEMBER_FUNCT();        
        m_cam.setHVRefreshPeriod(nbOfImages);
}

void Interface::getHVRefreshPeriod(int& nbOfImages){
	DEB_MEMBER_FUNCT();        
        m_cam.getHVRefreshPeriod(nbOfImages);
}

void Interface::setDelayBetweenFrames(int delayms){
	DEB_MEMBER_FUNCT();        
        m_cam.setDelayBetweenFrames(delayms);
}

void Interface::getDelayBetweenFrames(int& delayms){
	DEB_MEMBER_FUNCT();        
        m_cam.getDelayBetweenFrames(delayms);
}

void Interface::setColorMode(Camera::ColorMode color){
	DEB_MEMBER_FUNCT();        
        m_cam.setColorMode(color);
}

void Interface::getColorMode(Camera::ColorMode &color){
	DEB_MEMBER_FUNCT();        
        m_cam.getColorMode(color);
}



void Interface::setTrsfMode(Camera::TrsfMode mode){
  DEB_MEMBER_FUNCT();        
  m_cam.setTrsfMode(mode);
}

void Interface::getTrsfMode(Camera::TrsfMode &mode){
  DEB_MEMBER_FUNCT();        
  m_cam.getTrsfMode(mode);
}




void Interface::setSyncOutFunction(Camera::SyncOutFunction mode){
  DEB_MEMBER_FUNCT();        
  m_cam.setSyncOutFunction(mode);
}

void Interface::getSyncOutFunction(Camera::SyncOutFunction &mode){
  DEB_MEMBER_FUNCT();        
  m_cam.getSyncOutFunction(mode);
}

void Interface::setSyncOutPol(Camera::Polarity mode){
  DEB_MEMBER_FUNCT();
  m_cam.setSyncOutPol(mode);
}

void Interface::getSyncOutPol(Camera::Polarity &mode){
  DEB_MEMBER_FUNCT();        
  m_cam.getSyncOutPol(mode);
}

void Interface::setSyncInPol(Camera::Polarity mode){
  DEB_MEMBER_FUNCT();        
  m_cam.setSyncInPol(mode);
}

void Interface::getSyncInPol(Camera::Polarity &mode){
  DEB_MEMBER_FUNCT();        
  m_cam.getSyncInPol(mode);
}


void Interface::getTemperaturePeltierCold(float & information){
  DEB_MEMBER_FUNCT();        
  m_cam.getTemperaturePeltierCold(information);  
}

void Interface::getTemperaturePeltierHot(float& information){
  DEB_MEMBER_FUNCT();        
  m_cam.getTemperaturePeltierHot(information);  
}

void Interface::getHighVoltageTension(float& information){
  DEB_MEMBER_FUNCT();          
  m_cam.getHighVoltageTension(information);
}

void Interface::getBoxHumidity(float& information){
  DEB_MEMBER_FUNCT();               
  m_cam.getBoxHumidity(information);  
}

void Interface::getBoxTemperature(float& information){
  DEB_MEMBER_FUNCT();               
  m_cam.getBoxTemperature(information);  
}

void Interface::getPeltierPower(float& information){
  DEB_MEMBER_FUNCT();               
  m_cam.getPeltierPower(information);  
}

void Interface::getAlarmTempTooHot(bool& information){
  DEB_MEMBER_FUNCT();               
  m_cam.getAlarmTempTooHot(information);  
}

void Interface::getAlarmTempTooHotEnabled(bool& information){
  DEB_MEMBER_FUNCT();               
  m_cam.getAlarmTempTooHotEnabled(information);  
}

void Interface::getAlarmTempTooCold(bool& information){
  DEB_MEMBER_FUNCT();               
  m_cam.getAlarmTempTooCold(information);  
}

void Interface::getAlarmTempTooColdEnabled(bool& information){
  DEB_MEMBER_FUNCT();          
  m_cam.getAlarmTempTooColdEnabled(information);
}

void Interface::getAlarmHumidity(bool& information){
  DEB_MEMBER_FUNCT();          
  m_cam.getAlarmHumidity(information);
}

void Interface::getAlarmHumidityEnabled(bool& information){
  DEB_MEMBER_FUNCT();               
  m_cam.getAlarmHumidityEnabled(information);  
}













