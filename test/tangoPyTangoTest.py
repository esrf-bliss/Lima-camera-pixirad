import PyTango

pixi = PyTango.DeviceProxy("d05/pixirad/pixirad")
limaccd = PyTango.DeviceProxy("d05/pixirad/pixirad8")


def initialSetupForPX8():
  #doit = True
  #while(doit):
    #try:
      pixi.cooling_temperature_setpoint = -30
      pixi.high_voltage_biais = 400
      pixi.dead_time_free_mode = 'DEAD_TIME_FREE_MODE_ON'
      pixi.color_mode = 'COLMODE_1COL0'
      pixi.low_threshold0 =  0
      pixi.high_threshold0 = 60
      pixi.sensor_config_build = 'PX8'
      pixi.h_v_bias_mode_power = 1
      pixi.trsf_mode = "UNMOD"
      pixi.hybrid_mode = 'CDTE'
      doit = false
    #except Exception, e:
      #str(e)
      

def memAndCpu():
  limaccd.buffer_max_memory = 80
  #property : NbProcessingThread


### video_live + video_active => video 
### video_active seul => acquisition + retour video
### rien => acquisition seule

def videoTest():
  #limaccd.acq_nb_frames = 0
  limaccd.acq_expo_time = 0.3
  limaccd.video_live = 1
  ##limaccd.prepareAcq()
  ##limaccd.startAcq()




#camera.setHVBiasModePower(1)
#camera.setTrsfMode(camera.UNMOD)


#camera_interface.setHybridMode(camera.CDTE)

initialSetupForPX8()



