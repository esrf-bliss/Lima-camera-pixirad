
#include "PixiradCamera.h"

#include "PixiradReconstructionCtrlObj.h"

#include "processlib/LinkTask.h"
// #include "processlib/SoftRoi.h"
#include "processlib/ProcessExceptions.h"


#include "pixiradFpgaTools.h"


#include<cstdlib>
#include<cstring>

using namespace lima;
using namespace lima::Pixirad;


class _ReconstructionTask : public LinkTask
{
  DEB_CLASS_NAMESPC(DebModCamera, "_ReconstructionTask", "Pixirad");
public:
  _ReconstructionTask()
  {
    DEB_CONSTRUCTOR();
    
    
    DEB_TRACE()<< "Conversion Table construction" ;
    
  }
  
//     AT_InitialiseUtilityLibrary(); 
  
  
  ~_ReconstructionTask()
  {
    DEB_DESTRUCTOR();
  }
  
   void setBuffer( HwBufferCtrlObj* buffer_ctrl_obj) {
     DEB_MEMBER_FUNCT();
     m_BufferCtrlObjReconstructionTask= (SoftBufferCtrlObj *)buffer_ctrl_obj;
  };
  
  virtual Data process(Data&);
  
private:
//   Camera::SdkFrameDim m_sdk_frame_dim;
  SoftBufferCtrlObj* m_BufferCtrlObjReconstructionTask;
};

Data  _ReconstructionTask::process(Data& src)
{
  DEB_MEMBER_FUNCT();
  
  StdBufferCbMgr &bufferReconstructionTask = m_BufferCtrlObjReconstructionTask->getBuffer();
  
  int frame_number = src.frameNumber;
  
  DEB_TRACE()<< "Within the reconstruction task " << DEB_VAR2(frame_number, src);
  
  
  DEB_TRACE()<< "Memory allocation"; 
    
  // only a stupid copy for testing
  
     void *source = bufferReconstructionTask.getFrameBufferPtr(frame_number);
  
     void *destination = src.buffer->data;
  
     ushort *sourceAsInt = reinterpret_cast<ushort*>(source);     
     ushort *destinationAsInt = reinterpret_cast<ushort*>(destination);     
//      unsigned short *local_buffer_ptr
     
     
     
     DEB_TRACE()<< "FPGA Conversion"; // the hard way 
     
     
     //TODO: Adapt for all models
     int nbModules = 8;
     int colsPerDout = 32;
     int douts = 16;
     int pixieRows = 476;
     int pixieCols = 512;
     int codeDepth = 15;
     int matrix_dim_words = pixieRows*pixieCols;
     
     SENSOR Sens;
     Sens.Asic=PII;
     Sens.cols_per_dout = colsPerDout;
     Sens.rows  = pixieRows;
     Sens.matrix_size_pxls = matrix_dim_words;
     Sens.cols = pixieCols;
     Sens.dout = douts;
     
     
     
     Sens.conv_table.ptr=conversion_table_allocation(&Sens);
     
     
     Sens.conv_table.depth = 15;
     
     //ushort temporaryBufferLocal[8*476*512*15]; // TODO: change for one module. // This is local_buffer_ptr
     unsigned short *temporaryBufferLocal;
     temporaryBufferLocal=(unsigned short *)calloc(nbModules*pixieRows*pixieCols*codeDepth, sizeof(unsigned short));
     
     
     // First step bytes swapping      
     DEB_TRACE()<< "FPGA (1) Swapping"; 
     for(int i=0;i<nbModules;i++){
       for(int j=0;j<colsPerDout*pixieRows;j++){
	 for(int k=0;k<codeDepth;k++){
	   
	   my_bytes_swap(sourceAsInt+i+(j*nbModules*codeDepth)+(k*nbModules));
	   
	   temporaryBufferLocal[(i*colsPerDout*pixieRows*codeDepth)+(j*codeDepth)+k] = sourceAsInt[i+(j*nbModules*codeDepth)+(k*nbModules)];
	 }
       }
     }
     
//      DEB_TRACE()<< "CHECKING"; 
//      memcpy(destination, temporaryBufferLocal, 8*476*512);
     
     DEB_TRACE()<< "FPGA (2) Conversion"; 
     
     for(int i=0;i<nbModules;i++){
       for(int j=0;j<colsPerDout*pixieRows;j++){
 	 convert_bit_stream_to_counts(codeDepth, temporaryBufferLocal + (i*colsPerDout*pixieRows*codeDepth) + (j*codeDepth), destinationAsInt + (i*matrix_dim_words) + (j*douts), Sens, 0);
     }
    }
    
    
     
     DEB_TRACE()<< "FPGA (3) Decode + (4) Sort + (5) Map "; 

    for(int i=0;i<nbModules;i++){  
      decode_pixie_data_buffer( Sens.conv_table.ptr,  Sens.conv_table.depth, destinationAsInt+i*matrix_dim_words, Sens.matrix_size_pxls  );



      databuffer_sorting(destinationAsInt+i*matrix_dim_words,Sens);



      if (Sens.Asic==PII){
	map_data_buffer_on_pixie(destinationAsInt+i*matrix_dim_words,Sens);
      }
    }
    
    free(temporaryBufferLocal);
     
    
    DEB_ALWAYS()<< "Image processed" << DEB_VAR2(frame_number, src);
  return src;
}

//-----------------------------------------------------
// @brief Ctor
//-----------------------------------------------------
ReconstructionCtrlObj::ReconstructionCtrlObj(Camera& cam):
m_cam(cam)
{
  DEB_CONSTRUCTOR();
//   cam.m_pixirad->m_reconstructionBufferCtrlObj
  
  
  
  m_task = new _ReconstructionTask();
  m_task->setProcessingInPlace(true);
}

//-----------------------------------------------------
// @brief Dtor
//-----------------------------------------------------
ReconstructionCtrlObj::~ReconstructionCtrlObj()
{
  DEB_DESTRUCTOR();
  delete m_task;
}




void ReconstructionCtrlObj::prepareAcq()
{
  DEB_MEMBER_FUNCT();
  
  m_task->setBuffer(m_cam.m_pixirad->m_reconstructionBufferCtrlObj);
  
}

//-----------------------------------------------------
// @brief return the task if active otherwise return NULL
//-----------------------------------------------------
LinkTask* ReconstructionCtrlObj::getReconstructionTask()
{
  DEB_MEMBER_FUNCT();
  return m_task;
}



