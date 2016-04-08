
#include "PixiradCamera.h"

#include "PixiradReconstructionCtrlObj.h"

#include "processlib/LinkTask.h"
// #include "processlib/SoftRoi.h"
#include "processlib/ProcessExceptions.h"


#include "pixiradFpgaTools.h"
/*
#include <unistd.h> // for sleep // to be removed
#include <bitset> // for printing 16bits  needs to be removed
#include <iostream>*/

#include <fstream> // for .bin dump  to be removed


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
  
  void setNbModules(int nbmodules){
     DEB_MEMBER_FUNCT();
    m_nbmodules = nbmodules;
  }
  
  void setAllocationTable(unsigned short* table){
    DEB_MEMBER_FUNCT();
    conversion_table = table;
  }
  
  virtual Data process(Data&);
  
private:
//   Camera::SdkFrameDim m_sdk_frame_dim;
  SoftBufferCtrlObj* m_BufferCtrlObjReconstructionTask;
  int m_nbmodules;
  unsigned short * conversion_table;  
};

Data  _ReconstructionTask::process(Data& src)
{
  DEB_MEMBER_FUNCT();
  /*
     DEB_TRACE()<< "Mediterranean mode --- ";
     sleep(2);
     DEB_TRACE()<< " --- done";*/
  

  StdBufferCbMgr &bufferReconstructionTask = m_BufferCtrlObjReconstructionTask->getBuffer();
  
  
  int frame_number = src.frameNumber;
  
//   DEB_TRACE()<< "Within the reconstruction task " << DEB_VAR2(frame_number, src);
  
  
    
  // only a stupid copy for testing
  
//      DEB_TRACE()<< "Before get Frame Buffer";
     void *source = bufferReconstructionTask.getFrameBufferPtr(frame_number);
//      DEB_TRACE()<< "After get Frame Buffer";
  
     void *destination = src.buffer->data;
     
     //src.buffer->data = source;
 
     ushort *sourceAsInt = reinterpret_cast<ushort*>(source); 
     
//      ushort *destinationAsInt = reinterpret_cast<uint16_t*>(destination);   
     
     ushort *destinationAsInt = reinterpret_cast<ushort*>(destination);   
     
     
     
     DEB_TRACE()<< "FPGA Conversion " << DEB_VAR1(m_nbmodules); // the hard way 
     
     
     //TODO: Adapt for all models
//      int nbModules = 1;
//      if(m_cam.m_sensorConfigBuild == "PX8"){
//       nbModules = 8;
//       DEB_TRACE()<< "8 modules, more or less";
//      }
     int nbModules = m_nbmodules;   //TODO: change here
     int colsPerDout = 32;
     int douts = 16;
     int pixieRows = 476;
     int pixieCols = 512;
     int codeDepth = 15;
     
     
     int matrix_dim_words = pixieRows*pixieCols;
     
     
     
     
     char *sourceAsChar5 = reinterpret_cast<char*>(conversion_table); 
     std::ofstream b_stream5("/tmp/conv_table.bin", std::fstream::out | std::fstream::binary);
     b_stream5.write(sourceAsChar5, 32768*2); //PSTABLE_DEPTH is ushort
     b_stream5.close();
     
     
     //ushort temporaryBufferLocal[8*476*512*15]; // TODO: change for one module. // This is local_buffer_ptr
     unsigned short *temporaryBufferLocal;
      temporaryBufferLocal=(unsigned short *)calloc(nbModules*pixieRows*pixieCols, sizeof(unsigned short)); 
      
//       memset(&temporaryBufferLocal, '0', 2*nbModules*pixieRows*pixieCols); // not useful as we are doing calloc.
     
     
     char *sourceAsChar = reinterpret_cast<char*>(source); 
     std::ofstream b_stream("/tmp/source_1.bin", std::fstream::out | std::fstream::binary);
     b_stream.write(sourceAsChar, pixieRows*pixieCols*nbModules*2);
     b_stream.close();
     
     
     // First step bytes swapping      
     DEB_TRACE()<< "FPGA (1) Swapping "<< DEB_VAR4(nbModules, colsPerDout, pixieRows, codeDepth);
     for(int i=0;i<nbModules;i++){
       for(int j=0;j<colsPerDout*pixieRows;j++){
	 for(int k=0;k<codeDepth;k++){
	   
	   my_bytes_swap(sourceAsInt+i+(j*nbModules*codeDepth)+(k*nbModules)); // 
	   temporaryBufferLocal[(i*colsPerDout*pixieRows*codeDepth)+(j*codeDepth)+k] = sourceAsInt[i+(j*nbModules*codeDepth)+(k*nbModules)];
	   
	 }
       }
     }
     
     
     
     
     char *sourceAsChar2 = reinterpret_cast<char*>(temporaryBufferLocal); 
     std::ofstream b_stream2("/tmp/tempBufferLocal_2.bin", std::fstream::out | std::fstream::binary);
     b_stream2.write(sourceAsChar2, pixieRows*pixieCols*nbModules*2);
     b_stream2.close();
     
     
     
     
//      DEB_TRACE()<< "CHECKING"; 
//      memcpy(destination, temporaryBufferLocal, 8*476*512);
     DEB_TRACE()<< "FPGA (2) Conversion, bit stream to counts"; 
     
     for(int i=0;i<nbModules;i++){
       for(int j=0;j<colsPerDout*pixieRows;j++){
// 	  convert_bit_stream_to_counts(codeDepth, 
// 				       temporaryBufferLocal + (i*colsPerDout*pixieRows*codeDepth) + (j*codeDepth), 
// 				       destinationAsInt + (i*matrix_dim_words) + (j*douts), Sens, 0);
      // AD Style
	  convert_bit_stream_to_counts(codeDepth, 
				       temporaryBufferLocal + (i*colsPerDout*pixieRows*codeDepth) + (j*codeDepth), 
				       destinationAsInt + (i*matrix_dim_words) + (j*douts), douts);
	  
     }
    }
    /*
     char *sourceAsChar3 = reinterpret_cast<char*>(destinationAsInt); 
     std::ofstream b_stream3("/tmp/destinationAsInt_3a.bin", std::fstream::out | std::fstream::binary);
     b_stream3.write(sourceAsChar3, pixieRows*pixieCols*nbModules*2);
     b_stream3.close();
     */
    
     
     DEB_TRACE()<< "FPGA (3) Decode " << DEB_VAR3( conversion_table, matrix_dim_words, codeDepth);

    for(int i=0;i<nbModules;i++){  
      
//      The pseudo-random decoding must be temporarily disabled because the test pattern is natural binary coded. 
      decode_pixie_data_buffer( conversion_table,  
				codeDepth,
				destinationAsInt+i*matrix_dim_words,
				matrix_dim_words  );
    }
  /*    
     char *sourceAsChar6 = reinterpret_cast<char*>(destinationAsInt); 
     std::ofstream b_stream6("/tmp/destinationAsInt_3b.bin", std::fstream::out | std::fstream::binary);
     b_stream6.write(sourceAsChar6, pixieRows*pixieCols*nbModules*2);
     b_stream6.close();
  */   
      
     DEB_TRACE()<< "FPGA (4) Sort " << DEB_VAR2( conversion_table, matrix_dim_words);
      
    for(int i=0;i<nbModules;i++){ 
      databuffer_sorting(destinationAsInt+i*matrix_dim_words,
			 douts,
			 pixieRows,
			 colsPerDout,
			 matrix_dim_words
			);
    }
      
     DEB_TRACE()<< "FPGA  (5) Map " << DEB_VAR2( conversion_table, matrix_dim_words);
   /*   
     char *sourceAsChar7 = reinterpret_cast<char*>(destinationAsInt); 
     std::ofstream b_stream7("/tmp/destinationAsInt_3c.bin", std::fstream::out | std::fstream::binary);
     b_stream7.write(sourceAsChar7, pixieRows*pixieCols*nbModules*2);
     b_stream7.close();
    */ 
      
      
    for(int i=0;i<nbModules;i++){ 
	map_data_buffer_on_pixie(destinationAsInt+i*matrix_dim_words,
	  pixieRows,
	  pixieCols
	);
    }
    
   /* 
     char *sourceAsChar4 = reinterpret_cast<char*>(destinationAsInt); 
     std::ofstream b_stream4("/tmp/destinationAsInt_4.bin", std::fstream::out | std::fstream::binary);
     b_stream4.write(sourceAsChar4, pixieRows*pixieCols*nbModules*2);
     b_stream4.close();
     */
    
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
  m_task->setNbModules(m_cam.m_pixirad->m_nbModules);
  
  m_task->setAllocationTable(m_cam.m_pixirad->m_conversion_table);
  
     // transform the pixirad way to reuse Massimo swap function.
//      SENSOR Sens;
//      Sens.Asic=PII;
//      Sens.cols_per_dout = colsPerDout;
//      Sens.rows  = pixieRows;
//      Sens.matrix_size_pxls = matrix_dim_words;
//      Sens.cols = pixieCols;
//      Sens.dout = douts;     
//      Sens.conv_table.depth = codeDepth;
//      
     
  
}

//-----------------------------------------------------
// @brief return the task if active otherwise return NULL
//-----------------------------------------------------
LinkTask* ReconstructionCtrlObj::getReconstructionTask()
{
  DEB_MEMBER_FUNCT();
  return m_task;
}



