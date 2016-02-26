
#ifndef PIXIRADRECONSTRUCTIONCTRLOBJ_H
#define PIXIRADRECONSTRUCTIONCTRLOBJ_H

#include "lima/HwReconstructionCtrlObj.h"

#include "lima/Debug.h"

class _ReconstructionTask;


namespace lima
{
  namespace Pixirad
  {
    class Camera;
    
    class ReconstructionCtrlObj:  public HwReconstructionCtrlObj
    {
      DEB_CLASS_NAMESPC(DebModCamera, "PixiradReconstructionCtrlObj", "Pixirad");
      
    public:
      ReconstructionCtrlObj(lima::Pixirad::Camera& cam);
      ~ReconstructionCtrlObj();
      virtual LinkTask* getReconstructionTask();      
      void getActive(bool& active) const;
      void prepareAcq();
      
      
      
      
      
    private:
      Camera&		    m_cam;
      _ReconstructionTask*  m_task;
//       unsigned short * conversion_table_allocation(void);
//       void genera_tabella_clock(unsigned short *clocks, unsigned short dim, unsigned short counterwidth);
      //       bool                  m_active;
//       unsigned short *m_conversionTable;
    };
  }
}




#endif 
