#include <regex.h>

using namespace std;

namespace lima {
  namespace Pixirad {
    
    #define MAXIMUM_REQ_EN_KEV 150.0
    #define PIXIEII_TH 4
    #define  B_COEFF	39.3
    #define  A_COEFF	36.6
    #define  EXTDAC_LSB 781.0e-6
    #define  VAGND	0.6
    #define  VTHMAX_UPPER_LIMIT	2200
    #define  VTHMAX_LOWER_LIMIT	1000
    #define  VTHMAX_DECR_STEP 1
    #define  VTHMAX_MAX_ITERATIONS 2000
    #define	 VTH1_ACCURACY 1.0e-3
    #define  INT_DAC_STEPS	32
    
    bool findThisValueIn(regex_t regex, string whereToLook, float &result, Cond m_cond_regexExtract);
    
//       Cond m_cond;

      
      int PIXIEIIThresholdCalculator(int VthmaxUpperLimit, double *requestedEnergy_ptr, int *VThMax_ptr, int *thresholdRegisters_ptr, double *actualEnergy_ptr);
      
      double get_energy_from_fit(double Vth);
      double get_Vth_from_settings(int VTHMAX,double fraction);
      int set_closest_Eth_DAC(double * allowedEth_ptr,double Eth,int *DAC,double *EthSet);
      
      double get_vth_from_fit(double EnergyKev);

  }
}
