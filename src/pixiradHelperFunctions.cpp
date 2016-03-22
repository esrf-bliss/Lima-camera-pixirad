
#include <iostream>
#include <string>
//#include <regex> // C++ Style
#include <regex.h> // C style

#include <math.h> // for "fabs"

#include <stdlib.h>     /* atof */

#include<cstdlib>

#include "lima/Exceptions.h"
#include "lima/Debug.h"

#include "pixiradHelperFunctions.h"



using namespace std;
using namespace lima;
using namespace lima::Pixirad;


double fractions[]=
{0,0.01,0.02,0.03,0.04,0.05,0.06,0.07,0.08,0.09,0.1,
  0.12,0.14,0.16,0.18,0.2,0.22,0.24,0.26,0.28,
  0.32,0.36,0.40,0.44,0.48,0.52,0.56,0.60,
  0.7,0.8,0.9,1.0};
  
  
  int lima::Pixirad::PIXIEIIThresholdCalculator(int VthmaxUpperLimit, double *requestedEnergy_ptr, int *VThMax_ptr, int *thresholdRegisters_ptr, double *actualEnergy_ptr){
    double DeltaVth;
    double EthAllowedTable[INT_DAC_STEPS];
    double temp_double;
    int i;
    int VThMax;
    double Vth,Vth1;
    /********************************getting Vth1*****************************************/
    Vth1=get_vth_from_fit(requestedEnergy_ptr[0]);
    /***************search for (VTHMAX, %), Vth1 Best Match*******************************/
    VThMax=VthmaxUpperLimit;
    do{
      i=0;
      do{
	Vth=get_Vth_from_settings(VThMax,fractions[i]);
	DeltaVth=fabs(Vth1-Vth);
	i++;
      }
      while(DeltaVth>VTH1_ACCURACY && i<INT_DAC_STEPS);
      VThMax-=VTHMAX_DECR_STEP;}
      while(DeltaVth>VTH1_ACCURACY && VThMax>=VTHMAX_LOWER_LIMIT);
      
      /**********************************Set VTHMAX, DACs and Energy Threshold for fisrt color*************************************/
      thresholdRegisters_ptr[0]=i-1;// DAC count is the fractions table index
      actualEnergy_ptr[0]=get_energy_from_fit(Vth);
      *VThMax_ptr=VThMax;
      /**********************Filling Allowed Energy set table********************************/
      for(i=0;i<INT_DAC_STEPS;i++){
	temp_double=get_Vth_from_settings(VThMax,fractions[i]);
	EthAllowedTable[i]=(get_energy_from_fit(temp_double));
      }
      /***********************************Set Internal DACs and Energy Thresholds for remainings colors***************************************************/
      for(i=1;i<PIXIEII_TH;i++){
	set_closest_Eth_DAC(EthAllowedTable,requestedEnergy_ptr[i],thresholdRegisters_ptr+i,actualEnergy_ptr+i);
      }
      return(1);
  }
  
  double lima::Pixirad::get_energy_from_fit(double Vth){
    //With this coefficients Only one solutions is positive and there is always one if EnergyKev>=0)
    return((A_COEFF* pow(Vth,2)+ B_COEFF*Vth));
  }
  double lima::Pixirad::get_Vth_from_settings(int VTHMAX,double fraction){
    return((VTHMAX*EXTDAC_LSB-0.6)*fraction);
  }
  
  int lima::Pixirad::set_closest_Eth_DAC(double * allowedEth_ptr,double Eth,int *DAC,double *EthSet){
    int i;
    double temp_double,DeltaEth;
    i=1;
    temp_double=fabs(allowedEth_ptr[0]-Eth);
    do{
      DeltaEth=fabs((allowedEth_ptr[i]-Eth));
      if(DeltaEth<=temp_double){
	temp_double=DeltaEth;
	*DAC=i;// i is the allowed energy table index and (i.e. DAC counts)
	*EthSet=allowedEth_ptr[i];
      }
      i++;
    }while(i<INT_DAC_STEPS);
    
    return(i);
  }
  
  
  
  double  lima::Pixirad::get_vth_from_fit(double EnergyKev){
    //With this coefficients Only one solutions is positive and there is always one if EnergyKev>=0)
    return(((-1.0)*B_COEFF+sqrt(pow(B_COEFF,2)+(4.0*A_COEFF*EnergyKev)))/(2.0*A_COEFF));
  }
  
  
  



  bool lima::Pixirad::findThisValueIn(regex_t regex, string whereToLook, float &result)
{
//   DEB_MEMBER_FUNCT();
  bool success = false;

  int reti;
  char msgbuf[100];
  
  
  
  size_t nmatch = 2;
  regmatch_t pmatch[2];
  
  /* Execute regular expression */
  reti = regexec(&regex, whereToLook.c_str(),nmatch, pmatch, 0);
  if (!reti) {
    
    int start = pmatch[1].rm_so;
    int end = pmatch[1].rm_eo;
    
//     fprintf(stderr, "start %i , end %i \n", start, end);
    
    std::string subChain = whereToLook.substr (start,end-start); 
    result = (float)::atof(subChain.c_str());   
    success = true;
  }
  else if (reti == REG_NOMATCH) {
//     puts("No match");
  }
  else {
    regerror(reti, &regex, msgbuf, sizeof(msgbuf));
//     fprintf(stderr, "Regex match failed: %s\n", msgbuf);
//     exit(1);
  }
  
  return success;
}
  
  
  
  
  
  
  /*
  //   regex_t regex;
  regex_t preg;
  float result = 9999.99;
  int err;
  
  err = regcomp (&preg, whatToLook.c_str(), REG_EXTENDED);
  
  if (err == 0)
  {
    int match;
    size_t nmatch = 0;
    regmatch_t *pmatch = NULL;
    
    nmatch = preg.re_nsub;
    pmatch = (regmatch_t*)malloc (sizeof (*pmatch) * nmatch);
    if (pmatch)
    {
      match = regexec (&preg, whereToLook.c_str(), nmatch, pmatch, 0);
      regfree (&preg);
      if (match == 0)
      {
	char *site = NULL;
	int start = pmatch[0].rm_so;
	int end = pmatch[0].rm_eo;
	size_t size = end - start;
	DEB_TRACE() << "Bundaries "<<DEB_VAR3(start , end, size); 
	
	site = (char*)malloc (sizeof (*site) * (size + 1));
	if (site)
	{
	  strncpy (site, &whereToLook.c_str()[start], size);
	  site[size] = '\0';
	  printf ("%s\n", site);
	  result = atof(site);
	  
	  DEB_TRACE() << "SUCESSS "<<DEB_VAR2(site, result); 
	  
	  free (site);
	}
      }
      else if (match == REG_NOMATCH)
      {
	DEB_TRACE () <<"Not found " << DEB_VAR2(whatToLook, whereToLook);
      }
    }
  }
  return result;
  */