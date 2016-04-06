


#include "pixiradFpgaTools.h"

#include<cstdlib>
#include<cstring>


#include "lima/Exceptions.h"
#include "lima/Debug.h"



using namespace std;
using namespace lima;
using namespace lima::Pixirad;




// Theses functions came from the pixirad team for decoding the raw 15b images from the fpga to 16bits unsigned  images.




void lima::Pixirad::my_bytes_swap (unsigned short* us_ptr){
  
  char a,b,*temp_char_ptr;
  temp_char_ptr=(char*)us_ptr;
  a=*temp_char_ptr;
  b=*(temp_char_ptr+1);
  *(temp_char_ptr+1)=a;
  *(temp_char_ptr)=b;  
}



// This one came from AreaDetector from APS.

void lima::Pixirad::convert_bit_stream_to_counts(int code_depth, unsigned short* source_memory_offset,
                                        unsigned short* destination_memory_offset, int douts)
{
  
//   DEB_STATIC_FUNCT();
//    DEB_TRACE()<<"convert_bit_stream_to_counts "<<DEB_VAR4(code_depth, source_memory_offset, destination_memory_offset, douts);
  
    int i,j;
    unsigned short dout_masks[16], mask_seed=1;
    for(i=0;i<douts;i++) dout_masks[i]=(mask_seed<<i);
    for(j=0;j<douts;j++){
        destination_memory_offset[j]=0;
        for(i=code_depth-1;i>=0;i--){
            if(source_memory_offset[i] & dout_masks[j])
                destination_memory_offset[j]|= dout_masks[code_depth-i-1];
            else
                destination_memory_offset[j]&= ~dout_masks[code_depth-i-1];
        }
    }
//     return j;
}

/*

int lima::Pixirad::convert_bit_stream_to_counts(int code_depth,unsigned short* source_memory_offset,
				 unsigned short* destination_memory_offset,SENSOR Sens,int verbose){
//    DEB_STATIC_FUNCT();
//    DEB_TRACE()<<"convert_bit_stream_to_counts "<<DEB_VAR4(code_depth, source_memory_offset, destination_memory_offset, Sens);

//   This method convert the 15bit stream to 16 bits counters
  
  int i,j;
  unsigned short dout_masks[Sens.dout],counter_masks[code_depth]
  ,dout_mask_seed,cnt_mask_seed;
  if(Sens.Asic==PIII){
    dout_mask_seed=0x0001;
    cnt_mask_seed=(0x0001 << (code_depth-1));//Be aware counter are 15 bits wide
    for(i=0;i<Sens.dout;i++) dout_masks[i]=(dout_mask_seed<<i);
    for(i=0;i<code_depth;i++) counter_masks[i]=(cnt_mask_seed>>i);
  }
  if(Sens.Asic==PII){
    dout_mask_seed=0x0001;
    cnt_mask_seed=(0x0001 << (code_depth-1));
    
    // Building array of masks, dout_masks being for 16 bit, counter_masks for 15 bits.
    // dout_masks [ 0000...001,   0000...010,   000...100,   ... ,  010...000,  ]
    // counter_masks [ 010...000 ,  001....00,  ...  , 000...001 ]
    
    for(i=0;i<Sens.dout;i++) dout_masks[i]=(dout_mask_seed<<i);
    for(i=0;i<code_depth;i++) counter_masks[i]=(cnt_mask_seed>>i);
  }
  
  for(j=0;j<Sens.dout;j++){
//     for the next 16 =>0
    destination_memory_offset[j]=0;
    
    for(i=0;i<code_depth;i++){
      
//       Compare source / dest for all 15 masks
      
      // If it has a 1 at i place in bit stream copy it 
      if(source_memory_offset[i] & dout_masks[j]){
	destination_memory_offset[j]|= counter_masks[i];
      }
      // If it has a 0 copy it too.
      else{
	destination_memory_offset[j]&= ~counter_masks[i];
      }
    }
  }

  return(j);
}
	*/			 


 void lima::Pixirad::decode_pixie_data_buffer(unsigned short* table, int table_depth, unsigned short* databuffer,int databuffer_len){
  int i;
  for(i=0;i<databuffer_len;i++)
    databuffer[i]=table[databuffer[i]%table_depth];
}
				 



//databuffer_sorting arranges data from fpga in an array in which 16 sectors (31200 pixel each) are contiguos in memory
int lima::Pixirad::databuffer_sorting(unsigned short *buffer_a,SENSOR Sens){
  int PIXELS_IN_SECTOR;
  unsigned short	*buffer_b;
  PIXELS_IN_SECTOR=(Sens.cols_per_dout * Sens.rows);
  int sector_cntr,pixel_cntr;
  
  
  
  buffer_b=(unsigned short*)calloc(Sens.matrix_size_pxls, sizeof(unsigned short));
  
  /*
  if (buffer_b==NULL) {
    printf("DATA sorting:Memory allocation unsuccesfull!! Please contact an expert :-)\n");
    return(0);}*/
    
    //in the original buffer same index(position in the sector) pixels are stored contiguosly
    //the filling starts from the end of each sector up to the beginning
    //this because the first data you get from fpga actually is the last pixel of the sector
    //you are reading out
    for(sector_cntr=0; sector_cntr<Sens.dout; sector_cntr++){
      for(pixel_cntr=0; pixel_cntr<PIXELS_IN_SECTOR; pixel_cntr++){
	//buffer_b[((sector_cntr+1)*PIXELS_IN_SECTOR_MAP)-pixel_cntr-1]=buffer_a[sector_cntr+pixel_cntr*PIXIE_DOUTS];
	buffer_b[((sector_cntr+1)*PIXELS_IN_SECTOR) - pixel_cntr - 1]=buffer_a[sector_cntr + pixel_cntr*Sens.dout];
	//					printf("databuffer sorting:px=\t%d sect=\t%d buffer_a=\t%d\n"
	//							,pixel_cntr
	//							,sector_cntr
	//							,buffer_a[sector_cntr + pixel_cntr*Sens.dout]
	//					        );
      }
    }
    
    //last sector hasn't as many pixels as others so filling for it, must start before than PIXEL_IN_SECTORS-1
    // it has to start from PIXELS_IN_LAST_SECTOR-1
    //at this point sector_cntr is already at 15
    //for(pixel_cntr=0;pixel_cntr<PIXELS_IN_LAST_SECTOR;pixel_cntr++){
    //		buffer_b[((sector_cntr)*PIXELS_IN_SECTOR_MAP+PIXELS_IN_LAST_SECTOR_MAP)-pixel_cntr-1]=buffer_a[sector_cntr+pixel_cntr*PIXIE_DOUTS];}
    
    //copying buffer to the original one and adding
    for(pixel_cntr=0; pixel_cntr<Sens.matrix_size_pxls; pixel_cntr++)
      buffer_a[pixel_cntr]=buffer_b[pixel_cntr];
    //			for(pixel_cntr=0; pixel_cntr<Sens.matrix_size_pxls; pixel_cntr++)
    //				printf("databuffer sorting:buffer_b[%d]=%d\n",pixel_cntr,buffer_b[pixel_cntr]);
    free(buffer_b);
    return(1);}
    
    
    
    
    
    
    

//map_data_buffer_on_pixie rearranges data in PIXIE layout taking in account the "snake" readout architecture
int lima::Pixirad::map_data_buffer_on_pixie(unsigned short *buffer_a, SENSOR Sens){
unsigned short* temp_col;
unsigned short col_cntr,row_cntr;
int PIXIE_ROWS;
/******************************/
PIXIE_ROWS=Sens.rows;
/******************************/
temp_col=(unsigned short*)calloc(PIXIE_ROWS, sizeof(unsigned short));
// if (temp_col==NULL) {
// printf("DATA mapping:Memory allocation unsuccesfull!! Please contact an expert :-)\n");
// return(-1);}
// if(Sens.Asic!=PII){
// if(verbose>=VERBOSITY_LOW)
// printf("Asked To map data on PII but Asic type is not PII\n");
// }
//sectors 0,2,4,6,8,10,12,14 start with the first column(800 pix) in the right dir
//sectors 1,3,5,7,8,11,13,15 start with the first column(800 pix) in the reversed dir
//in general even index columns has the right dir and odd ones are reversed
for(col_cntr=0;col_cntr<Sens.cols;col_cntr++){
if ((col_cntr%2)){//only odd index columns are reversed
for(row_cntr=0;row_cntr<Sens.rows;row_cntr++){
temp_col[Sens.rows-row_cntr-1]=buffer_a[col_cntr*Sens.rows+row_cntr];}
for(row_cntr=0;row_cntr<Sens.rows;row_cntr++){
  buffer_a[col_cntr*Sens.rows+row_cntr]=temp_col[row_cntr];}
}
}
//mirroring
//			for(col_cntr=0;col_cntr<Sens.cols/2;col_cntr++){
//								for(row_cntr=0;row_cntr<Sens.rows;row_cntr++){
//									temp_col[row_cntr]=buffer_a[col_cntr*Sens.rows+row_cntr];}
//								for(row_cntr=0;row_cntr<Sens.rows;row_cntr++){
//									buffer_a[col_cntr*Sens.rows+row_cntr]=buffer_a[(Sens.cols-col_cntr-1)*Sens.rows+row_cntr];}
//								for(row_cntr=0;row_cntr<Sens.rows;row_cntr++){
//									buffer_a[(Sens.cols-col_cntr-1)*Sens.rows+row_cntr]=temp_col[row_cntr];
//									}
//							}

free(temp_col);
return(1);}



int lima::Pixirad::map_data_buffer_on_pixieIII(unsigned short *buffer_a,SENSOR Sens){
unsigned short* temp_sector;
unsigned short col_cntr,row_cntr,sector;
int PIXIE_ROWS,COLS_PER_DOUT;
/***************************************/
PIXIE_ROWS=Sens.rows;
COLS_PER_DOUT=Sens.cols_per_dout;
/***************************************/
temp_sector=(unsigned short*)calloc(PIXIE_ROWS*COLS_PER_DOUT, sizeof(unsigned short));
// if (temp_sector==NULL) {
// printf("DATA mapping:Memory allocation unsuccesfull!! Please contact an expert :-)\n");
// return(0);}
// if(Sens.Asic!=PIII){
// printf("Asked to map data on PIII but Asic is not PIII\n");
// return(-1);
// }
//sectors 0,2,4,6,8,10,12,14 start with the first column(800 pix) in the right dir
//sectors 1,3,5,7,8,11,13,15 start with the first column(800 pix) in the reversed dir
//in general even index columns has the right dir and odd ones are reversed
for(sector=0;sector<Sens.dout;sector++){
for(row_cntr=0;row_cntr<Sens.rows;row_cntr++){
  for(col_cntr=0;col_cntr<Sens.cols_per_dout;col_cntr++){
    //temp_sector[row_cntr*COLS_PER_DOUT+col_cntr]=buffer_a[row_cntr+col_cntr*PIXIE_ROWS+sector*PIXIE_ROWS*COLS_PER_DOUT];
    temp_sector[row_cntr+(Sens.cols_per_dout-1-col_cntr)*Sens.rows]=buffer_a[row_cntr*Sens.cols_per_dout+col_cntr+sector*Sens.rows*Sens.cols_per_dout];
    //temp_sector[row_cntr*COLS_PER_DOUT+col_cntr]=buffer_a[row_cntr*COLS_PER_DOUT+col_cntr+sector*PIXIE_ROWS*COLS_PER_DOUT];
  }
}
memcpy(buffer_a+Sens.rows*Sens.cols_per_dout*sector,temp_sector,Sens.cols_per_dout*Sens.rows*sizeof(unsigned short));
}
//mirroring
//			for(col_cntr=0;col_cntr<PIXIE_COLS/2;col_cntr++){
//								for(row_cntr=0;row_cntr<PIXIE_ROWS;row_cntr++){
//									temp_col[row_cntr]=buffer_a[col_cntr*PIXIE_ROWS+row_cntr];}
//								for(row_cntr=0;row_cntr<PIXIE_ROWS;row_cntr++){
//									buffer_a[col_cntr*PIXIE_ROWS+row_cntr]=buffer_a[(PIXIE_COLS-col_cntr-1)*PIXIE_ROWS+row_cntr];}
//								for(row_cntr=0;row_cntr<PIXIE_ROWS;row_cntr++){
//									buffer_a[(PIXIE_COLS-col_cntr-1)*PIXIE_ROWS+row_cntr]=temp_col[row_cntr];
//									}
//							}

free(temp_sector);
return(1);}










// this routine has been developed by SAndro, abd generates the lookup table
// for pixie counter conversion
void lima::Pixirad::genera_tabella_clock(unsigned short *clocks, unsigned short dim, unsigned short counterwidth){
  //unsigned int clocks[32768];
  unsigned long potenze[16], bit1,bit2;
  unsigned long i,tempo;
  
  
  potenze[0]=1;
  for (i=1; i<(unsigned long)counterwidth+1; i++)
    potenze[i]=potenze[i-1]*2;
  
  clocks[0]=0;
  tempo=0;
  for(i=1; i<dim; i++){
    
    bit1=tempo&potenze[14];
    if(bit1 != 0)bit1=1;
    bit2=tempo&potenze[6];
    if(bit2 != 0)bit2=1;
    bit1=!(bit1^bit2);
    tempo=tempo*2+bit1;
    tempo=tempo%potenze[15];
    clocks[tempo]=i;
    
  }
  clocks[0]=0;
  return;
}



unsigned short * lima::Pixirad::conversion_table_allocation(SENSOR* Sens_ptr){
  unsigned short *ush_ptr;
  int i;
  if((*Sens_ptr).Asic==PII){
    (*Sens_ptr).conv_table.depth=PSTABLE_DEPTH;
    ush_ptr=(unsigned short*)calloc(PSTABLE_DEPTH, sizeof(unsigned short));
    
    
     memset(ush_ptr, 0, PSTABLE_DEPTH*sizeof(unsigned short));
    
    (*Sens_ptr).conv_table.ptr=ush_ptr;
	genera_tabella_clock(ush_ptr,PSTABLE_DEPTH,PSCNT_WIDTH);
	return(ush_ptr);
  }
  else return(NULL);// PIII NOT SUPPORTED NOW
  
  
    
//     if((*Sens_ptr).Asic==PIII){
//       (*Sens_ptr).conv_table.depth=PSTABLE_DEPTH;
//        ush_ptr=PIIIConversion_table_allocation((*Sens_ptr).bit_per_cnt_std);
//       (*Sens_ptr).conv_table.ptr=ush_ptr;
//       return(ush_ptr);
//     }

}



/*
unsigned short * PIIIConversion_table_allocation(int code_depth){
  unsigned short *ush_ptr;
  int ret;
  ush_ptr=(unsigned short*)calloc(CONVERSIONTABLEDEPTH, sizeof(unsigned short));

      ret=GeneratePIIIConversionTable(ush_ptr,CONVERSIONTABLEDEPTH, code_depth);
      
      //the following function reverses the table generated by brez
      ret=InvertPIIIConversionTable(ush_ptr,CONVERSIONTABLEDEPTH,code_depth);
      if(ret!=0) printf("Table inverted returned with errors (%d)\n",ret);
      return(ush_ptr);}
}
*/











