
//#include "lima/Debug.h"



#define	PSTABLE_DEPTH  32768
#define PSCNT_WIDTH  15


using namespace std;

namespace lima {
  namespace Pixirad {
    


/**************************************************************/
enum DETECTOR_BUILD{PX1,PX2,PX4,PX8};
enum ROSCHEMA_t {MONO0,MONO1, MONO2, MONO3, MONO4, MONO5, MONO6, MONO7, DEFAULT} ;
enum ASIC_TYPE{PII,PIII};
enum HYBRID_TYPE{CDTE,GAAS};
enum PIXEL_ARRANGEMENT{EXAGON,SQUARE};
typedef struct{
  unsigned short *ptr;
  int depth;
}CONV_TABLE;

typedef struct{
  DETECTOR_BUILD Build;
  ASIC_TYPE Asic;
  HYBRID_TYPE Hybrid;
  ROSCHEMA_t ReadoutSchema;
  int matrix_size_pxls;
  int modules;
  int	rows;
  int cols;
  int dout;
  int cols_per_dout;
  int autocal_bit_cnt;
  int autocal_regs;
  int cnt_regs;
  int bit_per_cnt_std;
  int bit_per_cnt_short;
  int bit_parity;
  PIXEL_ARRANGEMENT pixel_arr;
  CONV_TABLE conv_table;
}SENSOR;

// All functions needs to extract/convert/map fpgastream to image
void my_bytes_swap (unsigned short* us_ptr);

int convert_bit_stream_to_counts(int code_depth, unsigned short* source_memory_offset, unsigned short* destination_memory_offset, SENSOR Sens, int verbose);





void decode_pixie_data_buffer(unsigned short* table, int table_depth, unsigned short* databuffer,int databuffer_len);

int databuffer_sorting(unsigned short *buffer_a,SENSOR Sens);
int map_data_buffer_on_pixie(unsigned short *buffer_a, SENSOR Sens);
int map_data_buffer_on_pixieIII(unsigned short *buffer_a,SENSOR Sens);


void genera_tabella_clock(unsigned short *clocks, unsigned short dim, unsigned short counterwidth);

unsigned short * conversion_table_allocation(SENSOR* Sens_ptr);


unsigned short * PIIIConversion_table_allocation(int code_depth);

  } //namespace pixirad
} // namespace lima
