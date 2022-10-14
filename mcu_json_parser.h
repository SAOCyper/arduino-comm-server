#ifndef __MCU_JSON_PARSER_H
#define __MCU_JSON_PARSER_H

uint8_t calc_chksum(uint8_t* buf, uint8_t len){
  uint8_t sum = 0;
  for(uint8_t i = 0;i<len;i++){
    sum = sum + buf[i];
  }
  return sum;
}


void mcu_comm_raw(uint8_t* datum){
  uint8_t i,data_array[64];

  data_array[0] = datum[0];
  data_array[1] = datum[1];
  
  if(data_array[0] == 0xFF & data_array[1] == 0xFE){
    data_array[2] = datum[2];
    for(i=0;i<data_array[2];i++){
      data_array[3+i] = datum[3+i]; 
    }
    //calculate the checksum of the data and put in the data array
    data_array[3+i] = calc_chksum(&data_array[3], data_array[2]);
    
    com2stm_PrintBuffer(data_array, 3+i+1); 
  } 
}

#endif
