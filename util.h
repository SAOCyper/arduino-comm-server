#ifndef __UTIL_H
#define __UTIL_H

void comPrintBuffer(uint8_t *data, size_t len)
{
    size_t i = 0;

    for(i=0;i<len;i++) Serial.print((char)data[i]);
    Serial.println();
}

void com2stm_PrintBuffer(uint8_t *data, size_t len)
{
    size_t i = 0;
    for(i=0;i<len;i++) Serial2.print((char)data[i]);

    /*for(i=0;i<len;i++) {
      Serial.print(data[i]);
      Serial.print(",");
    }*/
}
#endif
