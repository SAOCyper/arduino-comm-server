#ifndef _JSONPARSERS_H_
#define _JSONPARSERS_H_

#include "esp_json_parser.h"
#include "mcu_json_parser.h"

bool parseJSONData(DynamicJsonDocument doc){
  bool ret = false;
  const char* target = doc["target"];

  if(strcmp(target, "esp") == 0){
    ret = esp_json_parser(doc);
  }

  else if(strcmp(target, "mcu") == 0){
    ret = mcu_json_parser(doc);
  }

  return ret;
}

#endif
