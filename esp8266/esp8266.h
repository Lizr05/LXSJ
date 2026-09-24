#ifndef __ESP8266_H
#define __ESP8266_H

#include "main.h"

//热点
#define SSID    "TZ"
#define PASS    "12346789"

//TCP服务器
#define TCP_SSID    "ESP8266"
#define TCP_PASS    "12346789"

#define SOFTAP_SSID "ESP8266"  
#define SOFTAP_PASSWORD "12345678" 

#define Link_ID	1;

#define BUFF_SIZE 2048

int ParseUsartFrame(const char *buf,const char *check_data);
void Start_Recv(void);
int ESP_SendCommand(const char* cmd,const char* resp,uint32_t timeout);
int ESP8266_Setmode(uint8_t mode);
int ESP8266_Set_Wifi(void);
int ESP8266_Set_TCP(void);
int ESP8266_Connect_Wifi(void);
int ESP8266_SendToTCPServer(const char* txData, int timeout);
void ESP8266_Send_ESP32(const char* txData_, int timeout_);
#endif

