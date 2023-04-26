#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <EEPROM.h>


#define RELAY 0

#if 0
#define WIFI_SSID           "KT_GiGA_2G_Wave2_9B24"
#define WIFI_PASSWORD       "fed88cz047"
#define WIFI_HOST           "192.168.0.14"
#define WIFI_PORT           5000
#endif


#define BROADCAST           0x00
#define READ_MULTIPLE       0x03
#define WRITE_SINGLE        0x06
#define WRITE_MULTIPLE      0x10

#define WIFI_BUFFER_MAX     1024
#define EEPROM_SIZE         1024

unsigned char rxWifiData[WIFI_BUFFER_MAX] = { 0 };
unsigned char eeprom_modbus[EEPROM_SIZE]  = { 0 };
WiFiClient client;



/*
 *        EEPROM MAP == MODBUS MAP ADDRESS
 *        
 *        Address         Function Desc         Remark
 *        00              Relay Chaanel 1       0: off, 1: On
 *        01              Relay Channel 2       0: Off, 1: On
 *        02              Fan Speed             0: Stop, 1: Low, 2: Normal, 3: High, 4: Turbo
 *        03              MODBUS Slave Addr     Default: 0xFB
 *        04              Reset                 Not Used
 *        05              WiFi Server IP-1      MSB: W, LSB: X
 *        06              WiFi Server IP-2      MSB: Y, LSB: Z
 *        07              WiFi Server Port      
 *        08~23           WiFi AP SSID          MSB: Even, LSB: Odd
 *        24~31           WiFi AP Password      MSB: Even, LSB: Odd
 */
struct MODBUS {
  byte relayCh1State;
  byte relayCh2State;
  byte fanSpeed;
  byte slaveAddress_EEPROM;
  uint16_t wifiServerIP_EEPROM[2];
  uint16_t wifiServerPort_EEPROM;
  uint16_t asciiWiFiSSID_EEPROM[16];
  uint16_t asciiWiFiPassword_EEPROM[8];
}  modbus;


int  connectCount         = 0;


void initModbus() {
  EEPROM.begin(EEPROM_SIZE);
  
  setRelayCh1(eeprom_modbus, &modbus, 0);
  setRelayCh2(eeprom_modbus, &modbus, 0);
  setFanSpeed(eeprom_modbus, &modbus, 0);
  setSlaveAddress(eeprom_modbus, &modbus, (uint16_t) 0x03);


  if(modbus.slaveAddress_EEPROM == 0x00 || modbus.slaveAddress_EEPROM == 0xFF) {
    setSlaveAddress(eeprom_modbus, &modbus, (uint16_t) 251);
    Serial.print("Slave
  }

  
  modbus.wifiServerIP_EEPROM[0]   = EEPROM.read(1);
  modbus.wifiServerIP_EEPROM[1]   = EEPROM.read(2);
  modbus.wifiServerPort_EEPROM    = EEPROM.read(3);


  for(int i=0; i<16; i++) {
    modbus.asciiWiFiSSID_EEPROM[i] = EEPROM.read(i+4);
  }

  for(int i=0; i<8; i++) {
    modbus.asciiWiFiPassword_EEPROM[i] = EEPROM.read(i+20);
  }
}


void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println();

  
  // prepare GPIO0
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, HIGH); // ESP-01 Relay HIGH => off
  
  initModbus();
}

void loop() {
//  Serial.println(modbus.slaveAddress_EEPROM);
//  delay(2000);

  wifiRead();
}





void wifiRead(){
  unsigned short us_cnt = 0;
  unsigned long  ul_timeout = 500;
  unsigned long  ul_start;
      
  if(Serial.available() > 0) {
    delay(100); // 한글자씩 읽히는 것을 방지하기 위한 딜레이
    ul_start = millis();

    while(Serial.available() > 0) {
      byte ch = Serial.read();
      rxWifiData[us_cnt] = ch;
      
      us_cnt++;
      delay(10);
      if(millis() - ul_start > ul_timeout) break;
      if(us_cnt >= WIFI_BUFFER_MAX) break;
    }

    if(us_cnt > 0) {
      if(checkWifiCRC(us_cnt) == true) {      
        processWifiRead(rxWifiData, us_cnt);
        us_cnt = 0;
      }
    }
    
  }
  
}

bool checkWifiCRC(int len) {
  uint16_t crc = CRC_Compute(rxWifiData, len - 2);
//  uint16_t crc = CRC_Compute(rxWifiData, 6);
  uint8_t crcHi = (uint8_t)(crc >> 8);
  uint8_t crcLo = (uint8_t)(crc & 0x00FF);

  if(crcHi != rxWifiData[len - 2] || crcLo != rxWifiData[len - 1]){
    #ifdef _DEBUG
    printf("Get = %02X%02X, Cal = %02X%02X\n", rxWifiData[6], rxWifiData[7], crcHi, crcLo);
    #endif
    return false;
  } else {
    return true;
  }
}

void processWifiRead(unsigned char* packet, int len) {
  byte    slaveAddr       = packet[0];
  byte    functionCode    = packet[1];
//  byte    regAddr[2]      = { packet[2], packet[3] };
//  byte    readCount[2]    = { packet[4], packet[5] };
  byte    crc[2]          = { packet[len-2], packet[len-1] };


  if(modbus.slaveAddress_EEPROM == slaveAddr || modbus.slaveAddress_EEPROM == BROADCAST) {
    if(functionCode == READ_MULTIPLE) {            // Multiple Read
      makeWifiResponse(packet, len);
    } else if(functionCode == WRITE_SINGLE) {     // Single Write
      uint16_t   regAddr            = (packet[2] << 8) + packet[3];
      uint16_t   requestData        = (packet[4] << 8) + packet[5];

      makeWifiResponse(packet, len);
      processWrite(regAddr, requestData);
    } else if(functionCode == WRITE_MULTIPLE) {
      uint16_t   startAddr        = (packet[2] << 8) + packet[3];
      uint16_t   regCount         = (packet[4] << 8) + packet[5];
      uint8_t    writeCount       = packet[6];
      byte*      data             = (byte *) malloc(writeCount);
      memcpy(data, packet+7, sizeof(byte) * writeCount);

      for(int i=0; i<regCount; i++) {
        uint16_t u16Data = (data[2*i] << 8) + data[2*i+1];
        processWrite(startAddr + i, u16Data);
      }

      byte resPacket[8] = { modbus.slaveAddress_EEPROM, functionCode, packet[2], packet[3], packet[4], packet[5], 0, 0 };
      short int resCRC = CRC_Compute(resPacket, 6);
      resPacket[6] = (resCRC >> 8) & 0xFF;
      resPacket[7] = (resCRC     ) & 0xFF;

      makeWifiResponse(resPacket, 8);
      free(data);
    }

  } else {
    return;
  }

}





void readWifi(byte* packet, int start, int len, int maxBufSize, int maxAddrSize) {
  if(start + len > maxAddrSize) return; // 잘못된 범위

  byte readData[maxBufSize] = { 0, };
  
  for(int i=start; i<start+len; i++) {
    readData[i - start] = packet[i];
  }
}


void makeWifiResponse(byte* packet, int len) {
  byte    slaveAddr       = packet[0];
  byte    functionCode    = packet[1];

  byte*   generatedPacket;

  if(modbus.slaveAddress_EEPROM != slaveAddr) return;
  if(functionCode == WRITE_SINGLE) {
    generatedPacket = packet; // 에코
    Serial.write(generatedPacket, 8);
  } else if(functionCode == READ_MULTIPLE) {
    byte    regAddr[2]      = { packet[2], packet[3] };
    byte    readCount[2]    = { packet[4], packet[5] };
    
    uint16_t readStart = (regAddr[0] << 8) + regAddr[1];
    uint16_t readLength = (readCount[0] << 8) + readCount[1];
    uint16_t writeLength = readLength * 2;
    uint16_t totalLength = 5 + writeLength;

    generatedPacket = (byte *) malloc(totalLength);
    generateReadMultipleMessage(generatedPacket, readStart, writeLength);
    Serial.write(generatedPacket, totalLength);
    free(generatedPacket);
  } else if(functionCode == WRITE_MULTIPLE) {
    Serial.write(packet, len);
  }
  
}



void generateReadMultipleMessage(byte* packet, uint16_t nStart, uint16_t nLen) {
  uint16_t  totalLength   = 5 + nLen;
  uint8_t   bodyLength    = 0;
  
//  packet[0] = e2promData.slaveAddress;
  packet[0] = 0xFB;
  packet[1] = READ_MULTIPLE;

  for(int i=0; i<nLen; i++) {
    packet[3 + i] = eeprom_modbus[nStart + i];
    bodyLength++;
  }
  packet[2] = bodyLength & 0xFF;

  uint16_t computeCRC = CRC_Compute(packet, totalLength - 2);
  packet[totalLength - 1] = (computeCRC)      & 0xFF;
  packet[totalLength - 2] = (computeCRC >> 8) & 0xFF;
}



void processWrite(uint16_t u16RegAddr, uint16_t data) {
  switch(u16RegAddr) {
    case 0:
      setRelayCh1(eeprom_modbus, &modbus, data);
    break;
    case 1:
      setRelayCh2(eeprom_modbus, &modbus, data);
    break;
    case 2:
      setFanSpeed(eeprom_modbus, &modbus, data);
    break;
    case 3:
      setSlaveAddress(eeprom_modbus, &modbus, data);
    break;
    case 4:
      // RESET
    break;
    case 5:
      // WIFI Server IP A
    break;
    case 6:
      // WIFI Server IP B
    break;
    case 7:
      setWiFiPort(eeprom_modbus, &modbus, data);
    break;
  }
}
