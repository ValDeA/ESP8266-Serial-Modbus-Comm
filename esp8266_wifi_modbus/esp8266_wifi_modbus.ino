
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


//#define _DEBUG


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
  char wifiServerIP_EEPROM[15];
  uint16_t wifiServerPort_EEPROM;
  char asciiWiFiSSID_EEPROM[32];
  char asciiWiFiPassword_EEPROM[16];
}  modbus;

void applyRelay() {
  if(modbus.relayCh1State == 0x00) {
    digitalWrite(RELAY, HIGH); // ESP-01 Relay HIGH => off
  } else if(modbus.relayCh1State == 0x01) {
    digitalWrite(RELAY, LOW);  // ESP-01 Relay LOW  => on
  }

//  if(modbus.relayCh2State == 0x00) {
//    digitalWrite(RELAY2, HIGH); // ESP-01 Relay HIGH => off
//  } else if(modbus.relayCh2State == 0x01) {
//    digitalWrite(RELAY2, LOW);  // ESP-01 Relay LOW  => ofn
//  }
}

void initModbus() {
  EEPROM.begin(EEPROM_SIZE);
  
  setRelayCh1(eeprom_modbus, &modbus, 0);
  setRelayCh2(eeprom_modbus, &modbus, 0);
  setFanSpeed(eeprom_modbus, &modbus, 0);
  setSlaveAddress(eeprom_modbus, &modbus, (uint16_t) 0x03);


  if(modbus.slaveAddress_EEPROM == 0x00 || modbus.slaveAddress_EEPROM == 0xFF) {
    setSlaveAddress(eeprom_modbus, &modbus, (uint16_t) 251);
  }
}
void initWiFi() {
  if(WiFi.isConnected()) {
    WiFi.disconnect();
    delay(1000);
  }

//  setWiFiIP_A(eeprom_modbus, &modbus, 49320);                     // 49320 = C0A8 = 192 168
//  setWiFiIP_B(eeprom_modbus, &modbus, 14);                        // 14    = 000E = 0   14
//  setWiFiPort(eeprom_modbus, &modbus, 5000);                      // 5000  = 1388
//  setWiFiSSID(eeprom_modbus, &modbus, (unsigned char*) "KT_GiGA_2G_Wave2_9B24");   // KT_GiGA_2G_Wave2_9B24 = 4b,54,5f,47,69,47,41,5f,32,47,5f,57,61,76,65,32,5f,39,42,32,34
//  setWiFiPassword(eeprom_modbus, &modbus, (unsigned char*) "fed88cz047");          // fed88cz047            = 66,65,64,38,38,63,7a,30,34,37
  
  getWiFiSSID(&modbus);
  getWiFiPassword(&modbus);

  int  connectCount = 0;
  
  WiFi.begin(modbus.asciiWiFiSSID_EEPROM, modbus.asciiWiFiPassword_EEPROM);
//  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    connectCount++;
  
    if(connectCount > 10) {
      Serial.println("WiFi Connect Failed");
      break;
    }
  }
}
void wifiConnect() {
  if(client.connected()) {
    client.stop();
    delay(1000);
  }
  
  getWiFiServerIP(&modbus);
  
  int retryCount = 0;
  while(1) {
    if(!client.connect(modbus.wifiServerIP_EEPROM, modbus.wifiServerPort_EEPROM)) {
//    if(!client.connect(WIFI_HOST, WIFI_PORT)) {
      delay(1000);
      retryCount++;
    } else {
      break;
    }

    if(retryCount > 5) break;
  }

  if(client.connected()) {
    byte bySlaveAddrHigh = (modbus.slaveAddress_EEPROM >> 8) & 0xFF;
    byte bySlaveAddrLow  = (modbus.slaveAddress_EEPROM     ) & 0xFF;
  
    unsigned char firstPacket[8] = { BROADCAST, WRITE_SINGLE, 0x00, 0x03, bySlaveAddrHigh, bySlaveAddrLow, 0x0, 0x0 };
    unsigned short nCRC = CRC_Compute(firstPacket, 6);

    byte byCRCHigh = (nCRC >> 8) & 0xFF;
    byte byCRCLow  = (nCRC     ) & 0xFF;

    firstPacket[6] = byCRCHigh;
    firstPacket[7] = byCRCLow;

    client.write(firstPacket, 8);
  }
  
}



void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println();

  setPinMode();
  initModbus();

  initWiFi();
  wifiConnect();
}

void loop() {
  wifiRead();
  
  if(!client.connected()) {
    wifiConnect();
  }
}

void setPinMode() {
  // prepare GPIO0
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, HIGH); // ESP-01 Relay HIGH => off
}





void wifiRead(){
//  if(!client.connected()) {
//    initWiFi();
//    wifiConnect();
//    return;
//  }
  
  unsigned short us_cnt = 0;
  unsigned long  ul_timeout = 500;
  unsigned long  ul_start;

  #ifdef _DEBUG
  if(Serial.available() > 0) {
  #else
  if(client.available() > 0) {
  #endif
    delay(100); // 한글자씩 읽히는 것을 방지하기 위한 딜레이
    ul_start = millis();

    #ifdef _DEBUG
    while(Serial.available() > 0) {
      byte ch = Serial.read();
    #else
    while(client.available() > 0) {
      byte ch = client.read();
    #endif  
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

void serialRead(){ 
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
    if(functionCode == READ_MULTIPLE) {           // Multiple Read
      makeWifiResponse(packet, len);
    } else if(functionCode == WRITE_SINGLE) {     // Single Write
      uint16_t   regAddr            = (packet[2] << 8) + packet[3];
      uint16_t   requestData        = (packet[4] << 8) + packet[5];

      makeWifiResponse(packet, len);
      processSingleWrite(regAddr, requestData);
    } else if(functionCode == WRITE_MULTIPLE) {
      byte resPacket[8] = { modbus.slaveAddress_EEPROM, functionCode, packet[2], packet[3], packet[4], packet[5], 0, 0 };
      short int resCRC = CRC_Compute(resPacket, 6);
      resPacket[6] = (resCRC >> 8) & 0xFF;
      resPacket[7] = (resCRC     ) & 0xFF;

      makeWifiResponse(resPacket, 8);
      
      processMultipleWrite(packet, len);
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
    #ifdef _DEBUG
    Serial.write(generatedPacket, 8);
    #else
    client.write(generatedPacket, 8);
    #endif
  } else if(functionCode == READ_MULTIPLE) {
    byte    regAddr[2]      = { packet[2], packet[3] };
    byte    readCount[2]    = { packet[4], packet[5] };
    
    uint16_t readStart = (regAddr[0] << 8) + regAddr[1];
    uint16_t readLength = (readCount[0] << 8) + readCount[1];
    uint16_t writeLength = readLength * 2;
    uint16_t totalLength = 5 + writeLength;

    generatedPacket = (byte *) malloc(totalLength);
    generateReadMultipleMessage(generatedPacket, readStart, writeLength);
    #ifdef _DEBUG
    Serial.write(generatedPacket, totalLength);
    #else
    client.write(generatedPacket, totalLength);
    #endif
    free(generatedPacket);
  } else if(functionCode == WRITE_MULTIPLE) {
    #ifdef _DEBUG
    Serial.write(packet, len);
    #else
    client.write(packet, len);
    #endif
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



void processSingleWrite(uint16_t regAddr, uint16_t u16Data) {
  switch(regAddr) {
    case 0:
      setRelayCh1(eeprom_modbus, &modbus, u16Data);
      applyRelay();
    break;
    case 1:
      setRelayCh2(eeprom_modbus, &modbus, u16Data);
      applyRelay();
    break;
    case 2:
      setFanSpeed(eeprom_modbus, &modbus, u16Data);
    break;
    case 3:
      setSlaveAddress(eeprom_modbus, &modbus, u16Data);
    break;
    case 4:
      // RESET
    break;
    case 5:
      // WIFI Server IP A
      setWiFiIP_A(eeprom_modbus, &modbus, u16Data);
    break;
    case 6:
      // WIFI Server IP B
      setWiFiIP_B(eeprom_modbus, &modbus, u16Data);
    break;
    case 7:
      setWiFiPort(eeprom_modbus, &modbus, u16Data);
    break;
    case 8 ... 23: // 08 ~ 23
//      unsigned char ssid[32];
//      memset(ssid, 0, 32);
//      memcpy(ssid, data+(2*i), writeCount);
//      setWiFiSSID(eeprom_modbus, &modbus, ssid);
    break;
    case 24 ... 31: // 24 ~ 31
//      unsigned char password[16];
//      memset(password, 0, 16);
//      memcpy(password, data+(2*i), writeCount);
//      setWiFiPassword(eeprom_modbus, &modbus, password);
    break;
  }
}
void processMultipleWrite(byte* packet, int len) {
  uint16_t   startAddr        = (packet[2] << 8) + packet[3];
  uint16_t   regCount         = (packet[4] << 8) + packet[5];
  uint8_t    writeCount       = packet[6];
  byte       data[writeCount];

  memset(data, 0, sizeof(byte) * writeCount);
  memcpy(data, packet+7, sizeof(byte) * writeCount);

  for(int i=0; i<regCount; i++) {
    uint16_t u16Data = (data[2*i] << 8) + data[2*i+1];
    uint16_t regAddr = startAddr + i;

    switch(regAddr) {
      case 0:
        setRelayCh1(eeprom_modbus, &modbus, u16Data);
        applyRelay();
      break;
      case 1:
        setRelayCh2(eeprom_modbus, &modbus, u16Data);
        applyRelay();
      break;
      case 2:
        setFanSpeed(eeprom_modbus, &modbus, u16Data);
      break;
      case 3:
        setSlaveAddress(eeprom_modbus, &modbus, u16Data);
      break;
      case 4:
        // RESET
      break;
      case 5:
        // WIFI Server IP A
        setWiFiIP_A(eeprom_modbus, &modbus, u16Data);
      break;
      case 6:
        // WIFI Server IP B
        setWiFiIP_B(eeprom_modbus, &modbus, u16Data);
        delay(1000);
        
        initWiFi();
        wifiConnect();
      break;
      case 7:
        setWiFiPort(eeprom_modbus, &modbus, u16Data);
      break;
      case 8: // 08 ~ 23
        unsigned char ssid[32];
        memset(ssid, 0, 32);
        memcpy(ssid, data+(2*i), writeCount);

        delay(1000);
        
        setWiFiSSID(eeprom_modbus, &modbus, ssid);
        initWiFi();
        wifiConnect();
      break;
      case 24: // 24 ~ 31
        unsigned char password[16];
        memset(password, 0, 16);
        memcpy(password, data+(2*i), writeCount);
        setWiFiPassword(eeprom_modbus, &modbus, password);

        delay(1000);
        
        initWiFi();
        wifiConnect();
      break;
    }
  }
}
