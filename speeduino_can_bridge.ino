#include <can.h>
#include <mcp2515.h>

#define DEBUG

#ifdef __AVR_ATmega2560__
#define USE_HW_SERIAL3
#endif
#ifdef __AVR_ATmega328P__
#define USE_SW_SERIAL
#endif

#ifdef DEBUG
#include <stdio.h>
char debugBuffer[255];
#endif

#ifdef USE_SW_SERIAL
#include <SoftwareSerial.h>
#endif

#define SPEEDUINO_CANID 0x5A1
#define SPEEDUINO_R_COMMAND 0x30
#define SERIAL_BAUD 115200
#define SPEEDUINO_BAUD 115200

#define MILLIS_BETWEEN_READS 100

#define DATA_TO_REQUEST 16
#define COOLANT_OFFSET 7
#define RPM_OFFSET 14

#define SW_SERIAL_RX 7
#define SW_SERIAL_TX 8

#define MCP2515_CS 53
#define MCP2515_BITRATE CAN_500KBPS
#define MCP2515_CLOCK MCP_8MHZ

#define LED_TIME 500

byte dataCoolant; // current coolant
word dataRpm;     // current rpm
word dataSpeed;   // current speed

word requiredBytes = 0; // required number of bytes
word receivedBytes = 0; // received number of bytes
byte serialBuffer[255];  // buffer for storing serial data

unsigned long lastMillis;

enum State {
  state_waiting,
  state_request_data,
  state_reading_data,
  state_storing_data,
  state_reading_canbus,
  state_writing_canbus,
  state_error
};

#ifdef DEBUG
HardwareSerial &debugger = Serial;
#endif
#ifdef USE_SW_SERIAL
SoftwareSerial speeduino(SW_SERIAL_RX, SW_SERIAL_TX);
#endif
#ifdef USE_HW_SERIAL3
HardwareSerial &speeduino = Serial3;
#endif

MCP2515 mcp2515(MCP2515_CS);
struct can_frame canMsg201;
struct can_frame canMsg420;
struct can_frame canMsg4B0;

State currentState = state_waiting;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  #ifdef DEBUG
  debugger.begin(SERIAL_BAUD);
  #endif
  speeduino.begin(SPEEDUINO_BAUD);
  mcp2515.reset();
  mcp2515.setBitrate(MCP2515_BITRATE, MCP2515_CLOCK);
  mcp2515.setNormalMode();
  #ifdef DEBUG
  debugger.println("speeduino_can_bridge");
  sprintf(debugBuffer, "debug on serial @ %ld", SERIAL_BAUD);
  debugger.println(debugBuffer);
  #ifdef USE_SW_SERIAL
  sprintf(debugBuffer, "speeduino on sw serial @ %lld ", (int)SPEEDUINO_BAUD);
  #endif
  #ifdef USE_HW_SERIAL3
  sprintf(debugBuffer, "speeduino on hw serial3 @ %lld ", (int)SPEEDUINO_BAUD);
  #endif
  debugger.println(debugBuffer);
  #endif
  lastMillis = millis();
}

void loop() {
  digitalWrite(LED_BUILTIN, (millis() % LED_TIME) > LED_TIME / 2);
  switch (currentState) {
    case state_waiting:
      if (timeout()) {
        #ifdef DEBUG
        debugger.println("timeout elapsed");
        #endif
        currentState = state_request_data;
      }
      break;
    case state_request_data:
      #ifdef DEBUG
      debugger.println("requesting data");
      #endif
      speeduinoRequestRealtime(0, DATA_TO_REQUEST);
      currentState = state_reading_data;
      #ifdef DEBUG
      debugger.println("reading serial");
      #endif
      break;
    case state_reading_data:
      if (readSerial()) {
        #ifdef DEBUG
        debugger.println("finished reading serial");
        #endif
        currentState = state_storing_data;
      }
      break;
    case state_storing_data:
      // first byte should be r and second should be command
      if (serialBuffer[0] == (byte)'r' && serialBuffer[1] == SPEEDUINO_R_COMMAND) {
        #ifdef DEBUG
        debugger.println("storing data");
        #endif
        dataCoolant = serialBuffer[2 + COOLANT_OFFSET];
        dataRpm = word(serialBuffer[2 + RPM_OFFSET + 1], serialBuffer[2 + RPM_OFFSET + 0]);
        currentState = state_reading_canbus;
        #ifdef DEBUG
        debugger.println("waiting canbus message 0x4B0");
        #endif
      } else {
        #ifdef DEBUG
        sprintf(debugBuffer, "error 0x%02X 0x%02X", serialBuffer[0], serialBuffer[1]);
        debugger.println(debugBuffer);
        #endif
        currentState = state_error;
      }
      break;
    case state_reading_canbus:
      {
        MCP2515::ERROR e = mcp2515.readMessage(&canMsg4B0);
        if (e == MCP2515::ERROR_OK) {
          #ifdef DEBUG
          sprintf(debugBuffer, "got can bus frame with id: 0x%02X", (int)canMsg4B0.can_id);
          debugger.println(debugBuffer);
          #endif
          if (canMsg4B0.can_id == 0x4B0) {
            #ifdef DEBUG
            debugger.println("reading canbus message 0x4B0");
            #endif
            dataSpeed = word(canMsg4B0.data[4], canMsg4B0.data[5]);
            currentState = state_writing_canbus;
          }
        } else {
          #ifdef DEBUG
          if (e != MCP2515::ERROR_NOMSG) {
            sprintf(debugBuffer, "can bus error: %d", e);
            debugger.println(debugBuffer);
          }
          #endif
        }
      }
      break;
    case state_writing_canbus:
      #ifdef DEBUG
      debugger.println("writing to canbus");
      #endif
      stateWritingCanbus();
      currentState = state_waiting;
      break;
    case state_error:
      // do nothing
      break;
    default:
      // shouldn't get here...
      break;
  }
}

// check how many millis have elapsed and return true if we
// have reached the required number
bool timeout() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= MILLIS_BETWEEN_READS) {
    lastMillis = currentMillis;
    return true;
  }
  return false;
}

// send a request for realtime data
// reset received bytes and set required data length
void speeduinoRequestRealtime(word data_offset, word data_length) {
  speeduino.write((byte)'r');
  speeduino.write((byte)SPEEDUINO_CANID);
  speeduino.write((byte)SPEEDUINO_R_COMMAND);
  speeduino.write((byte)lowByte(data_offset));
  speeduino.write((byte)highByte(data_offset));
  speeduino.write((byte)lowByte(data_length));
  speeduino.write((byte)highByte(data_length));

  receivedBytes = 0;
  requiredBytes = data_length + 2;
}

// read from serial, return true if read enough bytes into buffer
bool readSerial() {
  while (speeduino.available() && receivedBytes < requiredBytes) {
    serialBuffer[receivedBytes] = speeduino.read();
    #ifdef DEBUG
    sprintf(debugBuffer, "0x%02X", serialBuffer[receivedBytes]);
    debugger.println(debugBuffer);
    #endif
    receivedBytes++;
  }
  return receivedBytes == requiredBytes;
}

void stateWritingCanbus() {
  word adjustedRpm = dataRpm * 4;

  byte adjustedCoolant;
  
  if (dataCoolant<=60) adjustedCoolant=0x28;
  else if (dataCoolant<=74) adjustedCoolant=0x63;
  else if (dataCoolant<=90) adjustedCoolant=0x88;
  else if (dataCoolant<=99) adjustedCoolant=0x9C;
  else adjustedCoolant=0xD0;

  #ifdef DEBUG
  sprintf(debugBuffer, "coolant: %d", dataCoolant);
  debugger.println(debugBuffer);
  
  sprintf(debugBuffer, "rpm: %d", dataRpm);
  debugger.println(debugBuffer);
  
  sprintf(debugBuffer, "speed: %d (%dmph)", dataSpeed, (int)((dataSpeed * 0.0066)-66));
  debugger.println(debugBuffer);
  #endif

  canMsg201.can_id = 0x201;
  canMsg201.can_dlc = 8;
  canMsg201.data[0] = highByte(adjustedRpm);// rpm
  canMsg201.data[1] = lowByte(adjustedRpm); // rpm
  canMsg201.data[2] = 0x00;                 // 
  canMsg201.data[3] = 0x00;                 // 
  canMsg201.data[4] = highByte(dataSpeed);  // speed
  canMsg201.data[5] = lowByte(dataSpeed);   // speed
  canMsg201.data[6] = 0x00;                 // 
  canMsg201.data[7] = 0x00;                 // 
  mcp2515.sendMessage(&canMsg201);

  canMsg420.can_id = 0x420;
  canMsg420.can_dlc = 8;
  canMsg420.data[0] = adjustedCoolant;  // ect
  canMsg420.data[1] = 0x00;         // pres.
  canMsg420.data[2] = 0x00;         // fuel flow
  canMsg420.data[3] = 0x00;         // prndl
  canMsg420.data[4] = 0x00;         // MIL/overdrive
  canMsg420.data[5] = 0x00;         // safe cooling/PATS
  canMsg420.data[6] = 0x00;         // charging system status
  canMsg420.data[7] = 0x00;         // engine off elapsed time
  mcp2515.sendMessage(&canMsg420);
}
