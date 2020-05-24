#include <can.h>
#include <mcp2515.h>

#define DEBUG
#define RESEND_DELAYED_REQUEST

#ifdef __AVR_ATmega2560__
#define USE_HW_SERIAL3
#define SPI_CS 53
#endif
#ifdef __AVR_ATmega328P__
#define USE_SW_SERIAL
#define SPI_CS 10
#endif

#ifdef DEBUG
#include <stdio.h>
char debugBuffer[255];
#define DEBUG_BAUD 115200
#endif

#ifdef USE_SW_SERIAL
#include <SoftwareSerial.h>
#define SW_SERIAL_RX 7
#define SW_SERIAL_TX 8
#endif

#define SPEEDUINO_CANID 0x5A1
#define SPEEDUINO_COMMAND 'r'
#define SPEEDUINO_COMMAND_TYPE 0x30
#define SPEEDUINO_BAUD 115200

#define DATA_TO_REQUEST 16
#define COOLANT_OFFSET 7
#define RPM_OFFSET_LO 14
#define RPM_OFFSET_HI 15

#define MCP2515_CS SPI_CS
#define MCP2515_BITRATE CAN_500KBPS
#define MCP2515_CLOCK MCP_8MHZ

#define LED_TIME 500
#define LED_PIN 9

// minimum interval for actions
#define SPEEDUINO_FETCH_INTERVAL 100
#define CANBUS_FETCH_INTERVAL 100
#define CANBUS_SEND201_INTERVAL 100
#define CANBUS_SEND420_INTERVAL 100

// maximum interval for actions
#define SPEEDUINO_FETCH_DELAYED 500
#define CANBUS_FETCH_DELAYED 500
#define CANBUS_SEND201_DELAYED 500
#define CANBUS_SEND420_DELAYED 500

// engine warning light states
#define MIL_FLASH 0xC0
#define MIL_ON 0x04
#define MIL_OFF 0x00

#define DEFAULT_COOLANT 0x00
#define DEFAULT_RPM 0x0000
#define DEFAULT_SPEED 0x2710

byte dataCoolant = DEFAULT_COOLANT; // current coolant
word dataRpm = DEFAULT_RPM;         // current rpm
word dataSpeed = DEFAULT_SPEED;     // current speed

unsigned long currentMillis;

// the last time the action was successful
unsigned long speeduinoFetchLastMillis;
unsigned long canbusFetchLastMillis;
unsigned long canbusSend201LastMillis;
unsigned long canbusSend420LastMillis;

// action delayed flags
bool speeduinoFetchDelayed = false;
bool canbusFetchDelayed = false;
bool canbusSend201Delayed = false;
bool canbusSend420Delayed = false;

// true if request sent to speeduino
// false if awaiting response
bool speeduinoRequested = false;

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
bool shownNoMessageError = false; // flag so we only show NO MESSAGE error once

void setup() {
  pinMode(LED_PIN, OUTPUT);
  
  #ifdef DEBUG
  debugger.begin(DEBUG_BAUD);
  #endif
  
  speeduino.begin(SPEEDUINO_BAUD);
  
  mcp2515.reset();
  mcp2515.setBitrate(MCP2515_BITRATE, MCP2515_CLOCK);
  mcp2515.setNormalMode();
  
  #ifdef DEBUG
  debugger.println("speeduino_can_bridge");
  sprintf(debugBuffer, "debug on serial @ %li", DEBUG_BAUD);
  debugger.println(debugBuffer);
  #ifdef USE_SW_SERIAL
  sprintf(debugBuffer, "speeduino on sw serial @ %li ", SPEEDUINO_BAUD);
  #endif
  #ifdef USE_HW_SERIAL3
  sprintf(debugBuffer, "speeduino on hw serial3 @ %li ", SPEEDUINO_BAUD);
  #endif
  debugger.println(debugBuffer);
  #endif

  speeduinoFetchLastMillis = 0;
  canbusFetchLastMillis = 0;
  canbusSend201LastMillis = 0;
  canbusSend420LastMillis = 0;
}

void loop() {
  currentMillis = millis();

  // flash the LED on LED_PIN once per LED_TIME
  digitalWrite(LED_PIN, (millis() % LED_TIME) > LED_TIME / 2);

  if (currentMillis - speeduinoFetchLastMillis >= SPEEDUINO_FETCH_INTERVAL) {
    if (!speeduinoRequested) {
      speeduino_request();
    } else {
      if (speeduino_read()) {
        speeduinoFetchLastMillis = currentMillis;
        speeduinoFetchDelayed = false;
      }
    }
    if (currentMillis - speeduinoFetchLastMillis >= SPEEDUINO_FETCH_DELAYED) {
      #ifdef RESEND_DELAYED_REQUEST
      speeduinoRequested = false;
      #endif
      speeduino_reset_data();
      speeduinoFetchDelayed = true;

      #ifdef DEBUG
      debugger.println("speeduino request delayed");
      #endif
    }
  }

  if (currentMillis - canbusFetchLastMillis >= CANBUS_FETCH_INTERVAL) {
    if (canbus_fetch()) {
      canbusFetchLastMillis = currentMillis;
      canbusFetchDelayed = false;
    }
    if (currentMillis - canbusFetchLastMillis >= CANBUS_FETCH_DELAYED) {
      canbus_reset_data();
      canbusFetchDelayed = true;

      #ifdef DEBUG
      debugger.println("canbus fetch delayed");
      #endif
    }
  }

  if (currentMillis - canbusSend201LastMillis >= CANBUS_SEND201_INTERVAL) {
    if (canbus_send_201()) {
      canbusSend201LastMillis = currentMillis;
      canbusSend201Delayed = false;
    }
    if (currentMillis - canbusSend201LastMillis >= CANBUS_SEND201_DELAYED) {
      canbusSend201Delayed = true;
    }
  }

  if (currentMillis - canbusSend420LastMillis >= CANBUS_SEND420_INTERVAL) {
    if (canbus_send_420()) {
      canbusSend420LastMillis = currentMillis;
      canbusSend420Delayed = false;
    }
    if (currentMillis - canbusSend420LastMillis >= CANBUS_SEND420_DELAYED) {
      canbusSend420Delayed = true;
    }
  }
}

// request data from speeduino
void speeduino_request() {
  #ifdef DEBUG
  sprintf(debugBuffer, "sending serial request 0x%02X 0x%02X", SPEEDUINO_COMMAND, SPEEDUINO_COMMAND_TYPE);
  debugger.println(debugBuffer);
  #endif

  speeduino.write((byte)SPEEDUINO_COMMAND);
  speeduino.write((byte)SPEEDUINO_CANID);
  speeduino.write((byte)SPEEDUINO_COMMAND_TYPE);
  speeduino.write((byte)0x00);
  speeduino.write((byte)0x00); // offset 0x0000
  speeduino.write((byte)lowByte(DATA_TO_REQUEST));
  speeduino.write((byte)highByte(DATA_TO_REQUEST));

  speeduinoRequested = true;
}

// attempt to read data from speeduino
bool speeduino_read() {
  if (speeduino.available() >= DATA_TO_REQUEST + 2) {
    char cmd = speeduino.read();
    char cmd_type = speeduino.read();

    #ifdef DEBUG
    sprintf(debugBuffer, "received serial response 0x%02X 0x%02X", cmd, cmd_type);
    debugger.println(debugBuffer);
    #endif
    
    if (cmd == SPEEDUINO_COMMAND && cmd_type == SPEEDUINO_COMMAND_TYPE) {
      for (int i = 0; i < DATA_TO_REQUEST; i++) {
        byte data = speeduino.read();
        switch (i) {
          case COOLANT_OFFSET:
            dataCoolant = data;
            break;
          case RPM_OFFSET_LO:
            dataRpm = word(dataRpm, data);
            break;
          case RPM_OFFSET_HI:
            dataRpm = word(data, dataRpm);
            break;
        }
      }
    } else {
      // error, empty the buffer
      while(speeduino.available()) {
        speeduino.read();
      }
    }
    speeduinoRequested = false;
    return true;
  }
  return false;
}

// reset data from speeduino
void speeduino_reset_data() {
  dataCoolant = DEFAULT_COOLANT;
  dataRpm = DEFAULT_RPM;
}

// fetch data from canbus
bool canbus_fetch() {
  MCP2515::ERROR e = mcp2515.readMessage(&canMsg4B0);
  if (e == MCP2515::ERROR_OK) {
    shownNoMessageError = false;
    
    #ifdef DEBUG
    sprintf(debugBuffer, "got can bus frame with id: 0x%02X", (int)canMsg4B0.can_id);
    debugger.println(debugBuffer);
    #endif

    if (canMsg4B0.can_id == 0x4B0) {
      #ifdef DEBUG
      debugger.println("reading canbus message 0x4B0");
      #endif

      dataSpeed = word(canMsg4B0.data[4], canMsg4B0.data[5]);
      return true;
    }
  } else {
    #ifdef DEBUG
    if (e != MCP2515::ERROR_NOMSG || !shownNoMessageError) {
      shownNoMessageError = true;
      sprintf(debugBuffer, "can bus error: %d", e);
      debugger.println(debugBuffer);
    }
    #endif
  }
  return false;
}

// reset data from canbus
void canbus_reset_data() {
  dataSpeed = DEFAULT_SPEED;
}

// send canbus 0x201 message
bool canbus_send_201() {
  #ifdef DEBUG
  debugger.println("sending 201");
  
  sprintf(debugBuffer, "rpm: %d", dataRpm);
  debugger.println(debugBuffer);
  
  sprintf(debugBuffer, "speed: %d (%dmph)", dataSpeed, (int)((dataSpeed * 0.0066)-66));
  debugger.println(debugBuffer);
  #endif
  
  word adjustedRpm = dataRpm * 4;
  
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
  
  MCP2515::ERROR e = mcp2515.sendMessage(&canMsg201);

  #ifdef DEBUG
  sprintf(debugBuffer, "can bus error: %d", e);
  debugger.println(debugBuffer);
  #endif
  
  return e == MCP2515::ERROR_OK;
}

// send canbus 0x420 message
bool canbus_send_420() {
  debugger.println("sending 420");

  byte milState = get_mil();

  #ifdef DEBUG
  sprintf(debugBuffer, "coolant: %d", dataCoolant);
  debugger.println(debugBuffer);
  sprintf(debugBuffer, "mil: 0x%02X", milState);
  #endif
  
  byte adjustedCoolant;
  if (dataCoolant<=60) adjustedCoolant=0x28;
  else if (dataCoolant<=74) adjustedCoolant=0x63;
  else if (dataCoolant<=90) adjustedCoolant=0x88;
  else if (dataCoolant<=99) adjustedCoolant=0x9C;
  else adjustedCoolant=0xD0;

  canMsg420.can_id = 0x420;
  canMsg420.can_dlc = 8;
  canMsg420.data[0] = adjustedCoolant;      // ect
  canMsg420.data[1] = 0x00;                 // pres.
  canMsg420.data[2] = 0x00;                 // fuel flow
  canMsg420.data[3] = 0x00;                 // prndl
  canMsg420.data[4] = milState;             // MIL/overdrive
  canMsg420.data[5] = 0x00;                 // safe cooling/PATS
  canMsg420.data[6] = 0x00;                 // charging system status
  canMsg420.data[7] = 0x00;                 // engine off elapsed time
  
  MCP2515::ERROR e = mcp2515.sendMessage(&canMsg420);
  
  #ifdef DEBUG
  sprintf(debugBuffer, "can bus error: %d", e);
  debugger.println(debugBuffer);
  #endif
  
  return e == MCP2515::ERROR_OK;
}

// get MIL state
byte get_mil() {
  byte milState = MIL_OFF;
  if (speeduinoFetchDelayed) {
    milState |= MIL_FLASH;
  }
  if (canbusFetchDelayed) {
    milState |= MIL_FLASH;
  }
  if (canbusSend201Delayed) {
    milState |= MIL_FLASH;
  }
  if (canbusSend420Delayed) {
    milState |= MIL_FLASH;
  }
  return milState;
}
