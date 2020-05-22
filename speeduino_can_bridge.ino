#define DEBUG

#ifdef __AVR_ATmega2560__
#define USE_HW_SERIAL3
#endif
#ifdef __AVR_ATmega328P__
#define USE_SW_SERIAL
#endif

#ifdef USE_SW_SERIAL
#include <SoftwareSerial.h>
#endif

#define SPEEDUINO_CANID 0x00
#define SPEEDUINO_R_COMMAND 0x30
#define SPEEDUINO_BAUD 115200

#define MILLIS_BETWEEN_READS 1000

#define DATA_TO_REQUEST 16
#define COOLANT_OFFSET 7
#define RPM_OFFSET 14

#define SW_SERIAL_RX 7
#define SW_SERIAL_TX 8

byte dataCoolant; // current coolant
byte dataRpmLo;   // current rpm lo byte
byte dataRpmHi;   // current rpm hi byte

word requiredBytes = 0; // required number of bytes
word receivedBytes = 0; // received number of bytes
byte serialBuffer[255];  // buffer for storing serial data

unsigned long lastMillis;

enum State {
  state_waiting,
  state_request_data,
  state_reading_data,
  state_storing_data,
  state_writing_canbus,
  state_error
};

#ifdef USE_SW_SERIAL
SoftwareSerial speeduino(SW_SERIAL_RX, SW_SERIAL_TX);
#endif
#ifdef USE_HW_SERIAL3
HardwareSerial &speeduino = Serial3;
#endif

State currentState = state_waiting;

void setup() {
  Serial.begin(9600);
  speeduino.begin(SPEEDUINO_BAUD);
  // TODO: initialise canbus
  #ifdef DEBUG
  Serial.println("started");
  #endif
  lastMillis = millis();
}

void loop() {
  switch (currentState) {
    case state_waiting:
      if (timeout()) {
        #ifdef DEBUG
        Serial.println("timeout elapsed");
        #endif
        currentState = state_request_data;
      }
      break;
    case state_request_data:
      #ifdef DEBUG
      Serial.println("requesting data");
      #endif
      speeduinoRequestRealtime(0, DATA_TO_REQUEST);
      currentState = state_reading_data;
      #ifdef DEBUG
      Serial.println("reading serial");
      #endif
      break;
    case state_reading_data:
      if (readSerial()) {
        #ifdef DEBUG
        Serial.println("finished reading serial");
        #endif
        currentState = state_storing_data;
      }
      break;
    case state_storing_data:
      // first byte should be r and second should be command
      if (serialBuffer[0] == (byte)'r' && serialBuffer[1] == SPEEDUINO_R_COMMAND) {
        #ifdef DEBUG
        Serial.println("storing data");
        #endif
        dataCoolant = serialBuffer[2 + COOLANT_OFFSET];
        dataRpmLo = serialBuffer[2 + RPM_OFFSET + 0];
        dataRpmHi = serialBuffer[2 + RPM_OFFSET + 1];
        currentState = state_writing_canbus;
      } else {
        #ifdef DEBUG
        Serial.println("error");
        Serial.println(serialBuffer[0]);
        Serial.println(serialBuffer[1]);
        #endif
        currentState = state_error;
      }
      break;
    case state_writing_canbus:
      #ifdef DEBUG
      Serial.println("writing to canbus");
      #endif
      stateWritingCanbus();
      currentState = state_waiting;
      break;
    case state_error:
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
    Serial.println(serialBuffer[receivedBytes]);
    #endif
    receivedBytes++;
  }
  return receivedBytes == requiredBytes;
}

void stateWritingCanbus() {
  #ifdef DEBUG
  Serial.print("coolant: ");
  Serial.print(dataCoolant);
  Serial.println();
  Serial.print("rpm: ");
  Serial.print(dataRpmLo);
  Serial.print(dataRpmHi);
  Serial.println();
  #endif
  // TODO: write to data canbus
}
