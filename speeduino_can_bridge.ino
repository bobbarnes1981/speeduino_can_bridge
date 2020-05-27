#include <can.h>
#include <mcp2515.h>

#define DEBUG

#define RESEND_DELAYED_REQUEST
#define RESTART_DELAYED_REQUEST

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

#define SPEEDUINO_CANID 0x510
#define SPEEDUINO_COMMAND 'r'
#define SPEEDUINO_COMMAND_TYPE 0x30
#define SPEEDUINO_BAUD 115200
#define SPEEDUINO_RESTART_COUNT 10

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
#define MIL_ON 0x40
#define MIL_OFF 0x00

#define MIN_COOLANT 0x00  // 0c
#define MAX_COOLANT 0xFF  // ?
#define MIN_RPM 0x0000    // 0rpm
#define MAX_RPM 0x7D00    // 8000rpm
#define MIN_SPEED 0x2710  // 0mph
#define MAX_SPEED 0x7FD7  // 150mph

#define STARTUP_SWEEP_DELAY 1000

byte dataCoolant = MIN_COOLANT; // current coolant
word dataRpm = MIN_RPM;         // current rpm
word dataSpeed = MIN_SPEED;     // current speed

unsigned long currentMillis;

// the last time the action was successful
unsigned long speeduinoFetchLastMillis = 0;
unsigned long canbusFetchLastMillis = 0;
unsigned long canbusSend201LastMillis = 0;
unsigned long canbusSend420LastMillis = 0;

// action delayed flags
bool speeduinoFetchDelayed = false;
bool canbusFetchDelayed = false;
bool canbusSend201Delayed = false;
bool canbusSend420Delayed = false;

// true if request sent to speeduino
// false if awaiting response
bool speeduinoRequested = false;

bool bridgeStartup = true;
bool startupSweepingUp = true;
unsigned long startupLastMillis = -STARTUP_SWEEP_DELAY;

#ifdef RESTART_DELAYED_REQUEST
int speeduinoFetchDelayedCount = 0;
#endif

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
}

void loop() {
  currentMillis = millis();

  // flash the LED on LED_PIN once per LED_TIME
  digitalWrite(LED_PIN, (millis() % LED_TIME) > LED_TIME / 2);

  if (bridgeStartup) {
    bridge_startup();
  } else {
    bridge_running();
  }
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
