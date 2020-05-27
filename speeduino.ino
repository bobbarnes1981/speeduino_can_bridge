
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
  dataCoolant = MIN_COOLANT;
  dataRpm = MIN_RPM;
}

#ifdef RESTART_DELAYED_REQUEST
void speeduino_restart() {
  speeduinoFetchDelayedCount = speeduinoFetchDelayedCount + 1;

  #ifdef DEBUG
  sprintf(debugBuffer, "%i of %i delayed requests", speeduinoFetchDelayedCount, SPEEDUINO_RESTART_COUNT);
  debugger.println(debugBuffer);
  #endif

  if (speeduinoFetchDelayedCount >= SPEEDUINO_RESTART_COUNT) {
    #ifdef DEBUG
    debugger.println("restarting speeduio serial connection");
    #endif
    
    speeduino.end();
    speeduino.begin(SPEEDUINO_BAUD);
    speeduinoFetchDelayedCount = 0;
  }
}
#endif
