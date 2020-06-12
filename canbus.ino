
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
  dataSpeed = MIN_SPEED;
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
  if (dataCoolant<=60) adjustedCoolant=0x28; // if less than 60 degrees c
  else if (dataCoolant<=74) adjustedCoolant=0x63; // if less than 74 degrees c
  else if (dataCoolant<=90) adjustedCoolant=0x88; // if less than 90 degrees c
  else if (dataCoolant<=99) adjustedCoolant=0x9C; // if less than 99 degrees c
  else adjustedCoolant=0xD0; // 100 degrees c and over

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
