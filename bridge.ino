
void bridge_startup() {
  if (currentMillis - startupLastMillis >= STARTUP_SWEEP_MILLIS / STARTUP_SWEEP_STEPS) {
    int speedStep = (STARTUP_SPEED_MAX - STARTUP_SPEED_MIN) / (STARTUP_SWEEP_STEPS * 2);
    int rpmStep = (STARTUP_RPM_MAX - STARTUP_RPM_MIN) / (STARTUP_SWEEP_STEPS * 2);

    #ifdef DEBUG
    sprintf(debugBuffer, "Step RPM: %i", rpmStep);
    debugger.println(debugBuffer);
    sprintf(debugBuffer, "Step Speed: %i", speedStep);
    debugger.println(debugBuffer);
    #endif
  
    if (startupSweepingUp) {
      startupRpmSweep += rpmStep;
      startupSpeedSweep += speedStep;
      if (startupRpmSweep >= STARTUP_RPM_MAX || startupSpeedSweep >= STARTUP_SPEED_MAX) {
        startupSweepingUp = false;
        startupRpmSweep = STARTUP_RPM_MAX;
        startupSpeedSweep = STARTUP_SPEED_MAX;
      }
    } else {
      startupRpmSweep -= rpmStep;
      startupSpeedSweep -= speedStep;
      if (startupRpmSweep <= STARTUP_RPM_MIN || startupSpeedSweep <= STARTUP_SPEED_MIN) {
        bridgeStartup = false;
        startupRpmSweep = STARTUP_RPM_MIN;
        startupSpeedSweep = STARTUP_SPEED_MIN;
      }
    }

    dataRpm = startupRpmSweep;
    dataSpeed = startupSpeedSweep;

    #ifdef DEBUG
    sprintf(debugBuffer, "Startup RPM: %i", startupRpmSweep);
    debugger.println(debugBuffer);
    sprintf(debugBuffer, "Startup Speed: %i", startupSpeedSweep);
    debugger.println(debugBuffer);
    #endif

    canbus_send_201();
    canbus_send_420();

    startupLastMillis = currentMillis;
  }
}

void bridge_running() {
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

      #ifdef RESTART_DELAYED_REQUEST
      speeduino_restart();
      #endif
      
      speeduino_reset_data();
      
      speeduinoFetchLastMillis = currentMillis;
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
      
      canbusFetchLastMillis = currentMillis;
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
      canbusSend201LastMillis = currentMillis;
      canbusSend201Delayed = true;
    }
  }

  if (currentMillis - canbusSend420LastMillis >= CANBUS_SEND420_INTERVAL) {
    if (canbus_send_420()) {
      canbusSend420LastMillis = currentMillis;
      canbusSend420Delayed = false;
    }
    if (currentMillis - canbusSend420LastMillis >= CANBUS_SEND420_DELAYED) {
      canbusSend420LastMillis = currentMillis;
      canbusSend420Delayed = true;
    }
  }  
}
