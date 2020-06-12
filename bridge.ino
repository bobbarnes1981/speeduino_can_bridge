
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
