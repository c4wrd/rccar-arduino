char receive_buffer[255];
char response_buffer[255];

void printPins() {
  int forwardPin = digitalRead(bmForwardPin);
  int reversePin = digitalRead(bmBackwardPin);
  char buffer [50];
  sprintf(buffer, "front: %d, back: %d", forwardPin, reversePin);
  updateDisplay((char*)&buffer, WHITE);
}

bool serverLoop(RCCarCommand *command) {
  int packetSize = server.parsePacket();
  if (packetSize)
  {
    char buffer [50];
    sprintf(buffer, "Received %d bytes", packetSize);
    Serial.println(buffer);
    
    int len = server.read((char*)command, sizeof(RCCarCommand));
    //memcpy(command, receive_buffer, sizeof(RCCarCommand)); // copy buffer to command
    
    /*for ( size_t i = 0; i < len; i++ ) {
      byte b = receive_buffer[i];
      uint8_t *p = (uint8_t*)(&command) + i;
      memcpy(p, &b, 1);
    }*/

    // TODO respond with Gyro information
    server.beginPacket(server.remoteIP(), server.remotePort());
    server.write("Received command...");
    server.endPacket();

    COMMAND_RECEIVED = true;
    LAST_COMMAND_TIME = millis();

    return true;
  }
  
  return false;
}

void setupWifi() {
  Serial.println("test");
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    updateDisplay("No wifi shield...");
    while (true);       // don't continue
  }

  String fv = WiFi.firmwareVersion();
  if (fv != "1.1.0") {
    Serial.println("Please upgrade the firmware");
  }

  while (WIFI_STATUS != WL_CONNECTED) {
    updateDisplay("Attempting to connect...");
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(WIFI_SSID);
    WIFI_STATUS = WiFi.begin(WIFI_SSID, PASSWORD);
    delay(1000);
  }

  server.begin(UDP_LISTEN_PORT);
}

void printWifiStatus() {
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print("Connected to ");
  display.println(WIFI_SSID);
  display.setCursor(0,16);
  display.print("IP address: ");
  display.println(WiFi.localIP());
  display.display();

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  // print where to go in a browser:
  Serial.println(ip);
}


