char receive_buffer[255];
char response_buffer[255];

bool serverLoop(RCCarCommand *command) {
  boolean cmd_ready = false;
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    //Serial.print("Received packet of size ");
    //Serial.println(packetSize);
    //Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    //Serial.print(remoteIp);
    //Serial.print(", port ");
    //Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    int len = Udp.read((char*)command, sizeof(RCCarCommand));
    if (len > 0) {
      cmd_ready = true;
      COMMAND_RECEIVED = true;
      LAST_COMMAND_TIME = millis();
    }

    sendResponse();
    
  }

  return cmd_ready;
}

void sendResponse() {
    Vector normAccel = mpu.readNormalizeAccel();
    Vector rawAccel = mpu.readRawAccel();
  
    // Calculate Pitch & Roll
    int pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
    int roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;

    String buffer;
    buffer += String(normAccel.XAxis);
    buffer += F(",");
    buffer += String(normAccel.YAxis);
    buffer += F(",");
    buffer += String(normAccel.ZAxis);
    buffer += F(",");
    buffer += String(pitch);
    buffer += F(",");
    buffer += String(roll);
    
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(buffer.c_str());
    Udp.endPacket();
}

void setupWifi() {
  WiFi.init(&Serial);

  //Serial.print("Attempting to start AP ");
  //Serial.println(ssid);

  IPAddress localIp(192, 168, 0, 1);
  WiFi.configAP(localIp);
  
  // start access point
  status = WiFi.beginAP(ssid, 10, pass, ENC_TYPE_WPA2_PSK);

  //Serial.println("Access point started");
  
  Udp.begin(UDP_LISTEN_PORT);
}


