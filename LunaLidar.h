const uint8_t HEADER = 0x59;
const uint8_t FRAME_SIZE = 9;

void setupLidar() {
  // Initialize serial and wait for port to open: 
  Serial1.begin(115200);

  delay(50);
}

bool lidarData(int16_t &dist, int16_t &flux, int16_t &temp) {

  uint8_t frame[FRAME_SIZE];
  uint8_t checksum = 0;

  // Flush all but last frame of data from the serial buffer.
  while (Serial1.available() > FRAME_SIZE) Serial1.read();

  // Search for HEADER bytes in the incoming data
  uint32_t serialTimeout = millis() + 1000;
  while ((frame[0] != HEADER) || (frame[1] != HEADER)) {
    if (Serial1.available()) {
      memmove(frame, frame + 1, FRAME_SIZE - 1);
      frame[FRAME_SIZE - 1] = Serial1.read();
    }

    if (millis() > serialTimeout) {
      Serial.println("TF Lidar not available yet.");
      delay(1000);
      return false;
    }
  }

  // Calculate checksum
  for (uint8_t i = 0; i < FRAME_SIZE - 1; i++) {
    checksum += frame[i];
  }

  // Verify checksum
  if (frame[FRAME_SIZE - 1] != (checksum & 0xFF)) {
    Serial.println("Checksum error.");
    return false;
  }

  // Extract and process data
  dist = frame[2] + (frame[3] << 8);
  flux = frame[4] + (frame[5] << 8);
  temp = frame[6] + (frame[7] << 8);

  // Convert temp code to degrees Celsius
  temp = (temp >> 3) - 256;

  return true;
}
