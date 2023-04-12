
#define HEADER     0x59
#define FRAME_SIZE 9

void setupLidar() {
  Serial1.begin(115200);
}

bool lidarData(int16_t &dist, int16_t &strength, int16_t &temp) {

  int check;
  int uart[9];
  int i;

  if (Serial1.available()) {
    if (Serial1.read() == HEADER) {
      uart[0] = HEADER;
      if (Serial1.read() == HEADER) {

        uart[1] = HEADER;
        for (i = 2; i < 9; i++) {
          uart[i] = Serial1.read();
        }
        check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];

        if (uart[8] == (check & 0xff)) {
          dist = uart[2] + (uart[3] << 8);
          strength = uart[4] + (uart[5] << 8);
          temp = uart[6] + (uart[7] << 8);

          // Convert temp code to degrees Celsius.
          temp = (temp >> 3) - 256;

          
        }
      }
    }
    return true;
  } else {
    Serial.println("TF Lidar not available yet.");
    delay(1000);
    return false;
  }
}