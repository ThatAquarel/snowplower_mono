//  Program For RaspberryPi 3B+ Shield
//    Board: Generic STM32F103R
//    Variant: STM32F103RB 20k Memory 128k Flash
//    Method: Serial
//    Clock Speed: 72Mhz
//    Optimize: Smallest(Default)

#define pwmA PA6
#define in1A PA5
#define in2A PA4

#define pwmB PA7
#define in1B PC4
#define in2B PC5

#define ledPin PC1

#define maxPacketLength 40

String movementPacket = "";
int movementResult[] = {0, 0, 0, 0, 0, 0};

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);

  pinMode(pwmA, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);

  pinMode(pwmB, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    movementPacket = Serial.readString();

    for (int i = 0; i < 6; i++) {
      movementResult[i] = splitString(movementPacket, ',', i).toInt();
    }

    analogWrite(pwmA, movementResult[2]);
    digitalWrite(in1A, movementResult[0]);
    digitalWrite(in2A, movementResult[1]);
  
    analogWrite(pwmB, movementResult[5]);
    digitalWrite(in1B, movementResult[4]);
    digitalWrite(in2B, movementResult[3]);
  }
}

String splitString(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
