#define ma0 PB6
#define mb0 PB7
#define ma1 PA0
#define mb1 PA1

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);
  
  pinMode(ma0, INPUT);
  pinMode(mb0, INPUT);
  pinMode(ma1, INPUT);
  pinMode(mb1, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(ma0), isr_m0, RISING);
  attachInterrupt(digitalPinToInterrupt(ma1), isr_m1, RISING);
}

volatile int32_t m0 = 0;
volatile int32_t m1 = 0;

void isr_m0(){
  if (digitalRead(mb0) == HIGH){
    m0++;
  } else {
    m0--;
  }
  
}

void isr_m1(){
  if (digitalRead(mb1) == HIGH){
    m1--;
  } else {
    m1++;
  }
}

//double pulses_to_rad(int pulses){
//  return pulses / 475.0 * TWO_PI;
//}

void loop() {
  Serial.print(m0);
  Serial.print(" ");
  Serial.print(m1);
  Serial.println(" ");
  delay(100);
}
