// Program for RaspberryPi 3B+ shield
//    Board    Generic stm32f103r series
//    Variant  Stm32f103rc (48k ram, 256k flash)
//    Upload   Serial
//    Cpu freq 72 mhz
//    Optimize Smallest (default)

#define BUF_SIZE 32
#define FRAME_START_FLAG 0x7E
#define FRAME_END_FLAG 0x7D

enum commands : uint8_t {
  NUL = 0x00,
  SYN = 0xAB,
  ACK = 0x4B,
  NAK = 0x5A,
  ERR = 0x3C,

  CTRL = 0x01,
  ODOM = 0x02
};

struct packet_t {
  commands cmd;
  size_t len;
  uint8_t *buf;

  packet_t(uint8_t *_buf, size_t _len) : cmd(NUL),
                                         len(_len),
                                         buf(_buf){};
  packet_t(commands _cmd, uint8_t *_buf, size_t _len) : cmd(_cmd),
                                                                  len(_len),
                                                                  buf(_buf){};
};

uint16_t comm_crc16(uint8_t *data_p, size_t len)
{
    if (len == 0 || data_p == nullptr)
        return 0xFFFF;

    uint8_t x;
    uint16_t crc = 0xFFFF;

    while (len--)
    {
        x = crc >> 8 ^ *data_p++;
        x ^= x >> 4;
        crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x << 5)) ^ ((uint16_t)x);
    }
    return crc;
}

void comm_init(unsigned long baud) {
    Serial.begin(baud);
}

void comm_recv_packet(struct packet_t *packet) {
    while (Serial.available() < 1);

    uint8_t flag = 0;
    while (flag != FRAME_START_FLAG)
        Serial.readBytes(&flag, 1);
    Serial.readBytes((uint8_t *)&packet->cmd, 1);

    uint16_t requested_len = 0;
    Serial.readBytes((uint8_t *)&requested_len, 2);
    if (requested_len > BUF_SIZE)
        requested_len = BUF_SIZE;
    packet->len = requested_len;

    Serial.readBytes(packet->buf, requested_len);
    uint16_t crc;
    Serial.readBytes((uint8_t *)&crc, 2);
    Serial.readBytes(&flag, 1);

    if (comm_crc16(packet->buf, requested_len) != crc)
        packet->cmd = NAK;
    if (flag != FRAME_END_FLAG)
        packet->cmd = ERR;
}

void comm_send_packet(struct packet_t *packet) {
    uint16_t crc = comm_crc16(packet->buf, packet->len);

    Serial.write(0x00);
    Serial.write(0x00);
    Serial.write(0x00);
    Serial.write(0x00);

    Serial.write(FRAME_START_FLAG);
    Serial.write(packet->cmd);
    Serial.write((uint8_t *)&(packet->len), 2);
    if (packet->buf != nullptr)
        Serial.write(packet->buf, packet->len);
    Serial.write((uint8_t *)&crc, 2);
    Serial.write(FRAME_END_FLAG);

    Serial.write(0x00);
    Serial.write(0x00);
    Serial.write(0x00);
    Serial.write(0x00);
}

uint8_t packet_buffer[BUF_SIZE];
packet_t *data_packet = new packet_t(packet_buffer, BUF_SIZE);
packet_t *ack_packet = new packet_t(ACK, nullptr, 0);
packet_t *nak_packet = new packet_t(NAK, nullptr, 0);
packet_t *err_packet = new packet_t(ERR, nullptr, 0);

#define pwm0 PA6
#define ina0 PA5
#define inb0 PA4

#define pwm1 PA7
#define ina1 PC4
#define inb1 PC5

#define ma0 PB6
#define mb0 PB7

#define ma1 PA0
#define mb1 PA1

//#define ledPin PC1

void setup() {
  comm_init(230400);

  pinMode(pwm0, OUTPUT);
  pinMode(ina0, OUTPUT);
  pinMode(inb0, OUTPUT);

  pinMode(pwm1, OUTPUT);
  pinMode(ina1, OUTPUT);
  pinMode(inb1, OUTPUT);

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

void loop() {
  comm_recv_packet(data_packet);
  comm_send_packet(ack_packet);

  if (data_packet->cmd == CTRL) {
    if (data_packet->len == 6) {
      analogWrite(pwm0, data_packet->buf[0]);
      digitalWrite(ina0, data_packet->buf[1]);
      digitalWrite(inb0, data_packet->buf[2]);
  
      analogWrite(pwm1, data_packet->buf[3]);
      digitalWrite(ina1, data_packet->buf[5]);
      digitalWrite(inb1, data_packet->buf[4]); 
    }

    comm_send_packet(ack_packet);
  } else if (data_packet->cmd == ODOM) {
    data_packet->len = 8;

    memcpy(data_packet->buf + 0, (uint8_t*)&m0, 4);
    memcpy(data_packet->buf + 4, (uint8_t*)&m1, 4);

    comm_send_packet(data_packet);
  }

  comm_send_packet(ack_packet);
}
