#define BUF_SIZE 32
#define FRAME_START_FLAG 0x7E
#define FRAME_END_FLAG 0x7D

enum commands : uint8_t {
  NUL = 0x00,
  SYN = 0xAB,
  ACK = 0x4B,
  NAK = 0x5A,
  ERR = 0x3C
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
    if (requested_len > packet->len)
        requested_len = packet->len;

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

void setup() {
  comm_init(115200);
}

void loop() {
  comm_send_packet(ack_packet);
  delay(10);
}
