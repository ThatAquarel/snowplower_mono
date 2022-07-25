#include <CppLinuxSerial/SerialPort.hpp>
#include <iostream>
#include <sstream>
#include <vector>
#include <cassert>
#include <cstring>

using namespace mn::CppLinuxSerial;

SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_230400, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);

//std::vector<uint8_t> r_buffer;
//std::vector<uint8_t> w_buffer;

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

void comm_init() {
    serialPort.SetTimeout(-1);
    serialPort.Open();
}

std::vector<uint8_t> r_buffer;
std::vector<uint8_t> temp_r_buffer;
void fill_r_buffer() {
	if (!serialPort.Available() && r_buffer.size()) return;

	temp_r_buffer.clear();
	serialPort.ReadBinary(temp_r_buffer);
	r_buffer.insert(r_buffer.end(), temp_r_buffer.begin(), temp_r_buffer.end());
}

void comm_recv_packet(struct packet_t *packet) {
find_packet:
	fill_r_buffer();

	while (r_buffer.front() != FRAME_START_FLAG) {
		if (!r_buffer.size())
			goto find_packet;
		r_buffer.erase(r_buffer.begin());
	}
	if (r_buffer.size() < 4)
		goto find_packet;

	r_buffer.erase(r_buffer.begin());

    packet->cmd = static_cast<commands>(r_buffer.front());
	r_buffer.erase(r_buffer.begin());
	
    uint16_t requested_len = r_buffer.at(0) | (r_buffer.at(1) << 8);
	r_buffer.erase(r_buffer.begin(), r_buffer.begin() + 2);
    if (requested_len > BUF_SIZE)
        requested_len = BUF_SIZE;
	packet->len = requested_len;
	
	while (r_buffer.size() < (requested_len + 3))
		fill_r_buffer();

	std::copy(r_buffer.begin(), r_buffer.begin() + requested_len, packet->buf);
	r_buffer.erase(r_buffer.begin(), r_buffer.begin() + requested_len);

    uint16_t crc = r_buffer.at(0) | (r_buffer.at(1) << 8);
	r_buffer.erase(r_buffer.begin(), r_buffer.begin() + 2);

    if (comm_crc16(packet->buf, requested_len) != crc)
        packet->cmd = NAK;
    if (r_buffer.at(0) != FRAME_END_FLAG)
        packet->cmd = ERR;
}


std::vector<uint8_t> w_buffer;
void comm_send_packet(struct packet_t *packet) {
    uint16_t crc = comm_crc16(packet->buf, packet->len);

    w_buffer.push_back(0x00);
    w_buffer.push_back(0x00);
    w_buffer.push_back(0x00);
    w_buffer.push_back(0x00);

    w_buffer.push_back(FRAME_START_FLAG);
    w_buffer.push_back(packet->cmd);
	
	uint16_t len = (uint16_t) packet->len;
	w_buffer.insert(w_buffer.end(), (uint8_t *)&len, (uint8_t *)&len + 2);
    if (packet->buf != nullptr && packet->len != 0)
        w_buffer.insert(w_buffer.end(), packet->buf, packet->buf + packet->len);
    w_buffer.insert(w_buffer.end(), (uint8_t *)&crc, (uint8_t *)&crc + 2);
    w_buffer.push_back(FRAME_END_FLAG);

    w_buffer.push_back(0x00);
    w_buffer.push_back(0x00);
    w_buffer.push_back(0x00);
    w_buffer.push_back(0x00);

	serialPort.WriteBinary(w_buffer);
	w_buffer.clear();
}

uint8_t packet_buffer[BUF_SIZE];
packet_t *data_packet = new packet_t(packet_buffer, BUF_SIZE);
packet_t *ack_packet = new packet_t(ACK, nullptr, 0);

void acknowledge() {
	comm_recv_packet(ack_packet);
	assert(ack_packet->cmd == ACK && "SYN failed");
}

void send_recv_command(commands cmd, uint8_t *buf, size_t buf_len) {
	data_packet->cmd = cmd;
	data_packet->len = buf_len;
	if (buf_len > BUF_SIZE)
		buf_len = BUF_SIZE;
	if (buf_len > 0 && buf != nullptr)
		std::memcpy(data_packet->buf, buf, buf_len);
	
	comm_send_packet(data_packet);
	acknowledge();

	comm_recv_packet(data_packet);
	acknowledge();
}

int main() {
	comm_init();
	
	uint8_t proto[6] = {0, 1, 0, 0, 1, 0};	

	while (1) {
		send_recv_command(CTRL, proto, 6);
		send_recv_command(ODOM, nullptr, 0);
		
		int m0, m1;
		std::memcpy((uint8_t*)&m0, data_packet->buf + 0, 4);
		std::memcpy((uint8_t*)&m1, data_packet->buf + 4, 4);

		std::cout << m0 << " " << m1 << std::endl;
	}	
}
