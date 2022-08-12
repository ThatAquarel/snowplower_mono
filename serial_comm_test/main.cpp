#include <CppLinuxSerial/SerialPort.hpp>
#include <iostream>
#include <sstream>

using namespace mn::CppLinuxSerial;

int main() {
	SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
	serialPort.SetTimeout(-1);
	serialPort.Open();
	serialPort.Close();
}

