#include <CppLinuxSerial/SerialPort.hpp>
#include <iostream>
#include <sstream>

using namespace mn::CppLinuxSerial;

int main() {
	float a = 3.1415;
	float b = -3.1415;

	int vf, vr;
	vf = std::abs(static_cast<int>(a / 3.1415926 * 255));
	vr = std::abs(static_cast<int>(b / 3.1415926 * 255));
	
	std::ostringstream ss;

	if (a > 0) {
		ss << "1,0,";	
	} else {
		ss << "0,1,";
	}
	ss << vf;
	ss << ",";
	
	if (b > 0) {
		ss << "1,0,";
	} else {
		ss << "0,1,";
	}
	ss << vr;

	std::cout << ss.str() << std::endl;	
	

	// Create serial port object and open serial port at 57600 buad, 8 data bits, no parity bit, and one stop bit (8n1)
	SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
	// Use SerialPort serialPort("/dev/ttyACM0", 13000); instead if you want to provide a custom baud rate
	serialPort.SetTimeout(-1); // Block when reading until any data is received
	serialPort.Open();

	// Write some ASCII data
	//serialPort.Write("1,0,255,1,0,0");

	// Read some data back (will block until at least 1 byte is received due to the SetTimeout(-1) call above)
//	std::string readData;
//	serialPort.Read(readData);

	// Close the serial port
	serialPort.Close();
}

