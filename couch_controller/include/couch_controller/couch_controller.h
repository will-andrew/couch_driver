#ifndef COUCH_CONTROLLER_H
#define COUCH_CONTROLLER_H

#include <string>
#include <stdint.h>
#include <stdexcept>
#include <iostream>
#include <exception>
#include <sstream>

#include <sys/socket.h>
#include <netinet/in.h>

#include "message.h"
#include "control_loop.h"

namespace couch_controller
{

const int UDP_TIMEOUT = 100000; // in us

class SocketException : public std::exception
{
	public:
	SocketException(const std::string stage, const int code) : stage(stage), code(code)  {}
	
	virtual ~SocketException() throw () {};
		
	virtual const char *what() const throw ()
	{
		static std::ostringstream oss;
		oss.str("");
		oss << "Socket operation failed. " << stage << ", " << code;
		return oss.str().c_str();
	}

	private:
	int code;
	std::string stage;
};

struct controllerStatus
{
	struct timestamp timestamp;

	double disps[4];
	double vels[4];

	float vBat;

	int mtemps[4];
	int ctemps[4];

	float currents[4];
	float voltages[4];
	int drives[4];
	int errors[4];

	int flags;

	bool estop;
};
	
enum pwrErrors {
	PWR_ERROR_NONE = 0,
	PWR_ERROR_PEAK_CURRENT = 1,
	PWR_ERROR_AVE_CURRENT = 2,
	PWR_ERROR_HIGH_VOLTAGE = 3,
	PWR_ERROR_LOW_VOLTAGE = 4,
	PWR_ERROR_HIGH_TEMP = 5,
	N_PWR_ERRORS
};

class Controller
{
	public:
	static const double ENCODER_TICKS_PER_REV = 1024 * 7.5;
	static const double WHEEL_CIRCUMFERENCE = 250.0 * 3.14 / 1000;
	static const double BASE_WIDTH = 0.485;
	static const double BASE_LENGTH = 1.060;
	static const double DRIVE_PERIOD = 0.02; // From firmware, drive.c LOOP_RATE
	static const int FRONT_LEFT = 1;
	static const int BACK_LEFT = 0;
	static const int FRONT_RIGHT = 2;
	static const int BACK_RIGHT = 3;

	// PWM values (us)
	static const int US_STOP = 1500;
	static const int US_RANGE = 500; // +/-

	Controller(std::string ip, bool openLoop = false) throw (SocketException);
	~Controller();

	std::string getMCErrorString(int error);

	struct controllerStatus getStatusBlocking() throw (SocketException);

	void setVels(double (&vels)[4]) throw (SocketException);
	 
	void setPIDGains(double f, double i, double maxAccel) throw (SocketException);

	void setStatusHz(uint8_t hz) throw (SocketException); // Set to 0 to stop

	void setTime(struct timestamp time) throw (SocketException);

	bool ping();

	bool isOpenLoop();

	private:
	int udpSocket;
	struct sockaddr_in controllerAddr;
	uint8_t statusHz;
	bool openLoop_;

	int uptime_;

	double oldDisps_[4];
	struct timestamp oldTimestamp_;

	ControlLoop *loops[4];

	// Converts -1 to 1 range to a PWM value in us
	uint16_t commandToUs(double cmd);

	void sendPacket(void *pkt, size_t len) throw (SocketException);
	void sendSpeedPacket(uint16_t (&us)[4]) throw (SocketException);
};

}

#endif

