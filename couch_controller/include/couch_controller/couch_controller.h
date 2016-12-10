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

	double vBat;

	int mtemps[4];
	int ctemps[4];

	double currents[4];

	bool estop;
};

class Controller
{
	public:
	static const double ENCODER_TICKS_PER_REV = 8192;
	static const double WHEEL_DIAMETER = 171.0 * 3.14 / 1000;
	static const double BASE_WIDTH = 1.5;
	static const double BASE_LENGTH = 0.75;
	static const double DRIVE_PERIOD = 0.02; // From firmware, drive.c LOOP_RATE
	static const int FRONT_LEFT = 1;
	static const int BACK_LEFT = 0;
	static const int FRONT_RIGHT = 2;
	static const int BACK_RIGHT = 3;

	Controller(std::string ip) throw (SocketException);
	~Controller();

	struct controllerStatus getStatusBlocking() throw (SocketException);

	void setVels(double (&vels)[4]) throw (SocketException);
	 
	void setPIDGains(double lp, double li, double ld, double lffp, double lffd, double lffdd,
			double rp, double ri, double rd, double rffp, double rffd, double rffdd) throw (SocketException);

	void setStatusHz(uint8_t hz) throw (SocketException); // Set to 0 to stop

	void setTime(struct timestamp time) throw (SocketException);

	bool ping();

	private:
	int udpSocket;
	struct sockaddr_in controllerAddr;
	uint8_t statusHz;

	ControlLoop *loops[4];

	void sendPacket(void *pkt, size_t len) throw (SocketException);
};

}

#endif

