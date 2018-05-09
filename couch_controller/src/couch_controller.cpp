#include <stdint.h>
#include <cstdlib>
#include <stdbool.h>
#include <cstring>
#include <cerrno>
#include <string>
#include <math.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdio.h>

#include "couch_controller/couch_controller.h"
#include "couch_controller/message.h"
#include "couch_controller/control_loop.h"

namespace couch_controller
{
	Controller::Controller(std::string ip, bool openLoop) throw (SocketException)
	{
		// Setup socket
		udpSocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (udpSocket < 0) {
			throw SocketException("Opening", udpSocket);
		}

		struct timeval tv;
		tv.tv_sec = UDP_TIMEOUT / 1000000;
		tv.tv_usec = UDP_TIMEOUT % 1000000;

		setsockopt(udpSocket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

		openLoop_ = openLoop;

		// Store controller address
		memset(&controllerAddr, 0, sizeof(controllerAddr));
		controllerAddr.sin_family = AF_INET;
		controllerAddr.sin_port = htons(CONTROLLER_PORT);
		if (inet_aton(ip.c_str(), &controllerAddr.sin_addr) == 0) {
			throw SocketException("Parsing IP", 0);
		}

		// Check if controller is online
		if (!ping()) {
			throw SocketException("Couldn't connect", 0);
		}

		for (int i = 0; i < 4; i++) {
			loops[i] = new ControlLoop(0.01, 4.0);
		}
	}

	Controller::~Controller()
	{
		close(udpSocket);
	}

	bool Controller::ping()
	{
		struct pkt_cmd_ping pkt;
		pkt.header.type = PKT_TYPE_PING;
		sendPacket(&pkt, sizeof(pkt));

		socklen_t addrlen = sizeof(controllerAddr);
		int recv = recvfrom(udpSocket, &pkt, sizeof(pkt), 0, (struct sockaddr *)&controllerAddr, &addrlen);

		if (recv == sizeof(pkt) && pkt.header.type == PKT_TYPE_PINGRESP) {
			return true;
		} else {
			return false;
		}
	}

	class controllerStatus Controller::getStatusBlocking() throw (SocketException)
	{
		struct pkt_stat_stream pkt;
		socklen_t addrlen = sizeof(controllerAddr);

		int recv = recvfrom(udpSocket, &pkt, sizeof(pkt), 0, (struct sockaddr *)&controllerAddr, &addrlen);

		if (recv != (int)sizeof(pkt)) {
			if (recv == -1) {
				throw SocketException("Receiving/errno", errno);
			} else {
				throw SocketException("Receiving", recv);
			}
		}

		struct controllerStatus stat;

		stat.timestamp = pkt.timestamp;

		for (int i = 0; i < 4; i++) {
			stat.disps[i] = (double)pkt.disps[i] / ENCODER_TICKS_PER_REV *
				WHEEL_CIRCUMFERENCE;
			if (i == 1 || i == 3) {
				stat.disps[i] = -stat.disps[i];
			}

			// Determine velocity from first order displacement difference
			if (oldTimestamp_.tv_sec == 0) {
				stat.vels[i] = 0;
			} else {
				double delay = (stat.timestamp.tv_sec - oldTimestamp_.tv_sec)
					+ (double)(stat.timestamp.tv_nsec - oldTimestamp_.tv_nsec) / 1000000000;
				stat.vels[i] = (stat.disps[i] - oldDisps_[i]) / delay;
			}
			oldDisps_[i] = stat.disps[i];

			stat.mtemps[i] = pkt.mtemps[i];
			stat.ctemps[i] = pkt.ctemps[i];

			stat.drives[i] = pkt.drive_fb[i];
			stat.errors[i] = pkt.error_codes[i];
			stat.voltages[i] = (float)pkt.voltages[i] / 100;
			stat.currents[i] = (float)pkt.currents[i] / 100 ;
		}
		oldTimestamp_ = stat.timestamp;

		stat.vBat = (float)pkt.vbat / 100;

		static bool oldEstop = false;
		stat.estop = (pkt.flags & 1) == 1;

		stat.flags = pkt.flags;

		if (!oldEstop && stat.estop) {
			for (int i = 0; i < 4; i++) {
				loops[i]->zeroIntegrator();
			}
		}
		oldEstop = stat.estop;

		// If an encoder fails, switch to open loop
		if (stat.flags & FLAG_ENCODERFAIL) {
			openLoop_ = true;
		}

		// If doing closed loop control, run the controller and send back a command
		if (!openLoop_) {
			uint16_t us[4];
			for (int i = 0; i < 4; i++) {
				us[i] = commandToUs(loops[i]->doLoop(stat.vels[i]));
			}
			sendSpeedPacket(us);
		}
		return stat;
	}

	bool Controller::isOpenLoop()
	{
		return openLoop_;
	}

	std::string Controller::getMCErrorString(int error)
	{
		switch (error) {
			case PWR_ERROR_NONE:
				return "No error";
			case PWR_ERROR_PEAK_CURRENT:
				return "Peak current limit exceeded";
			case PWR_ERROR_AVE_CURRENT:
				return "Average current limit exceeded";
			case PWR_ERROR_HIGH_VOLTAGE:
				return "High voltage limit exceeded";
			case PWR_ERROR_LOW_VOLTAGE:
				return "Low voltage limit exceeded";
			case PWR_ERROR_HIGH_TEMP:
				return "High temperature limit exceeded";
			default:
				return "Unknown error";
		}
	}

	void Controller::sendSpeedPacket(uint16_t (&us)[4]) throw (SocketException)
	{
		struct pkt_cmd_wheel pkt;

		//std::cout << "Setting speeds to " << us[0] << ", " << us[1] << ", " << us[2] << ", " << us[3] << std::endl;

		pkt.header.type = PKT_TYPE_WHEEL;
		for (int i = 0; i < 4; i++) {
			pkt.pwms[i] = us[i];
		}

		sendPacket(&pkt, sizeof(pkt));
	}

	void Controller::sendPacket(void *pkt, size_t len) throw (SocketException)
	{
		if (sendto(udpSocket, pkt, len, 0,
					(struct sockaddr *)&controllerAddr, sizeof(controllerAddr)) < 0) {
			throw SocketException("Sending", (int)(((pkt_header *)pkt)->type));
		}
	}

	uint16_t Controller::commandToUs(double cmd)
	{
		if (cmd > 1) {
			cmd = 1;
		} else if (cmd < -1) {
			cmd = -1;
		}
		return US_STOP + (int16_t)round(US_RANGE * cmd);
	}
	
	void Controller::setVels(double (&vels)[4]) throw (SocketException)
	{
		if (openLoop_) {
			// Send commands directly
			uint16_t us[4];
			for (int i = 0; i < 4; i++) {
				us[i] = commandToUs(vels[i]);
			}
		//std::cout << "Setting speeds to " << us[0] << ", " << us[1] << ", " << us[2] << ", " << us[3] << std::endl;
			sendSpeedPacket(us);
		} else {
			for (int i = 0; i < 4; i++) {
				loops[i]->setPoint = vels[i];
			}
		}
	}

	int16_t truncGain(double gain)
	{
		if (gain > 2) {
			gain = 2;
		} else if (gain < 0) {
			gain = 0;
		}

		return (int16_t)(gain * 16383);
	}

	void Controller::setPIDGains(double f, double i, double maxAccel) throw (SocketException)
	{
		for (int i = 0; i < 4; i++) {
			loops[i]->ki = i;
			loops[i]->kffp = f;
			loops[i]->maxAcceleration = maxAccel;
		}
	}

	void Controller::setStatusHz(uint8_t hz) throw (SocketException)
	{
		struct pkt_cmd_stream pkt;

		pkt.header.type = PKT_TYPE_STREAM;
		pkt.frequency = hz;

		sendPacket(&pkt, sizeof(pkt));

		for (int i = 0; i < 4; i++) {
			loops[i]->dt = 1.0 / hz;
		}

		statusHz = hz;
	}

	void Controller::setTime(struct timestamp time) throw (SocketException)
	{
		struct pkt_cmd_time pkt;

		pkt.header.type = PKT_TYPE_TIME;
		pkt.time = time;

		sendPacket(&pkt, sizeof(pkt));
	}
}

