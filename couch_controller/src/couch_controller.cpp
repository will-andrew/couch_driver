#include <stdint.h>
#include <cstdlib>
#include <stdbool.h>
#include <cstring>
#include <cerrno>
#include <string>

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
	Controller::Controller(std::string ip) throw (SocketException)
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
			stat.vels[i] = (double)pkt.vels[i] / ENCODER_TICKS_PER_REV *
				WHEEL_CIRCUMFERENCE / DRIVE_PERIOD;
			stat.mtemps[i] = pkt.mtemps[i];
			stat.ctemps[i] = pkt.ctemps[i];

			stat.currents[i] = (double)pkt.currents[i] / 65536.0 * 3.3 / 0.045; // 45 mV / A
		}

		stat.vBat = (double)pkt.vbat / 65536.0 * 3.3 / 3.3 * 85.3;

		static bool oldEstop = false;
		stat.estop = (pkt.flags & 1) == 1;

		if (!oldEstop && stat.estop) {
			for (int i = 0; i < 4; i++) {
				loops[i]->zeroIntegrator();
			}
		}
		oldEstop = stat.estop;

		// Run drive loop
		/*struct pkt_cmd_wheel spkt;

		spkt.header.type = PKT_TYPE_WHEEL;
		spkt.left = leftLoop->doLoop(stat.lVel);
		spkt.right = -rightLoop->doLoop(stat.rVel);

		sendPacket(&spkt, sizeof(spkt));*/

		return stat;
	}

	void Controller::sendPacket(void *pkt, size_t len) throw (SocketException)
	{
		if (sendto(udpSocket, pkt, len, 0,
					(struct sockaddr *)&controllerAddr, sizeof(controllerAddr)) < 0) {
			throw SocketException("Sending", (int)(((pkt_header *)pkt)->type));
		}
	}
	
	void Controller::setVels(double (&vels)[4]) throw (SocketException)
	{
		struct pkt_cmd_wheel pkt;

		pkt.header.type = PKT_TYPE_WHEEL;
		for (int i = 0; i < 4; i++) {
			//pkt.vels[i] = (int16_t)(vels[i] / WHEEL_DIAMETER * ENCODER_TICKS_PER_REV);
			pkt.vels[i] = (int16_t)(vels[i] * 2000); // TODO fix this
		}

		sendPacket(&pkt, sizeof(pkt));

		/*leftLoop->setPoint = lVel;
		rightLoop->setPoint = rVel;*/
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

	void Controller::setPIDGains(double lp, double li, double ld, double lffp, double lffd, double lffdd,
			double rp, double ri, double rd, double rffp, double rffd, double rffdd) throw (SocketException)
	{
		/*struct pkt_cmd_pid pkt;

		pkt.header.type = PKT_TYPE_PID;
		pkt.l_p = truncGain(lp);
		pkt.l_i = truncGain(li);
		pkt.l_d = truncGain(ld);
		pkt.r_p = truncGain(rp);
		pkt.r_i = truncGain(ri);
		pkt.r_d = truncGain(rd);

		sendPacket(&pkt, sizeof(pkt));*/

		/*leftLoop->kp = lp;
		leftLoop->ki = li;
		leftLoop->kd = ld;
		leftLoop->kffp = lffp;
		leftLoop->kffd = lffd;
		leftLoop->kffdd = lffdd;

		rightLoop->kp = rp;
		rightLoop->ki = ri;
		rightLoop->kd = rd;
		rightLoop->kffp = rffp;
		rightLoop->kffd = rffd;
		rightLoop->kffdd = rffdd;*/
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

