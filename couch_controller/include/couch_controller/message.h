/* 
 * File:   udp_message.h
 * Author: will
 *
 * Created on 8 April 2015, 2:50 PM
 */

// THIS FILE NEEDS GNU99, LITTLE ENDIAN

#ifndef UDP_MESSAGE_H
#define	UDP_MESSAGE_H

#include <stdint.h>

#define CONTROLLER_PORT 744

// At the beginning of all packets
struct pkt_header {
	char type;
	uint8_t crc; // NOT USED (UDP checksum instead) crc of all bytes in the message except this one
} __attribute__((packed));

struct timestamp {
    uint32_t tv_sec;
    uint32_t tv_nsec;
};

// ----[Packets received by controller]-----------------------------------------
// A - start stop stream of status packets to the IP that sent this at the
// requested frequency
#define PKT_TYPE_STREAM 'A'
struct pkt_cmd_stream {
	struct pkt_header header;
	uint8_t frequency; // Hz, approx. 0 = stopped
} __attribute__((packed));

// B - set wheel velocity setpoints
#define PKT_TYPE_WHEEL 'B'
struct pkt_cmd_wheel {
	struct pkt_header header;
	int16_t vels[4];
} __attribute__((packed));

// F - set time
#define PKT_TYPE_TIME 'F'
struct pkt_cmd_time {
    struct pkt_header header;
    struct timestamp time;
} __attribute__((packed));

// G - ping
#define PKT_TYPE_PING 'G'
struct pkt_cmd_ping {
	struct pkt_header header;
} __attribute__((packed));

// I - ??

// ----[Packets sent by controller]---------------------------------------------
// Z - status streamed back by A command
#define PKT_TYPE_STAT 'Z'
struct pkt_stat_stream {
    struct pkt_header header;
    struct timestamp timestamp;

    // Wheel displacement in ticks
    int32_t disps[4];
    // Wheel velocities in ticks per SECOND
    int16_t vels[4];

    uint16_t vbat; // in mv
	
	int8_t mtemps[4]; // Motor temperatures in C
	int8_t ctemps[4]; // Controller temperatures in C
	
	int16_t currents[4];
	
    uint16_t flags; // bit 0 = estop
} __attribute__((packed, aligned(4)));

// Y - response to ping
#define PKT_TYPE_PINGRESP 'Y'
struct pkt_stat_pingresp {
	struct pkt_header header;
} __attribute__((packed));

#endif	/* UDP_MESSAGE_H */
