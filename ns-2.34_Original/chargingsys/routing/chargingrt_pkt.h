/*
 * nrrrouting_pkt.h
 *
 *  Created on: Sep 21, 2009
 *      Author: ypeng
 */

#ifndef CHARGINGRT_PKT_H_
#define CHARGINGRT_PKT_H_

#include "packet.h"

#define HDR_CHARGINGRT_PKT(p) hdr_chargingrt_pkt::access(p)

struct hdr_chargingrt_pkt{
	nsaddr_t pkt_src_;			// node initialized this packet
	u_int16_t pkt_len_;			// packet length
	u_int16_t pkt_seq_num_;		// packet sequence
	static int offset_;
	inline static int& offset(){return offset_;}
	inline static hdr_chargingrt_pkt* access(const Packet* p) {
		return (hdr_chargingrt_pkt*)p->access(offset_);
	}
};

#endif
