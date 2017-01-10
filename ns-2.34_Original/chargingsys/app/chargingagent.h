

#ifndef NS_CHARGINGAGENT_H
#define NS_CHARGINGAGENT_H

#include "agent.h"
#include "trafgen.h"
#include "packet.h"
#include "random.h"
#include "address.h"
#include "ip.h"

#include "chargingapp.h"
#include "chargingcommon.h"

#define SAMPLERATE 8000
#define RTP_M 0x0080 // marker for significant events

class ChargingAgent : public Agent {
public:
	ChargingAgent();
	ChargingAgent(packet_t);
	void stop();
	virtual void sendmsg(int nbytes, const char *flags = 0) {
		sendmsg(nbytes, NULL, flags);
	}
	virtual void sendmsg(int nbytes, AppData* data, const char *flags = 0);
	virtual void recv(Packet* pkt, Handler*);
	virtual int command(int argc, const char*const* argv);
protected:
	int seqno_;
private:
	int lastSeq;
	double lastTime;
};

#endif
