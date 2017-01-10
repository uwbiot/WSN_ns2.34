

#ifndef NS_RILAGENT_H
#define NS_RILAGENT_H

#include "agent.h"
#include "trafgen.h"
#include "packet.h"
#include "random.h"
#include "address.h"
#include "ip.h"

#include "rilapp.h"
#include "rilcommon.h"


class RILAgent : public Agent {
public:
	RILAgent();
	RILAgent(packet_t);
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
