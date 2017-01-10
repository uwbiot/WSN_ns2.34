

#include "chargingagent.h"
#include <iostream>
static class ChargingAgentClass : public TclClass {
public:
	ChargingAgentClass() : TclClass("Agent/Charging") {}
	TclObject* create(int, const char*const*) {
		return (new ChargingAgent());
	}
}class_ChargingAgent;

ChargingAgent::ChargingAgent() : Agent(PT_CHARGING), seqno_(-1), lastSeq(-1), lastTime(-1.0) {
	bind("packetSize_", &size_);
}

ChargingAgent::ChargingAgent(packet_t type) : Agent(type), lastSeq(-1), lastTime(-1.0) {
	bind("packetSize_", &size_);
}

void ChargingAgent::stop() {

}

//commonherader|ipheader|packetdata+appdata header|nrrapp header
//
void ChargingAgent::sendmsg(int nbytes, AppData* data, const char* flags) {
	int n = nbytes;
	assert (size_ > 0);
	if (n == -1) {
		return;
	}	
	// If they are sending data, then it must fit within a single packet.
	if (n > size_) {
		return;
	}

	//double local_time = NOWTIME;
	PacketData* pd = (PacketData*)data;
	CAppMessageT_* cappmsgp = (CAppMessageT_*)pd->data();
	Packet* p = 0;
	if (n > 0) {
		// allocate a packet
		p = allocpkt();
		// fill common header content
		hdr_cmn *ch = hdr_cmn::access(p);
		ch->size() = n;
		ch->timestamp() = (u_int32_t)(NOW);
		ch->ptype() = PT_CHARGING;
		ch->num_forwards() = 0;
		// fill ip header content
		hdr_ip *iph = hdr_ip::access(p);
	    iph->ttl() = 100;
	    iph->saddr() = addr();
	    iph->sport() = 0;
	    iph->dport() = 0;
	    // check nrrmsg net type to update the ip header
	    if (cappmsgp->netType == MCAST || cappmsgp->netType == BCAST) {
	    	iph->daddr() = IP_BROADCAST;
		    iph->sport() = 0;
		    iph->dport() = 0;
	    }
	    else {
	    	iph->daddr() = cappmsgp->dest;
	    }
	    p->setdata(data);
		target_->recv(p);
	}
	idle();
}
void ChargingAgent::recv(Packet* pkt, Handler* h) {
#if 1
	if (app_ ) {
		hdr_cmn* ch = hdr_cmn::access(pkt);
		hdr_ip* ih = hdr_ip::access(pkt);
		// this data may include unicast msg to me, snooped unicast msg to others and bcast msg
		//PacketData* pd = (PacketData*)pkt->userdata();
		//CAppMessageT_* msg = (CAppMessageT_*)pd->data();
		// this is a unicast message sent to by me
        //printf("node %d recv app\n", here_.addr_);
		if (ih->daddr() == here_.addr_) {
			app_->process_data(ch->size(), pkt->userdata());
			Packet::free(pkt);
		}
#if 0
		// this is a new bcast msg, need to rebroadcast
		else if (ih->daddr() == IP_BROADCAST && ih->saddr() != addr() && msg->seq > lastSeq && msg->timestamp != lastTime) {
			lastSeq = msg->seq;
			lastTime = msg->timestamp;
			ch->direction() = hdr_cmn::DOWN;
			app_->process_data(ch->size(), pkt->userdata()); // signal up
			target_->recv(pkt);// rebroadcast
		}
#endif
		// if this is not a unicast message sent to me, i cannot free it here
	} else {
		// don't process it if agent is null
		Packet::free(pkt);
	}
#endif
}


int ChargingAgent::command(int argc, const char*const* argv) {
	Tcl& tcl = Tcl::instance();
	if (argc == 3) {
		if (strcmp(argv[1], "connect-agent") == 0) {
			Agent* a=0;
			a = (Agent*)tcl.lookup(argv[2]);
			if (a!=0) {
				std::cout<<a->addr()<<std::endl;
				std::cout<<a->port()<<std::endl;
			}
			dst_.addr_ = a->addr();
			dst_.port_ = a->port();
			return (TCL_OK);
		}
	}
	return (Agent::command(argc, argv));
}

