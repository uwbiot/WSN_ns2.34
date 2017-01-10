/*
 * nrrrouting.cc
 *
 *  Created on: Sep 21, 2009
 *      Author: ypeng
 */

#include "chargingrt.h"
#include "parent.h"
using namespace std;

int hdr_chargingrt_pkt::offset_;
static class ChargingRTHeaderClass:public PacketHeaderClass {
public:
	ChargingRTHeaderClass():
		PacketHeaderClass("PacketHeader/ChargingRT", sizeof(hdr_chargingrt_pkt)) {
		bind_offset(&hdr_chargingrt_pkt::offset_);
	}
}class_chartingrt_hdr;

static class ChargingRTClass : public TclClass {
public:
    ChargingRTClass() : TclClass("Agent/ChargingRT") {}
    TclObject* create(int argc, const char*const* argv) {
        return (new ChargingRT((nsaddr_t)Address::instance().str2addr(argv[4])));
    }
}class_chargingrt;

ChargingRT::ChargingRT(nsaddr_t id) : Agent(PT_CHARGINGRT),
		ra_addr_(id),seq_num_(0),packetSent_(0),
        parentID_(-1), sinkID_(0), hopCnt_(0xffff),myCost_(MAX_COST){
	bind("RTYPE", &routingType_);
    //bind("BeaconRate", &beaconInterval_);
	btimer = new RTBeaconTimer(this);
}

ChargingRT::~ChargingRT(){
	if (btimer) btimer->force_cancel();
	delete btimer;
    btimer = 0;
}

int ChargingRT::command(int argc, const char*const* argv) {
	Tcl& tcl = Tcl::instance();
	if(argc == 2) {
		if(strncasecmp(argv[1], "id", 2) == 0) {
		  return TCL_OK;
		}
		else if(strncasecmp(argv[1], "start", 2) == 0) {
			//if (btimer) btimer->sched(1.0+CHARGINGJITTER);
			printf("node %d x %f y %f \n",ra_addr_, mnode->X(),mnode->Y());
			return TCL_OK;
		 }
	  }
	  else if(argc == 3) {
		if(strcmp(argv[1], "log-target") == 0 || strcmp(argv[1], "tracetarget") == 0) {
		   logtarget_ = (Trace*) TclObject::lookup(argv[2]);
		   if(logtarget_ == 0) return TCL_ERROR;
		   return TCL_OK;
		}
		else if (strcmp(argv[1], "port-dmux") == 0) {
			dmux_ = (PortClassifier *)TclObject::lookup(argv[2]);
			if (dmux_ == 0) {
				fprintf (stderr, "%s: %s lookup of %s failed\n", __FILE__, argv[1], argv[2]);
				return TCL_ERROR;
			}
			return TCL_OK;
		}
        else if (strcmp(argv[1], "add-node") == 0) {
			mnode = (MobileNode*)tcl.lookup(argv[2]);
            if (mnode!=0) {return TCL_OK;}
            else return TCL_ERROR;            
        }
	  }
	  return Agent::command(argc, argv);
}

void ChargingRT::updateNeightorTable(neighborT_& node) {
    if (God::instance()->isSink(ra_addr_)) return;
	list<neighborT_>::iterator itl = neighborL.begin();
	list<neighborT_>::iterator end = neighborL.end();
//printf("neighborinfo: node %d x %f y %f eng %f cost %f sinkid %d\n", node.id, node.x, node.y, node.energy, node.cost, node.sinkID);
    while (itl!=end){
		if (itl->id == node.id) {
			itl->x = node.x;
			itl->y = node.y;
            itl->energy = node.energy;
            itl->cost = node.cost;
            itl->sinkID = node.sinkID;
            itl->hopCnt = node.hopCnt;
            itl->timestamp = node.timestamp;
            itl->parent = node.parent;
            if (itl->energy<1) {neighborL.erase(itl);}
			return;
		}
		itl++;
	}
    if (node.energy>1){
        neighborL.push_back(node);
    }
}

void ChargingRT::evictNeighbor(int nodeid) {
	list<neighborT_>::iterator itl = neighborL.begin();
	list<neighborT_>::iterator end = neighborL.end();
    while (itl!=end){
		if (itl->id == nodeid) {
            neighborL.erase(itl);
            //God::instance()->ctrace()->log("node %d evict %d @ %f\n", ra_addr_, nodeid, NOW);
			return;
		}
		itl++;
	}
}

void ChargingRT::updateNeighborTime(int nodeid) {
	list<neighborT_>::iterator itl = neighborL.begin();
	list<neighborT_>::iterator end = neighborL.end();
    while (itl!=end){
		if (itl->id == nodeid) {
            itl->timestamp = NOW;
            itl->parent = ra_addr_;
            itl->sinkID = sinkID_;
			return;
		}
		itl++;
	}
}

double ChargingRT::getMinHop(int* sinkID, int* minid) {
	list<neighborT_>::iterator it = neighborL.begin();
	list<neighborT_>::iterator end = neighborL.end();
    double mincost = MAX_COST;
    *sinkID = sinkID_; *minid = -1;
	while (it!=end) {
        if (it->energy<1) {it++;continue;}
		    double cost = it->hopCnt + 1;
            if (cost < mincost) {
                mincost = cost;
                *minid = it->id;
                sinkID_ = it->sinkID;
                *sinkID = sinkID_;
                hopCnt_ = it->hopCnt+1;
            }
        it++;
	}
    myCost_ = mincost;
    return mincost;
}

double ChargingRT::getMinCost(int* sinkID, int* minid) {
	list<neighborT_>::iterator it = neighborL.begin();
	list<neighborT_>::iterator end = neighborL.end();
    double mincost = MAX_COST;
    *sinkID = sinkID_; *minid = -1;
        double leftratio = em()->energy()/MAX_ENERGY;
        double pktCost = 0.1;
        if ( leftratio > 0.93 || leftratio < 0.07) {pktCost = 1;}
        else if (leftratio > 0.85 || leftratio < 0.15) {pktCost = 0.5;}
        else pktCost = 0.1;
	while (it!=end) {
        if (it->energy<1) {it++;continue;}
        if (it->timestamp + 10.0*beaconInterval_ < NOW || it->parent == ra_addr_) {
            //printf("node %d it %d timestamp %f now %f\n", ra_addr_, it->id, it->timestamp, NOW);
            it++;
            continue;
        }
	    double cost = it->cost;// + pktCost*pow(10000, 1-em()->energy()/MAX_ENERGY);
        if (God::instance()->isSink(it->id)) {
            mincost = cost;
            *minid = it->id;
            sinkID_ = it->sinkID;
            *sinkID = sinkID_;
            hopCnt_ = it->hopCnt+1;
            break;
        }
        if (cost < mincost) {
            mincost = cost;
            *minid = it->id;
            sinkID_ = it->sinkID;
            *sinkID = sinkID_;
            hopCnt_ = it->hopCnt+1;
        }
        it++;
	}
    myCost_ =  mincost+ pktCost*pow(10000, 1-em()->energy()/MAX_ENERGY);//mincost;
    return mincost;
}

double ChargingRT::getMinDist(int* sinkID, int* minid) {
    list<neighborT_>::iterator it = neighborL.begin();
    list<neighborT_>::iterator end = neighborL.end();
    double mincost = MAX_COST;
    *sinkID = sinkID_; *minid = -1;
    while (it!=end) {
        if (it->energy<1) {it++;continue;}
        if (it->timestamp + 10.0*beaconInterval_ < NOW || it->parent == ra_addr_) {
            //printf("node %d it %d timestamp %f now %f\n", ra_addr_, it->id, it->timestamp, NOW);
            it++;
            continue;
        }
        double ndis = it->cost + pow(it->x-mnode->X(),2)+pow(it->y-mnode->Y(),2);

        if (God::instance()->isSink(it->id)) {
            mincost = ndis;
            *minid = it->id;
            sinkID_ = it->sinkID;
            *sinkID = sinkID_;
            hopCnt_ = it->hopCnt+1;
            break;
        }
        if (ndis+0.1 < mincost) {
            mincost = ndis;
            *minid = it->id;
            sinkID_ = it->sinkID;
            *sinkID = sinkID_;
            hopCnt_ = it->hopCnt+1;
        }
	    it++;
    }
    myCost_ = mincost;
    return mincost;
}

int ChargingRT::getNextHop(int destid, int* sinkID,Packet *p) {
	struct hdr_ip *ih = HDR_IP(p);
	struct hdr_cmn *ch = HDR_CMN(p);
	if (destid == ra_addr_) return -1;
	if (God::instance()->isSink(ra_addr_)) {
    //if (ra_addr_==0) {
        *sinkID = ra_addr_;
        return ra_addr_;
    }
    int nexthop = -1;
    if (routingType_ == 1) {
        getMinCost(sinkID, &nexthop);
    }
    else if (routingType_ == 2) {
        getMinDist(sinkID, &nexthop);
    }
    else if (routingType_ == 3) {
        nexthop = ra_addr_-1;
    }
    else if (routingType_ == 4) {
        nexthop = 0;    
    }
    else if (routingType_ == 5) {
        nexthop = parent_id[ra_addr_];
    }
    else {
        printf("routingType_ %d\n",routingType_);
        nexthop = -1;
    }
    printf("node %d dest %d nexthop %d\n", ra_addr_, *sinkID, nexthop);
	return nexthop;
}
 
void ChargingRT::recv(Packet* p, Handler* h) {
	struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_ip *ih = HDR_IP(p);
	// it's a routing message, not an application message
	if(ch->ptype() == PT_CHARGINGRT) {
		recv_chargingrt_pkt(p);
		Packet::free(p);
		return;
	}
	//  Must be a packet I'm originating...
	if((ih->saddr() == ra_addr_) && (ch->num_forwards() == 0)) {
		ch->size() += IP_HDR_LEN;
		if ( (u_int32_t)ih->daddr() != IP_BROADCAST) {
			ih->ttl_ = 30;
		}
	}
	//  I received a packet that I sent, viewed as a routing loop.
	else if(ih->saddr() == ra_addr_) {
        evictNeighbor(parentID_);
		Packet::free(p);
		return;
	 }
	//  Packet I need to forward...
	 else {
		if(--ih->ttl_ == 0) {
		    Packet::free(p);
		    return;
		}
	 }
     updateNeighborTime(ch->prev_hop_);
     if (parentID_ == ch->prev_hop_ && (!God::instance()->isSink(ra_addr_))) {
        evictNeighbor(ch->prev_hop_);
     }

	 if ( (u_int32_t)ih->daddr() != IP_BROADCAST)
	   forward(1, p, 0.0);
	 else
	   forward(0, p, 0.0);
}

void ChargingRT::forward(int rt, Packet *p, double delay) {
	struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_ip *ih = HDR_IP(p);

	if(ih->ttl_ == 0) {
		Packet::free(p);
		return;
	}
	// recv a packet to me, either dest to me, or broadcasts
	if ((ih->daddr() == here_.addr_|| ih->daddr() == (nsaddr_t)IP_BROADCAST) &&
			(ch->ptype() != PT_CHARGINGRT) && ch->direction() == hdr_cmn::UP) {
		dmux_->recv(p,0);
		return;
	}
#if 0
	// for snoop purpose, its a forwarding unicast message
	if (ih->daddr() != here_.addr_ && ih->daddr() != (nsaddr_t)IP_BROADCAST &&
		ih->saddr() != here_.addr_ && ch->next_hop_ == here_.addr_&&
		(ch->ptype() != PT_NRRROUTING) && ch->direction() == hdr_cmn::UP) {
		dmux_->recv(p,0);
	}
#endif
	if (rt) {
        int sinkid = God::instance()->sink_num();
		int nexthop = getNextHop(ih->daddr(), &sinkid, p);
		if (nexthop < 0) {
            //printf("no route from node %d to node %d\n", ih->saddr(), ih->daddr());
			Packet::free(p);
			return;
		}
        else {
            if (nexthop != parentID_) {
            	//God::instance()->ctrace()->log("node %d new parent %d sinkid %d eng %f @ %f\n", here_.addr_, nexthop, sinkid, em()->energy(), NOW);
            }
            parentID_ = nexthop;
        }
        ih->daddr() = sinkid;
		ch->next_hop_ = nexthop;
		ch->addr_type() = NS_AF_INET;
	}
	else { // if it is a broadcast packet
		assert(ih->daddr() == (nsaddr_t)IP_BROADCAST);
		ch->addr_type() = NS_AF_NONE;
	}
	ch->direction() = hdr_cmn::DOWN;
	ch->num_forwards()++;
	ch->xmit_failure_ = 0;
	ch->xmit_failure_data_ = 0;
    ch->prev_hop_ = ra_addr_;
    packetSent_++;
    Scheduler::instance().schedule(target_, p, 0.001 * Random::uniform());
}

void ChargingRT::recv_chargingrt_pkt(Packet* p) {
	struct hdr_ip *ih = HDR_IP(p);
	if (ih->daddr() == (nsaddr_t)IP_BROADCAST) {
		if (ih->saddr()!=here_.addr_) {
			PacketData* pp = (PacketData*)p->userdata();
			BeaconMsgT_* bmsg = (BeaconMsgT_*)pp->data();
			//if (bmsg->seq>curBseq && bmsg->time>curBtime) {
                //printf("node %d recv beacon msg from %d seq %d\n", ra_addr_, ih->saddr(), bmsg->seq);
				updateNeightorTable(bmsg->myinfo);
				return;
			//}
		}
	}
	return;
}

void ChargingRT::send_chargingrt_pkt() {
	return;
}

void ChargingRT::sendBeacon(){
	static int seq;
	BeaconMsgT_ bmsg;
	PacketData* data = new PacketData(sizeof(BeaconMsgT_));
	bmsg.seq = seq++;
	bmsg.time = NOW;
	bmsg.myinfo.id=mnode->nodeid();
	bmsg.myinfo.x=mnode->X();
	bmsg.myinfo.y=mnode->Y();
    bmsg.myinfo.energy = em()->energy();
	if (God::instance()->isSink(ra_addr_)) {
        myCost_ = 0;
        hopCnt_ = 0;
        sinkID_ = ra_addr_;
        parentID_ = ra_addr_;
    }
    else {
        if (sinkID_ == 0) {
                sinkID_ = God::instance()->sink_num();
        }
    }
    bmsg.myinfo.cost = myCost_;
    bmsg.myinfo.sinkID = sinkID_;
    bmsg.myinfo.hopCnt = hopCnt_;
    bmsg.myinfo.timestamp = NOW;
    bmsg.myinfo.parent = parentID_;
	memcpy(data->data(), &bmsg, sizeof(bmsg));

	Packet *p = allocpkt();
	hdr_cmn *ch = hdr_cmn::access(p);
	ch->size() = sizeof(bmsg);
	ch->timestamp() = (u_int32_t)(NOW);
	ch->ptype() = PT_CHARGINGRT;
	ch->num_forwards() = 0;
    ch->next_hop_ = (nsaddr_t)IP_BROADCAST;
	ch->addr_type() = NS_AF_NONE;
	ch->direction() = hdr_cmn::DOWN;       //important: change the packet's direction
	ch->size() += IP_HDR_LEN;
	ch->xmit_failure_ = 0;
	ch->xmit_failure_data_ = 0;

	hdr_ip *iph = hdr_ip::access(p);
	iph->ttl() = 100;
	iph->saddr() = here_.addr_;
	iph->daddr() = IP_BROADCAST;
	iph->sport() = RT_PORT;
	iph->dport() = RT_PORT;
    //printf("node %d send beacon msg seq %d\n", ra_addr_, bmsg.seq);
	p->setdata(data);
    packetSent_++;
	Scheduler::instance().schedule(target_, p, CHARGINGJITTER);
	btimer->resched(beaconInterval_+CHARGINGJITTER);
}


void RTBeaconTimer::expire(Event* e) {
	rtinstance_->sendBeacon();
}



