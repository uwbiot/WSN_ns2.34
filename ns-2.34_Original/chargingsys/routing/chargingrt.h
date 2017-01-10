/*
 * nrrrouting.h
 *
 *  Created on: Sep 21, 2009
 *      Author: ypeng
 */

#ifndef CHARGINGRT_H
#define CHARGINGRT_H




#include <list>
#include <iostream>
#include <limits>

#include "chargingrt_pkt.h"
#include "agent.h"
#include "packet.h"
#include "trace.h"
#include "timer-handler.h"
#include "random.h"
#include "classifier-port.h"
#include "address.h"
#include "config.h"
#include "mobilenode.h"
#include "god.h"

#include "chargingcommon.h"

class ChargingRT;             // forward declaration

#define MAX_ENERGY 10000.0
#define MAX_COST 1000000.0

typedef struct neighbor {
	int id;
    int sinkID;
    int hopCnt;
    int parent;
	double x;
	double y;
    double energy;
    double cost;
    double timestamp;
	neighbor():id(0),sinkID(0),hopCnt(0xffff),x(0.0),y(0.0),energy(MAX_ENERGY), cost(MAX_COST), timestamp(0){}
}neighborT_;

class RTBeaconTimer : public TimerHandler {
public:
	RTBeaconTimer(ChargingRT* rt) : rtinstance_(rt) {}
protected:
	void expire(Event*);
	ChargingRT* rtinstance_;
};

class ChargingMsgHandle: public Handler {
public:
	virtual void handle(Event* event) {
        printf("message send done\n");
    }
};

typedef struct BeaconMsg {
	int seq;
	double time;
	neighborT_ myinfo;
	BeaconMsg():seq(0),time(0),myinfo(){}
}BeaconMsgT_;


/* Agent */
class ChargingRT : public Agent {
public:
    ChargingRT(nsaddr_t);
    ~ChargingRT();
    int  	command(int, const char*const*);
    void 	recv(Packet*, Handler*);
    void	sendBeacon();
protected:
    PortClassifier*     dmux_;      // For passing packets up to agents.
    Trace*              logtarget_; // For logging.
    inline 	nsaddr_t&         ra_addr()        { return ra_addr_; }
    void   	recv_chargingrt_pkt(Packet*);
    void   	send_chargingrt_pkt();
    int 	getNextHop(int destid, int* sinkID,Packet *p);
    void 	forward(int rt, Packet *p, double delay);
    void 	updateNeightorTable(neighborT_&);
    double  getMinCost(int* sinkID, int* minid);
    double  getMinDist(int* sinkID, int* minid);
    double  getMinHop(int* sinkID, int* minid);
    void    evictNeighbor(int nodeid);
    void    updateNeighborTime(int nodeid);
private:
    nsaddr_t            ra_addr_;
    u_int32_t           seq_num_;
    u_int32_t 			packetSent_;
    int32_t             parentID_;
    int32_t             sinkID_;
    int32_t             hopCnt_;
    double              myCost_;
    int32_t             routingType_;
    int32_t             beaconInterval_;
    RTBeaconTimer*      btimer;
    MobileNode*         mnode;
    std::list<neighborT_> neighborL;
    inline EnergyModel* em() {return mnode->energy_model(); }
};

#endif /* NRRROUTING_H_ */
