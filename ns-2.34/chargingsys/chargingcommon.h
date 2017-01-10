/*
 * nrrcommon.h
 *
 *  Created on: Sep 1, 2009
 *      Author: ypeng
 */

#ifndef CHARGINGCOMMON_H
#define CHARGINGCOMMON_H

// c++ header
#include <iostream>
#include <list>
#include <set>

// ns2 header
#include "scheduler.h"
#include "app.h"
#include "timer-handler.h"
#include "node.h"
#include "mobilenode.h"
#include "random.h"

#include "god.h"

// defines


//#define NOWTIME Scheduler::instance().clock()
#define CHARGINGJITTER  (Random::uniform()*1.1)

enum configuration {
	UCAST = 0xfff0,
    MCAST = 0xfffe,
    BCAST = 0xffff,
    INVALID = 0xff00,
    SCALE = 200,
    BASESTATION = 0,
	MAX_DATA_SIZE = 16,
	DTIMER = 1,
	STIMER = 2,
    CTIMER = 3,
};
// enums
enum nodeRole {
	BACKUP,
	PRIME,
};
enum nodeStatus {
	SLEEP,
	ACTIVE,
	DEAD,
};
enum msgType {
	REQ_READY,
	REQ_DEADLINE,
	DATA_ENERGY,
	DATA_SENSING,
	CTRL_LOWENG,
	CTRL_CHANGEROLE,
	CTRL_CHANGESTATUS,
	CTRL_QUERY,
	CTRL_INFO,
};

class ChargingApp;

struct setInfo {
	int setID;
	int role;
	double energyLeft;
};

typedef struct CAppMessage {
	CAppMessage():source(0), dest(0), netType(0), msgType(0), seq(0),
			len(0), timestamp(0){}
	int source;		// source node id
	int dest;		// destination node id
	int netType; 	// network message type: unicast, broadcast, multicast
	int msgType;	// message type, see msgType enum
	int seq;		// sequence number
	int len;		// length of the data set
	double timestamp;	// time stamp of this message
	int data[MAX_DATA_SIZE];		// data should be array
}CAppMessageT_;

typedef struct CTRLData{
	int setid;
	int areaid;
	int nodeid;
	double currentlifetime;
}CTRLDataT_;

#endif


